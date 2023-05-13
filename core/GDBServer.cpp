#include <charconv>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <vector>

#if defined(_WIN32)
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "GDBServer.h"
#include "ARMv6MCore.h"

GDBServer::GDBServer(uint16_t port) : port(port)
{
}

bool GDBServer::start()
{
    if(listenFd != -1)
        return false;

    if((listenFd = socket(AF_INET6, SOCK_STREAM, 0)) == -1)
        return false;
    
    int yes = 1;

    if(setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
    {
        close(listenFd);
        return false;
    }

    // allow IPv4 connections
    yes = 0; // no
    if(setsockopt(listenFd, IPPROTO_IPV6, IPV6_V6ONLY, &yes, sizeof(int)) == -1)
    {
        close(listenFd);
        return false;
    }

    // bind
    struct sockaddr_in6 addr = {};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port);

    if(inet_pton(AF_INET6, "::", &addr.sin6_addr) != 1)
        return false;

    if(bind(listenFd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        close(listenFd);
        return false;
    }

    if(listen(listenFd, 1) == -1)
    {
        close(listenFd);
        return false;
    }

    return true;
}

void GDBServer::stop()
{
    if(clientFd != -1)
    {
        close(clientFd);
        clientFd = -1;
    }

    if(listenFd != -1)
    {
        close(listenFd);
        listenFd = -1;
    }
}

bool GDBServer::update()
{
    fd_set set;

    FD_ZERO(&set);
    // check client if connected, otherwise listen
    int fd = clientFd == -1 ? listenFd : clientFd;

    FD_SET(fd, &set);

    timeval timeout = {0, 0};

    int ready = select(fd + 1, &set, nullptr, nullptr, &timeout);

    if(ready > 0)
    {
        if(FD_ISSET(listenFd, &set))
        {
            // accept
            sockaddr_storage remoteAddr;
            socklen_t addrLen = sizeof(remoteAddr);
            auto remoteSockaddr = reinterpret_cast<sockaddr *>(&remoteAddr);

            int fd = accept(listenFd, remoteSockaddr, &addrLen);

            if(fd == -1)
                return false;
            else
            {
                char hoststr[NI_MAXHOST];
                char portstr[NI_MAXSERV];

                int rc = getnameinfo(remoteSockaddr, addrLen, hoststr, sizeof(hoststr), portstr, sizeof(portstr), NI_NUMERICHOST | NI_NUMERICSERV);

                if(rc == 0)
                    printf("new connection from %s port %s\n", hoststr, portstr);

                int yes = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(int));

                clientFd = fd;
            }
        }
        
        if(FD_ISSET(clientFd, &set))
        {
            // read
            char c;
            auto received = recv(clientFd, &c, 1, 0);

            if(received > 0)
            {
                switch(c)
                {
                    case '+': // pos ack
                    case '-': // neg ack
                        printf("gdb: ack %c\n", c);
                        break;

                    case '$':
                    {
                        // read the command
                        std::vector<char> command;

                        do
                        {
                            received = recv(clientFd, &c, 1, 0);

                            if(received <= 0)
                                break;

                            command.push_back(c);
                        } while (c != '#');

                        char cs[2];
                        received = recv(clientFd, &cs, 2, MSG_WAITALL);

                        if(received == 2)
                        {
                            command.back() = 0; // replace # with null

                            // verify checksum
                            uint8_t checksum = 0;
                            std::from_chars(cs, cs + 2, checksum, 16);

                            uint8_t calcChecksum = 0;
                            for(auto &c : command)
                                calcChecksum += c;

                            if(calcChecksum != checksum)
                                return sendNegAck(clientFd);

                            // handle command
                            std::string_view commandStr(command.data());

                            if(commandStr == "?") // initial halt reason
                            {
                                // FIXME: probably should've halted the cpu...
                                sendReply(clientFd, "S05", 3); // It's a trap!
                            }
                            else if(commandStr == "g") // read regs
                                return handleReadRegisters(clientFd);
                            else
                            {
                                printf("gdb: %s cs %02X / %02X\n", command.data() - 1, checksum, calcChecksum);
                                return sendEmptyReply(clientFd);
                            }
                        }

                        break;
                    }
                    
                    default:
                        printf("gdb: %c\n", c);
                }
                
            }
            
            if(received <= 0)
            {
                // disconnect or error
                close(clientFd);
                clientFd = -1;
            }

        }
    }
    else if(ready != 0)
        return false;

    return true;
}

// cpu specific bits
void GDBServer::setCPUs(ARMv6MCore *cpus, size_t numCPUs)
{
    this->cpus = cpus;
    this->numCPUs = numCPUs;
}

bool GDBServer::handleReadRegisters(int fd)
{
    const int numRegs = 16, numFPARegs = 8, numFlagsRegs = 2;
    char reply[(numRegs * 4 + numFPARegs * 12 + numFlagsRegs * 4) * 2 + 1]; // * 2 chars per byte

    int cpsrOff = numRegs * 8 + numFPARegs * 24 + 8; // second flags reg

    // r0-15
    auto regs = cpus[0].regs; // TODO: threads
    for(int i = 0; i < 16; i++)
    {
        auto swapped = regs[i] >> 24 | regs[i] << 24 | (regs[i] & 0xFF0000) >> 8 | (regs[i] & 0xFF00) << 8;
        snprintf(reply + i * 8, 9/*+null*/, "%08X", swapped);
    }

    // fill fpa regs
    for(size_t i = 16 * 8; i < sizeof(reply) - 8; i++)
        reply[i] = 'x';
    
    // cpsr
    auto cpsr = cpus[0].cpsr;
    auto swapped = cpsr >> 24 | cpsr << 24 | (cpsr & 0xFF0000) >> 8 | (cpsr & 0xFF00) << 8;
    snprintf(reply + cpsrOff, 9/*+null*/, "%08X", swapped);

    return sendReply(fd, reply, sizeof(reply) - 1);
}

// reply helpers
bool GDBServer::sendReply(int fd, const char *reply, size_t len)
{
    auto buf = new uint8_t[len + 5];

    buf[0] = '+'; // ack

    // reply data
    buf[1] = '$';
    memcpy(buf + 2, reply, len);

    // checksum
    uint8_t checksum = 0;
    for(size_t i = 0; i < len; i++)
        checksum += reply[i];

    auto toHex = [](int i)
    {
        if(i < 10)
            return '0' + i;
        else
            return 'A' + i - 10;
    };

    auto off = len + 2;
    buf[off++] = '#';
    buf[off++] = toHex(checksum / 16);
    buf[off++] = toHex(checksum % 16);

    size_t sendLen = off;
    bool ret = sendAll(fd, buf, sendLen, 0) && sendLen == off;

    delete[] buf;

    return ret;
}

bool GDBServer::sendEmptyReply(int fd)
{
    size_t len = 5;
    return sendAll(fd, "+$#00", len, 0) && len == 5;
}

bool GDBServer::sendNegAck(int fd)
{
    size_t len = 1;
    return sendAll(fd, "-", len, 0) && len == 1;
}

bool GDBServer::sendAll(int fd, const void *data, size_t &len, int flags)
{
    size_t total_sent = 0;
    size_t to_send = len;
    ssize_t sent = 0;

    while(to_send)
    {
        sent = send(fd, reinterpret_cast<const char *>(data) + total_sent, to_send, flags);
        if(sent == -1)
            break;
        total_sent += sent;
        to_send -= sent;
    }

    len = total_sent;
    return sent != -1;
}

int GDBServer::close(int fd)
{
#ifdef _WIN32
    return closesocket(fd);
#else
    return ::close(fd);
#endif
}

