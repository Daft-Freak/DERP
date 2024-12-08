#include <cstdio>
#include <cstring>
#include <string>
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

#include "GPIO.h"
#include "SUMPServer.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::SUMP;

enum class SUMPCommand
{
    // short (one byte)
    Reset             = 0x00,
    Run               = 0x01,
    ID                = 0x02,
    XON               = 0x11,
    XOFF              = 0x13,

    // long (5 bytes)
    SetDivider        = 0x80,
    SetReadDelayCount = 0x81,
    SetFlags          = 0x82,
    SetDelayCount     = 0x83,
    SetReadCount      = 0x84,
    SetTriggerMask    = 0xC0,
    SetTriggerValues  = 0xC1,
    SetTriggerConfig  = 0xC2,

    // extended (OLS)
    GetMetadata       = 0x04,
};

SUMPServer::SUMPServer(uint16_t port) : port(port)
{
}

SUMPServer::~SUMPServer()
{
    delete[] sampleBuffer;
}

bool SUMPServer::start()
{
    if(listenFd != -1)
    {
        logf(LogLevel::Error, logComponent, "Failed to start: already started!");
        return false;
    }

    if((listenFd = socket(AF_INET6, SOCK_STREAM, 0)) == -1)
    {
        logf(LogLevel::Error, logComponent, "Failed to start: couldn't create socket!");
        return false;
    }
    
    int yes = 1;

    if(setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&yes), sizeof(int)) == -1)
    {
        close(listenFd);
        return false;
    }

    // allow IPv4 connections
    yes = 0; // no
    if(setsockopt(listenFd, IPPROTO_IPV6, IPV6_V6ONLY, reinterpret_cast<char *>(&yes), sizeof(int)) == -1)
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
        logf(LogLevel::Error, logComponent, "Failed to start: couldn't bind socket!");
        close(listenFd);
        return false;
    }

    if(listen(listenFd, 1) == -1)
    {
        logf(LogLevel::Error, logComponent, "Failed to start: couldn't listen on socket!");
        close(listenFd);
        return false;
    }

    return true;
}

void SUMPServer::stop()
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

bool SUMPServer::update(bool block)
{
    fd_set set;

    FD_ZERO(&set);
    // check client if connected, otherwise listen
    int fd = clientFd == -1 ? listenFd : clientFd;

    FD_SET(fd, &set);

    timeval timeout = {0, 0};

    // probably should take a timeout here
    if(block)
        timeout.tv_usec = 10 * 1000; // 10ms

    int ready = select(fd + 1, &set, nullptr, nullptr, &timeout);

    if(ready > 0)
    {
        if(FD_ISSET(listenFd, &set))
        {
            // accept
            sockaddr_storage remoteAddr;
            socklen_t addrLen = sizeof(remoteAddr);
            auto remoteSockaddr = reinterpret_cast<sockaddr *>(&remoteAddr);

            int newFd = accept(listenFd, remoteSockaddr, &addrLen);

            if(newFd == -1)
                return false;
            else
            {
                char hoststr[NI_MAXHOST];
                char portstr[NI_MAXSERV];

                int rc = getnameinfo(remoteSockaddr, addrLen, hoststr, sizeof(hoststr), portstr, sizeof(portstr), NI_NUMERICHOST | NI_NUMERICSERV);

                if(rc == 0)
                    logf(LogLevel::Info, logComponent, "new connection from %s port %s", hoststr, portstr);

                int yes = 1;
                setsockopt(newFd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char *>(&yes), sizeof(int));

                clientFd = newFd;
            }
        }
        
        if(FD_ISSET(clientFd, &set))
        {
            std::lock_guard lock(clientMutex);

            // read
            uint8_t cmd;
            auto received = recv(clientFd, reinterpret_cast<char *>(&cmd), 1, 0);

            if(received > 0)
            {
                switch(static_cast<SUMPCommand>(cmd))
                {
                    case SUMPCommand::Reset:
                        running = false;
                        sampleOffset = 0;
                        break;

                    case SUMPCommand::Run:
                        logf(LogLevel::Debug, logComponent, "run!");
                        running = true;
                        // should sync IO, but this is on a thread
                        break;

                    case SUMPCommand::ID:
                    {
                        size_t len = 4;
                        if(!sendAll(clientFd, "1ALS", len, 0) || len != 4)
                            logf(LogLevel::Error, logComponent, "failed to send id!");
                        break;
                    }

                    case SUMPCommand::GetMetadata:
                    {
                        uint32_t sampleMem = 1 * 1024 * 1024 * 1024; // 200M samples is probably enough
                        uint32_t freq = 125 * 1000 * 1000; // TODO: this should be sysclk

                        static const uint8_t metadata[]
                        {
                            0x01, 'D', 'E', 'R', 'P', ' ', 'G', 'P', 'I', 'O', 0, // name
                            0x21, uint8_t(sampleMem >> 24), uint8_t(sampleMem >> 16), uint8_t(sampleMem >> 8), uint8_t(sampleMem), // sample memory
                            0x23, uint8_t(freq >> 24), uint8_t(freq >> 16), uint8_t(freq >> 8), uint8_t(freq), // max sample rate
                            0x40, 30, // num probes
                            0x41, 2, // protocol version
                            0x00,
                        };

                        size_t len = sizeof(metadata);
                        if(!sendAll(clientFd, metadata, len, 0) || len != sizeof(metadata))
                            logf(LogLevel::Error, logComponent, "failed to send metadata!");
                        break;
                    }

                    case SUMPCommand::SetDivider:
                    {
                        uint32_t data;
                        if(recv(clientFd, reinterpret_cast<char *>(&data), 4, 0) != 4)
                            logf(LogLevel::Error, logComponent, "failed to recv data for set divider!");
                        else
                        {
                            divider = (data & 0xFFFFFF) + 1;
                            logf(LogLevel::Debug, logComponent, "set divider = %u", divider);
                        }
                        break;
                    }
                    case SUMPCommand::SetReadDelayCount:
                    {
                        uint32_t data;
                        if(recv(clientFd, reinterpret_cast<char *>(&data), 4, 0) != 4)
                            logf(LogLevel::Error, logComponent, "failed to recv data for set read/delay count!");
                        else
                        {
                            readCount = ((data & 0xFFFF) + 1) * 4;
                            delayCount = ((data >> 16) + 1) * 4;
                            delete[] sampleBuffer;
                            sampleBuffer = new uint32_t[readCount];
                            logf(LogLevel::Debug, logComponent, "set read count = %u, delay count = %u", readCount, delayCount);
                        }
                        break;
                    }

                    case SUMPCommand::SetDelayCount:
                    {
                        uint32_t data;
                        if(recv(clientFd, reinterpret_cast<char *>(&data), 4, 0) != 4)
                            logf(LogLevel::Error, logComponent, "failed to recv data for set delay count!");
                        else
                        {
                            delayCount = (data + 1) * 4;
                            logf(LogLevel::Debug, logComponent, "set delay count = %u", delayCount);
                        }
                        break;
                    }
                    case SUMPCommand::SetReadCount:
                    {
                        uint32_t data;
                        if(recv(clientFd, reinterpret_cast<char *>(&data), 4, 0) != 4)
                            logf(LogLevel::Error, logComponent, "failed to recv data for set read count!");
                        else
                        {
                            readCount = (data + 1) * 4;
                            delete[] sampleBuffer;
                            sampleBuffer = new uint32_t[readCount];
                            logf(LogLevel::Debug, logComponent, "set read count = %u", readCount);
                        }
                        break;
                    }

                    case SUMPCommand::SetFlags:
                    case SUMPCommand::SetTriggerMask:
                    case SUMPCommand::SetTriggerValues:
                    case SUMPCommand::SetTriggerConfig:
                    {
                        uint32_t data;
                        if(recv(clientFd, reinterpret_cast<char *>(&data), 4, 0) != 4)
                            logf(LogLevel::Error, logComponent, "failed to recv data for cmd %02X!", cmd);
                        else
                        {
                            logf(LogLevel::NotImplemented, logComponent, "cmd %02X data %08X", cmd, data);
                        }
                        break;
                    }

                    default:
                        logf(LogLevel::NotImplemented, logComponent, "cmd %02X", cmd);
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
void SUMPServer::setGPIO(GPIO *gpio)
{
    this->gpio = gpio;
    gpio->setUpdateCallback([this](auto time, auto &gpio, auto elapsed){onGPIOUpdate(time, gpio, elapsed);});
}

bool SUMPServer::sendAll(int fd, const void *data, size_t &len, int flags)
{
    size_t total_sent = 0;
    size_t to_send = len;
    int sent = 0;

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

int SUMPServer::close(int fd)
{
#ifdef _WIN32
    return closesocket(fd);
#else
    return ::close(fd);
#endif
}

void SUMPServer::onGPIOUpdate(uint64_t time, GPIO &gpio, uint32_t elapsedCycles)
{
    if(!running || sampleOffset >= readCount)
        return;

    // TODO: triggers, delay
    
    while(elapsedCycles)
    {
        auto step = std::min(divCounter, elapsedCycles);

        sampleBuffer[sampleOffset++] = gpio.getPadState();

        elapsedCycles -= step;

        divCounter -= step;
        if(divCounter == 0)
            divCounter = divider;

        if(sampleOffset == readCount)
        {
            std::lock_guard lock(clientMutex);

            // filled buffer, send data
            size_t len = readCount * 4;
            sendAll(clientFd, sampleBuffer, len, 0);
            logf(LogLevel::Debug, logComponent, "sent %zu/%u\n", len, readCount * 4);
            break;
        }
    }
}
