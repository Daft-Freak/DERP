#include <charconv>
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

#include "GDBServer.h"
#include "ARMv6MCore.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::GDB;

static void byteToHex(char *out, uint8_t byte)
{
    int hi = byte / 16;
    int lo = byte % 16;

    out[0] = hi < 10 ? ('0' + hi) : ('A' + hi - 10);
    out[1] = lo < 10 ? ('0' + lo) : ('A' + lo - 10);
}

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

bool GDBServer::update(bool block)
{
    if(cpuMutex.try_lock())
    {
        // TODO: threads
        if(cpus && cpus[0].debugHalted && !cpuHalted && clientFd != -1)
        {
            // cpu probably hit a breakpoint
            sendReply(clientFd, "S05", 3); // It's a trap!
            // TODO: something like T05thread:n;hwbreak ?
            cpuHalted = true;
        }
        cpuMutex.unlock();
    }

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

                // halt the CPUs on connection
                haltCPUs();
                setAttachedToCPUs(true);
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
                    case 3: // ctrl-c
                        haltCPUs();
                        sendReply(clientFd, "S05", 3); // It's a trap!
                        break;

                    case '+': // pos ack
                        break;
                    case '-': // neg ack
                        logf(LogLevel::Debug, logComponent, "ack %c", c);
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
                        received = recv(clientFd, cs, 2, MSG_WAITALL);

                        if(received == 2)
                        {
                            command.back() = 0; // replace # with null

                            // verify checksum
                            uint8_t checksum = 0;
                            std::from_chars(cs, cs + 2, checksum, 16);

                            uint8_t calcChecksum = 0;
                            for(auto &cmdChar : command)
                                calcChecksum += cmdChar;

                            if(calcChecksum != checksum)
                                return sendNegAck(clientFd);

                            // handle command
                            std::string_view commandStr(command.data(), command.size() - 1);

                            if(commandStr == "?") // initial halt reason
                                sendReply(clientFd, "S05", 3); // It's a trap!
                            else if(commandStr == "c") // continue
                                return handleContinue(clientFd);
                            else if(commandStr == "g") // read regs
                                return handleReadRegisters(clientFd);
                            else if(commandStr[0] == 'G') // write regs
                                return handleWriteRegisters(clientFd, commandStr);
                            else if(commandStr[0] == 'm') // read mem
                                return handleReadMemory(clientFd, commandStr);
                            else if(commandStr[0] == 'M') // write mem
                                return handleWriteMemory(clientFd, commandStr);
                            else if(commandStr[0] == 'Z') // add breakpoint
                                return handleAddBreakpoint(clientFd, commandStr);
                            else if(commandStr[0] == 'z') // remove breakpoint
                                return handleRemoveBreakpoint(clientFd, commandStr);
                            else if(commandStr.compare(0, 6, "qRcmd,") == 0) // commands
                                return handleCommand(clientFd, commandStr);
                            else if(commandStr.compare(0, 11, "qSupported:") == 0)
                                return sendReply(clientFd, "qXfer:memory-map:read+", 22);
                            else if(commandStr.compare(0, 6, "qXfer:") == 0)
                                return handleXfer(clientFd, commandStr);
                            else if(commandStr.compare(0, 12, "vFlashErase:") == 0)
                                return handleFlashErase(clientFd, commandStr);
                            else if(commandStr.compare(0, 12, "vFlashWrite:") == 0)
                                return handleFlashWrite(clientFd, commandStr);
                            else if(commandStr == "vFlashDone")
                                return sendReply(clientFd, "OK", 2);
                            else
                            {
                                logf(LogLevel::NotImplemented, logComponent, "command %s cs %02X / %02X", command.data(), checksum, calcChecksum);
                                return sendEmptyReply(clientFd);
                            }
                        }

                        break;
                    }
                    
                    default:
                        logf(LogLevel::NotImplemented, logComponent, "char %c", c);
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

void GDBServer::haltCPUs()
{
    std::lock_guard lock(cpuMutex);

    for(size_t i = 0; i < numCPUs; i++)
        cpus[i].debugHalted = true;

    cpuHalted = true;
}

void GDBServer::setAttachedToCPUs(bool attached)
{
    std::lock_guard lock(cpuMutex);

    for(size_t i = 0; i < numCPUs; i++)
        cpus[i].debuggerAttached = attached;
}

bool GDBServer::handleContinue(int fd)
{
    std::lock_guard lock(cpuMutex);

    for(size_t i = 0; i < numCPUs; i++)
        cpus[i].debugHalted = false;

    cpuHalted = false;

    return sendPosAck(fd); // will reply when we stop
}

bool GDBServer::handleReadRegisters(int fd)
{
    std::lock_guard lock(cpuMutex);

    const int numRegs = 16, numFPARegs = 8, numFlagsRegs = 2;
    char reply[(numRegs * 4 + numFPARegs * 12 + numFlagsRegs * 4) * 2 + 1]; // * 2 chars per byte

    int cpsrOff = numRegs * 8 + numFPARegs * 24 + 8; // second flags reg

    // r0-15
    auto regs = cpus[0].regs; // TODO: threads
    for(int i = 0; i < 16; i++)
    {
        auto r = regs[i];

        if(i == 15)
            r -= 2;

        auto swapped = r >> 24 | r << 24 | (r & 0xFF0000) >> 8 | (r & 0xFF00) << 8;
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

bool GDBServer::handleWriteRegisters(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    const int numRegs = 16, numFPARegs = 8, numFlagsRegs = 2;
    const int expectedLen = (numRegs * 4 + numFPARegs * 12 + numFlagsRegs * 4) * 2;
    int cpsrOff = numRegs * 8 + numFPARegs * 24 + 8; // second flags reg

    if(command.length() - 1 != expectedLen)
        return sendEmptyReply(fd);

    // parse r0-15
    auto regs = cpus[0].regs; // TODO: threads

    auto p = command.data() + 1;
    uint32_t val;

    for(int i = 0; i < 16; i++)
    {
        auto res = std::from_chars(p, p + 8, val, 16);

        if(res.ec != std::errc{})
            break;

        auto swapped = val >> 24 | val << 24 | (val & 0xFF0000) >> 8 | (val & 0xFF00) << 8;

        regs[i] = swapped;

        p = res.ptr;
    }

    // parse CPSR
    p = command.data() + 1 + cpsrOff;
    auto res = std::from_chars(p, p + 8, val, 16);

    if(res.ec == std::errc{})
    {
        auto swapped = val >> 24 | val << 24 | (val & 0xFF0000) >> 8 | (val & 0xFF00) << 8;
        cpus[0].cpsr = swapped;
    }

    return sendReply(fd, "OK", 2);
}

bool GDBServer::handleReadMemory(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    // parse
    uint32_t addr, len;

    // address
    auto res = std::from_chars(command.data() + 1, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    // length
    res = std::from_chars(res.ptr + 1, command.data() + command.length(), len, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    // read
    auto reply = new char[len * 2];

    int cycles;
    for(uint32_t i = 0; i < len; i++)
    {
        auto b = cpus[0].readMem8(addr + i, cycles); // TODO: more direct?
        byteToHex(reply + i * 2, b);
    }

    bool ret = sendReply(fd, reply, len * 2);

    delete[] reply;

    return ret;
}

bool GDBServer::handleWriteMemory(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    // parse
    uint32_t addr, len;

    // address
    auto res = std::from_chars(command.data() + 1, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    // length
    res = std::from_chars(res.ptr + 1, command.data() + command.length(), len, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    if(*res.ptr != ':')
        return sendEmptyReply(fd); // E?

    res.ptr++;

    // write
    int cycles;
    uint8_t b[4];

    for(uint32_t i = 0; i < len; i++)
    {
        // write a word if possible
        if(len - i >= 4 && !((addr + i) & 3))
        {
            for(int j = 0; j < 4; j++) {
                res = std::from_chars(res.ptr, res.ptr + 2, b[j], 16);

                if(res.ec != std::errc{})
                    return sendEmptyReply(fd); // should probably send whatever error is appropriate...
            }

            cpus[0].writeMem32(addr + i, *(uint32_t *)b, cycles);
            i += 3;
            continue;
        }

        res = std::from_chars(res.ptr, res.ptr + 2, b[0], 16);

        if(res.ec != std::errc{})
            return sendEmptyReply(fd); // should probably send whatever error is appropriate...

        cpus[0].writeMem8(addr + i, b[0], cycles); // TODO: more direct?
    }

    return sendReply(fd, "OK", 2);
}

bool GDBServer::handleAddBreakpoint(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    int type = command[1] - '0';

    if(type != 0 && type != 1) // software or hardware breakpoint
        return sendEmptyReply(fd);

    // parse
    uint32_t addr;

    // address
    auto res = std::from_chars(command.data() + 3, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    // TODO: threads
    cpus[0].breakpoints.insert(addr);

    return sendReply(fd, "OK", 2);
}

bool GDBServer::handleRemoveBreakpoint(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    int type = command[1] - '0';

    if(type != 0 && type != 1) // software or hardware breakpoint
        return sendEmptyReply(fd);

    // parse
    uint32_t addr;

    // address
    auto res = std::from_chars(command.data() + 3, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);
    
    // TODO: threads
    cpus[0].breakpoints.erase(addr);

    return sendReply(fd, "OK", 2);
}

bool GDBServer::handleCommand(int fd, std::string_view command)
{
    std::string decCommand((command.length() - 6) / 2, '\0');

    auto p = command.data() + 6;
    auto end = command.data() + command.length();

    int off = 0;
    while(p < end)
    {
        char c;
        auto res = std::from_chars(p, p + 2, c, 16);

        if(res.ec != std::errc{})
            break;

        decCommand[off++] = c;

        p = res.ptr;
    }

    if(decCommand == "reset halt" || decCommand == "reset init")
    {
        std::lock_guard lock(cpuMutex);

        cpus[0].getMem().reset();
        haltCPUs();
        return sendReply(fd, "OK", 2);
    }

    logf(LogLevel::NotImplemented, logComponent, "qRcmd: %.*s", int(decCommand.length()), decCommand.data());
    return sendEmptyReply(fd);
}

bool GDBServer::handleXfer(int fd, std::string_view command)
{
    // parse
    size_t prevColon = 5;
    auto colon = command.find_first_of(':', prevColon + 1);
    if(colon == std::string_view::npos)
        return sendReply(fd, "E00", 3);

    auto object = command.substr(prevColon + 1, colon - prevColon - 1);

    prevColon = colon;
    colon = command.find_first_of(':', prevColon + 1);
    if(colon == std::string_view::npos)
        return sendReply(fd, "E00", 3);

    auto operation = command.substr(prevColon + 1, colon - prevColon - 1);

    prevColon = colon;
    colon = command.find_first_of(':', prevColon + 1);
    if(colon == std::string_view::npos)
        return sendReply(fd, "E00", 3);

    auto annex = command.substr(prevColon + 1, colon - prevColon - 1);

    prevColon = colon;

    // offset
    uint32_t offset, length;
    auto res = std::from_chars(command.data() + prevColon + 1, command.data() + command.length(), offset, 16);

    if(res.ec != std::errc{})
        return sendReply(fd, "E00", 3);

    // length
    res = std::from_chars(res.ptr + 1, command.data() + command.length(), length, 16);

    if(res.ec != std::errc{})
        return sendReply(fd, "E00", 3);

    if(operation == "read")
    {
        auto cmdEnd = command.data() + command.length();
        if(res.ptr != cmdEnd)
            return sendReply(fd, "E00", 3);

        if(object == "memory-map")
        {
            if(annex != "")
                return sendReply(fd, "E00", 3);

            // minimal map with only flash region
            static const char *memoryMap = R"(<?xml version="1.0"?>
<!DOCTYPE memory-map PUBLIC "+//IDN gnu.org//DTD GDB Memory Map V1.0//EN" "http://sourceware.org/gdb/gdb-memory-map.dtd">
<memory-map>
    <memory type="flash" start="0x10000000" length="0x1000000">
        <property name="blocksize">0x1000</property>
    </memory>
    <memory type="ram" start="0x11000000" length="0xef000000"/>
</memory-map>
            )";
            static const size_t memoryMapLen = strlen(memoryMap);

            size_t replyLen = 0;

            if(offset < memoryMapLen)
                replyLen = length < memoryMapLen ? length : memoryMapLen;

            if(!replyLen)
                return sendReply(fd, "l", 1); // no more data
            
            // reply m + data
            // TODO: skipping escapes due to no #$*} in the data
            auto replyBuf = new char[replyLen + 1];
            replyBuf[0] = 'm';
            memcpy(replyBuf + 1, memoryMap + offset, replyLen);

            auto ret = sendReply(fd, replyBuf, replyLen + 1);

            delete[] replyBuf;

            return ret;
        }
    }

    logf(LogLevel::NotImplemented, logComponent, "qXfer: %.*s %.*s %.*s off %u len %u", int(object.length()), object.data(), int(operation.length()), operation.data(), int(annex.length()), annex.data(), offset, length);

    return sendEmptyReply(fd);
}

bool GDBServer::handleFlashErase(int fd, std::string_view command)
{
    // parse
    uint32_t addr, len;

    // address
    auto res = std::from_chars(command.data() + 12, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    // length
    res = std::from_chars(res.ptr + 1, command.data() + command.length(), len, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    logf(LogLevel::Debug, logComponent, "flash erase %08X len %i", addr, len);
    
    // yep, we definitely erased it...
    return sendReply(fd, "OK", 2);
}

bool GDBServer::handleFlashWrite(int fd, std::string_view command)
{
    std::lock_guard lock(cpuMutex);

    // parse
    uint32_t addr, len = 0;

    // address
    auto res = std::from_chars(command.data() + 12, command.data() + command.length(), addr, 16);

    if(res.ec != std::errc{})
        return sendEmptyReply(fd);

    if(addr < 0x10000000 || addr >= 0x11000000)
        return sendReply(fd, "E.memtype", 9);

    // write
    auto p = res.ptr + 1;
    auto end = command.data() + command.length();

    auto out = cpus[0].getMem().mapAddress(addr);

    while(p != end)
    {
        uint8_t b = *p++;

        if(b == '}')
            b = *p++ ^ 0x20;

        *out++ = b;

        len++;
    }

    logf(LogLevel::Debug, logComponent, "flash write %08X len %i", addr, len);
    
    return sendReply(fd, "OK", 2);
}

// reply helpers
bool GDBServer::sendReply(int fd, const char *reply, size_t len)
{
    auto buf = new char[len + 5];

    buf[0] = '+'; // ack

    // reply data
    buf[1] = '$';
    memcpy(buf + 2, reply, len);

    // checksum
    uint8_t checksum = 0;
    for(size_t i = 0; i < len; i++)
        checksum += reply[i];

    auto off = len + 2;
    buf[off++] = '#';
    byteToHex(buf + off, checksum);
    off += 2;

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

bool GDBServer::sendPosAck(int fd)
{
    size_t len = 1;
    return sendAll(fd, "+", len, 0) && len == 1;
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

int GDBServer::close(int fd)
{
#ifdef _WIN32
    return closesocket(fd);
#else
    return ::close(fd);
#endif
}

