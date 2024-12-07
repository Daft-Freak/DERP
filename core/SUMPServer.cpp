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

SUMPServer::SUMPServer(uint16_t port) : port(port)
{
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
            // read
            uint8_t cmd;
            auto received = recv(clientFd, &cmd, 1, 0);

            if(received > 0)
            {
                switch(cmd)
                {
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

