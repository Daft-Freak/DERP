#pragma once

#include <cstdint>
#include <mutex>
#include <string_view>

class GPIO;
class SUMPServer final
{
public:
    SUMPServer(uint16_t port = 5555);

    bool start();
    void stop();

    bool update(bool block = false);

    void setGPIO(GPIO *gpio);

private:
    bool sendAll(int fd, const void *data, size_t &len, int flags);
    int close(int fd);

    uint16_t port;
    int listenFd = -1, clientFd = -1;

    GPIO *gpio = nullptr;
};