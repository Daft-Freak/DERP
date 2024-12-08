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

    uint32_t readCount = 0; // samples to read after trigger
    uint32_t delayCount = 0; // samples to wait before sending data
    uint32_t divider = 0;
};