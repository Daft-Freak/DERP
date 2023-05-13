#pragma once

#include <cstdint>

class GDBServer final
{
public:
    GDBServer(uint16_t port = 3333);

    bool start();
    void stop();

    bool update();

private:
    bool sendEmptyReply(int fd);
    bool sendNegAck(int fd);

    bool sendAll(int fd, const void *data, size_t &len, int flags);
    int close(int fd);

    uint16_t port;
    int listenFd = -1, clientFd = -1;
};