#pragma once

#include <cstdint>
#include <string_view>

class ARMv6MCore;
class GDBServer final
{
public:
    GDBServer(uint16_t port = 3333);

    bool start();
    void stop();

    bool update();

    void setCPUs(ARMv6MCore *cpus, size_t numCPUs);

private:
    void haltCPUs();
    bool handleContinue(int fd);
    bool handleReadRegisters(int fd);
    bool handleWriteRegisters(int fd, std::string_view command);
    bool handleReadMemory(int fd, std::string_view command);
    bool handleWriteMemory(int fd, std::string_view command);
    bool handleAddBreakpoint(int fd, std::string_view command);
    bool handleRemoveBreakpoint(int fd, std::string_view command);
    bool handleCommand(int fd, std::string_view command);

    bool sendReply(int fd, const char *reply, size_t len);
    bool sendEmptyReply(int fd);
    bool sendPosAck(int fd);
    bool sendNegAck(int fd);

    bool sendAll(int fd, const void *data, size_t &len, int flags);
    int close(int fd);

    uint16_t port;
    int listenFd = -1, clientFd = -1;

    ARMv6MCore *cpus = nullptr;
    size_t numCPUs = 0;

    bool cpuHalted = false;
};