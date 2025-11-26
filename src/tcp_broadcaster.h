
#pragma once
#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

class TcpBroadcaster {
public:
    explicit TcpBroadcaster(uint16_t port);
    ~TcpBroadcaster();

    void start();
    void stop();

    void broadcast(const std::vector<uint8_t>& frame);

private:
    void accept_loop();

    uint16_t port_;
    int listen_fd_;
    std::thread accept_thread_;
    std::mutex clients_mutex_;
    std::vector<int> clients_;
    std::atomic<bool> running_{false};
};
