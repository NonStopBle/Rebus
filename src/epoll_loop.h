
#pragma once
#include <sys/epoll.h>
#include <unistd.h>
#include <functional>
#include <vector>

class EpollLoop {
public:
    EpollLoop();
    ~EpollLoop();

    bool add_fd(int fd, uint32_t events);
    bool modify_fd(int fd, uint32_t events);
    bool remove_fd(int fd);

    void run(const std::function<void(int, uint32_t)> &handler, int timeout_ms = 10);

private:
    int epoll_fd_;
    static constexpr int MAX_EVENTS = 4096;
};
