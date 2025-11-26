
#include "epoll_loop.h"
#include <stdexcept>

EpollLoop::EpollLoop() {
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        throw std::runtime_error("Failed to create epoll fd");
    }
}

EpollLoop::~EpollLoop() {
    if (epoll_fd_ >= 0) {
        close(epoll_fd_);
    }
}

bool EpollLoop::add_fd(int fd, uint32_t events) {
    epoll_event ev{};
    ev.events = events;
    ev.data.fd = fd;
    return epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == 0;
}

bool EpollLoop::modify_fd(int fd, uint32_t events) {
    epoll_event ev{};
    ev.events = events;
    ev.data.fd = fd;
    return epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &ev) == 0;
}

bool EpollLoop::remove_fd(int fd) {
    return epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) == 0;
}

void EpollLoop::run(const std::function<void(int, uint32_t)> &handler, int timeout_ms) {
    std::vector<epoll_event> events(MAX_EVENTS);
    while (true) {
        int n = epoll_wait(epoll_fd_, events.data(), MAX_EVENTS, timeout_ms);
        if (n < 0) {
            continue;
        }
        for (int i = 0; i < n; ++i) {
            handler(events[i].data.fd, events[i].events);
        }
    }
}
