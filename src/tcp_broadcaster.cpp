
#include "tcp_broadcaster.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

TcpBroadcaster::TcpBroadcaster(uint16_t port)
    : port_(port), listen_fd_(-1) {}

TcpBroadcaster::~TcpBroadcaster() {
    stop();
}

void TcpBroadcaster::start() {
    if (running_) return;
    running_ = true;

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::perror("TcpBroadcaster socket");
        running_ = false;
        return;
    }

    int opt = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::perror("TcpBroadcaster bind");
        ::close(listen_fd_);
        listen_fd_ = -1;
        running_ = false;
        return;
    }

    if (::listen(listen_fd_, 16) < 0) {
        std::perror("TcpBroadcaster listen");
        ::close(listen_fd_);
        listen_fd_ = -1;
        running_ = false;
        return;
    }

    accept_thread_ = std::thread(&TcpBroadcaster::accept_loop, this);
    std::cout << "[TcpBroadcaster] listening on port " << port_ << std::endl;
}

void TcpBroadcaster::stop() {
    if (!running_) return;
    running_ = false;

    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : clients_) {
        ::shutdown(fd, SHUT_RDWR);
        ::close(fd);
    }
    clients_.clear();
}

void TcpBroadcaster::accept_loop() {
    while (running_) {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = ::accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if (cfd < 0) {
            if (running_) {
                std::perror("TcpBroadcaster accept");
            }
            break;
        }
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.push_back(cfd);
        }
    }
}

void TcpBroadcaster::broadcast(const std::vector<uint8_t>& frame) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto it = clients_.begin(); it != clients_.end();) {
        int fd = *it;
        ssize_t n = ::send(fd, frame.data(), frame.size(), MSG_NOSIGNAL);
        if (n < 0) {
            std::perror("TcpBroadcaster send");
            ::close(fd);
            it = clients_.erase(it);
        } else {
            ++it;
        }
    }
}
