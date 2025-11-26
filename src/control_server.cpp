
#include "control_server.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#pragma pack(push, 1)
struct CtrlHeader {
    uint32_t magic;    // 'RBS1' = 0x31534252
    uint8_t  version;
    uint8_t  msg_type; // 0 = FULL_SNAPSHOT_REPLACE
    uint16_t reserved;
    uint32_t length;   // payload length
};
#pragma pack(pop)

ControlServer::ControlServer(int port, SnapshotCallback cb)
    : port_(port), callback_(std::move(cb)) {}

ControlServer::~ControlServer() {
    if (listen_fd_ >= 0) {
        close(listen_fd_);
    }
}

bool ControlServer::setup_listen_socket() {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::perror("[Control] socket");
        return false;
    }

    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port_));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::perror("[Control] bind");
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (listen(listen_fd_, 16) < 0) {
        std::perror("[Control] listen");
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    std::cout << "[Control] Listening on port " << port_ << std::endl;
    return true;
}

bool ControlServer::read_exact(int fd, void *buf, std::size_t len) {
    std::size_t off = 0;
    while (off < len) {
        ssize_t n = ::recv(fd, (char*)buf + off, len - off, 0);
        if (n <= 0) {
            return false;
        }
        off += (std::size_t)n;
    }
    return true;
}

void ControlServer::handle_client(int client_fd) {
    CtrlHeader hdr{};
    if (!read_exact(client_fd, &hdr, sizeof(hdr))) {
        std::cerr << "[Control] Failed to read header\n";
        return;
    }

    if (hdr.magic != 0x31534252u || hdr.version != 1 || hdr.msg_type != 0) {
        std::cerr << "[Control] Invalid header\n";
        return;
    }

    uint32_t length = hdr.length;
    if (length == 0 || length > (16U * 1024U * 1024U)) {
        std::cerr << "[Control] Invalid length\n";
        return;
    }

    std::vector<uint8_t> payload(length);
    if (!read_exact(client_fd, payload.data(), payload.size())) {
        std::cerr << "[Control] Failed to read payload\n";
        return;
    }

    // parse payload
    const uint8_t *p = payload.data();
    const uint8_t *end = payload.data() + payload.size();

    auto read_u16 = [&](uint16_t &out) -> bool {
        if (p + 2 > end) return false;
        out = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        p += 2;
        return true;
    };
    auto read_u32 = [&](uint32_t &out) -> bool {
        if (p + 4 > end) return false;
        out =  (uint32_t)p[0]
             | ((uint32_t)p[1] << 8)
             | ((uint32_t)p[2] << 16)
             | ((uint32_t)p[3] << 24);
        p += 4;
        return true;
    };

    uint16_t dev_count = 0;
    if (!read_u16(dev_count)) {
        std::cerr << "[Control] Failed to read device_count\n";
        return;
    }

    std::vector<ControlDevice> devices;
    devices.reserve(dev_count);

    for (uint16_t di = 0; di < dev_count; ++di) {
        ControlDevice d{};
        if (!read_u32(d.device_id)) { std::cerr << "[Control] dev_id fail\n"; return; }

        if (p + 1 > end) { std::cerr << "[Control] protocol fail\n"; return; }
        d.protocol = *p++;

        if (p + 1 > end) { std::cerr << "[Control] unit_id fail\n"; return; }
        d.unit_id = *p++;

        uint16_t reserved = 0;
        if (!read_u16(reserved)) { std::cerr << "[Control] reserved fail\n"; return; }

        uint32_t ip_be = 0;
        if (!read_u32(ip_be)) { std::cerr << "[Control] ip_be fail\n"; return; }

        uint16_t port = 0;
        if (!read_u16(port)) { std::cerr << "[Control] port fail\n"; return; }
        d.port = port;

        uint16_t polling_ms = 0;
        if (!read_u16(polling_ms)) { std::cerr << "[Control] polling_ms fail\n"; return; }
        d.polling_ms = polling_ms;

        uint16_t reg_count = 0;
        if (!read_u16(reg_count)) { std::cerr << "[Control] reg_count fail\n"; return; }

        // convert ip_be to dotted string
        uint32_t ip_host = ntohl(ip_be);
        struct in_addr a{};
        a.s_addr = htonl(ip_host);
        char buf[INET_ADDRSTRLEN] = {0};
        const char *ip_str = inet_ntop(AF_INET, &a, buf, sizeof(buf));
        if (!ip_str) {
            d.host = "0.0.0.0";
        } else {
            d.host = ip_str;
        }

        d.regs.reserve(reg_count);
        for (uint16_t ri = 0; ri < reg_count; ++ri) {
            ControlRegister r{};
            if (!read_u32(r.reg_id)) { std::cerr << "[Control] reg_id fail\n"; return; }

            if (p + 1 > end) { std::cerr << "[Control] func fail\n"; return; }
            r.function = *p++;

            if (p + 1 > end) { std::cerr << "[Control] mode fail\n"; return; }
            r.mode = *p++;

            if (!read_u16(r.address)) { std::cerr << "[Control] addr fail\n"; return; }
            if (!read_u16(r.quantity)) { std::cerr << "[Control] qty fail\n"; return; }
            if (!read_u16(r.polling_ms)) { std::cerr << "[Control] reg polling fail\n"; return; }

            if (p + 1 > end) { std::cerr << "[Control] priority fail\n"; return; }
            r.priority = *p++;

            if (p + 1 > end) { std::cerr << "[Control] reserved2 fail\n"; return; }
            uint8_t reserved2 = *p++;
            (void)reserved2;

            d.regs.push_back(std::move(r));
        }

        devices.push_back(std::move(d));
    }

    if (callback_) {
        callback_(devices);
    }
}

void ControlServer::run() {
    if (!setup_listen_socket()) {
        return;
    }

    while (true) {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if (cfd < 0) {
            std::perror("[Control] accept");
            continue;
        }
        handle_client(cfd);
        close(cfd);
    }
}
