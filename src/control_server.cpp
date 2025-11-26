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
    std::cout << "[Control] ========================================" << std::endl;
    std::cout << "[Control] New client connected" << std::endl;
    
    CtrlHeader hdr{};
    if (!read_exact(client_fd, &hdr, sizeof(hdr))) {
        std::cerr << "[Control] ERROR: Failed to read header" << std::endl;
        return;
    }

    std::cout << "[Control] Header received:" << std::endl;
    std::cout << "[Control]   magic: 0x" << std::hex << hdr.magic << std::dec << std::endl;
    std::cout << "[Control]   version: " << (int)hdr.version << std::endl;
    std::cout << "[Control]   msg_type: " << (int)hdr.msg_type << std::endl;
    std::cout << "[Control]   length: " << hdr.length << " bytes" << std::endl;

    if (hdr.magic != 0x31534252u || hdr.version != 1 || hdr.msg_type != 0) {
        std::cerr << "[Control] ERROR: Invalid header" << std::endl;
        return;
    }

    uint32_t length = hdr.length;
    if (length == 0 || length > (16U * 1024U * 1024U)) {
        std::cerr << "[Control] ERROR: Invalid payload length: " << length << std::endl;
        return;
    }

    std::vector<uint8_t> payload(length);
    if (!read_exact(client_fd, payload.data(), payload.size())) {
        std::cerr << "[Control] ERROR: Failed to read payload" << std::endl;
        return;
    }

    std::cout << "[Control] Payload received: " << payload.size() << " bytes" << std::endl;
    std::cout << "[Control] First 32 bytes (hex): ";
    for (size_t i = 0; i < std::min(payload.size(), (size_t)32); i++) {
        printf("%02X ", payload[i]);
    }
    std::cout << std::endl;

    // Parse payload
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
        std::cerr << "[Control] ERROR: Failed to read device_count" << std::endl;
        return;
    }

    std::cout << "[Control] Device count: " << dev_count << std::endl;

    std::vector<ControlDevice> devices;
    devices.reserve(dev_count);

    for (uint16_t di = 0; di < dev_count; ++di) {
        std::cout << "[Control] --- Parsing device #" << (di + 1) << " ---" << std::endl;
        
        ControlDevice d{};
        if (!read_u32(d.device_id)) { 
            std::cerr << "[Control] ERROR: Failed to read device_id" << std::endl;
            return;
        }
        std::cout << "[Control]   device_id: " << d.device_id << std::endl;

        if (p + 1 > end) { 
            std::cerr << "[Control] ERROR: Failed to read protocol" << std::endl;
            return;
        }
        d.protocol = *p++;
        std::cout << "[Control]   protocol: " << (d.protocol == 0 ? "TCP" : "UDP") 
                  << " (" << (int)d.protocol << ")" << std::endl;

        if (p + 1 > end) { 
            std::cerr << "[Control] ERROR: Failed to read unit_id" << std::endl;
            return;
        }
        d.unit_id = *p++;
        std::cout << "[Control]   unit_id: " << (int)d.unit_id << std::endl;

        uint16_t reserved = 0;
        if (!read_u16(reserved)) { 
            std::cerr << "[Control] ERROR: Failed to read reserved" << std::endl;
            return;
        }

        uint32_t ip_be = 0;
        if (!read_u32(ip_be)) { 
            std::cerr << "[Control] ERROR: Failed to read ip_be" << std::endl;
            return;
        }

        std::cout << "[Control]   ip_be (raw): 0x" << std::hex << ip_be << std::dec << std::endl;

        uint16_t port = 0;
        if (!read_u16(port)) { 
            std::cerr << "[Control] ERROR: Failed to read port" << std::endl;
            return;
        }
        d.port = port;
        std::cout << "[Control]   port: " << d.port << std::endl;

        uint16_t polling_ms = 0;
        if (!read_u16(polling_ms)) { 
            std::cerr << "[Control] ERROR: Failed to read polling_ms" << std::endl;
            return;
        }
        d.polling_ms = polling_ms;
        std::cout << "[Control]   polling_ms: " << d.polling_ms << std::endl;

        uint16_t reg_count = 0;
        if (!read_u16(reg_count)) { 
            std::cerr << "[Control] ERROR: Failed to read reg_count" << std::endl;
            return;
        }
        std::cout << "[Control]   register_count: " << reg_count << std::endl;

        // ============================================
        // FIXED: Correct IP conversion
        // ============================================
        // The Python side sends: ip_be = struct.unpack(">I", socket.inet_aton(ip))[0]
        // This creates a big-endian (network order) uint32
        // Then packs it with struct.pack("<I", ip_be) = little-endian encoding
        // 
        // So we receive: the BYTES of a network-order IP, stored in little-endian format
        // 
        // Example: "172.16.6.2"
        // 1. inet_aton("172.16.6.2") = [AC 10 06 02] (big-endian/network order)
        // 2. unpack(">I") reads it as 0xAC100602
        // 3. pack("<I", 0xAC100602) writes bytes as [02 06 10 AC] (little-endian)
        // 4. We read it back as uint32: 0xAC100602 (because we use little-endian read_u32)
        //
        // So ip_be already contains the correct value in host byte order!
        // We just need to convert it to a string.

        // Extract individual octets
        uint8_t oct1 = (ip_be >> 24) & 0xFF;  // Most significant byte
        uint8_t oct2 = (ip_be >> 16) & 0xFF;
        uint8_t oct3 = (ip_be >> 8) & 0xFF;
        uint8_t oct4 = ip_be & 0xFF;          // Least significant byte

        char buf[INET_ADDRSTRLEN];
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u", oct1, oct2, oct3, oct4);
        d.host = buf;

        std::cout << "[Control]   host: " << d.host << " ✓" << std::endl;

        d.regs.reserve(reg_count);
        for (uint16_t ri = 0; ri < reg_count; ++ri) {
            ControlRegister r{};
            if (!read_u32(r.reg_id)) { 
                std::cerr << "[Control] ERROR: Failed to read reg_id" << std::endl;
                return;
            }

            if (p + 1 > end) { 
                std::cerr << "[Control] ERROR: Failed to read function" << std::endl;
                return;
            }
            r.function = *p++;

            if (p + 1 > end) { 
                std::cerr << "[Control] ERROR: Failed to read mode" << std::endl;
                return;
            }
            r.mode = *p++;

            if (!read_u16(r.address)) { 
                std::cerr << "[Control] ERROR: Failed to read address" << std::endl;
                return;
            }
            if (!read_u16(r.quantity)) { 
                std::cerr << "[Control] ERROR: Failed to read quantity" << std::endl;
                return;
            }
            if (!read_u16(r.polling_ms)) { 
                std::cerr << "[Control] ERROR: Failed to read reg polling_ms" << std::endl;
                return;
            }

            if (p + 1 > end) { 
                std::cerr << "[Control] ERROR: Failed to read priority" << std::endl;
                return;
            }
            r.priority = *p++;

            if (p + 1 > end) { 
                std::cerr << "[Control] ERROR: Failed to read reserved2" << std::endl;
                return;
            }
            uint8_t reserved2 = *p++;
            (void)reserved2;

            std::cout << "[Control]     Register #" << (ri + 1) << ":"
                      << " id=" << r.reg_id
                      << " func=" << (int)r.function
                      << " mode=" << (r.mode == 0 ? "read" : "write")
                      << " addr=" << r.address
                      << " qty=" << r.quantity
                      << " poll=" << r.polling_ms << "ms"
                      << " priority=" << (int)r.priority << std::endl;

            d.regs.push_back(std::move(r));
        }

        devices.push_back(std::move(d));
    }

    std::cout << "[Control] ✓ Successfully parsed " << devices.size() << " device(s)" << std::endl;
    std::cout << "[Control] ========================================" << std::endl;

    if (callback_) {
        callback_(devices);
    }
}

void ControlServer::run() {
    if (!setup_listen_socket()) {
        return;
    }

    std::cout << "[Control] Ready to accept connections..." << std::endl;

    while (true) {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if (cfd < 0) {
            std::perror("[Control] accept");
            continue;
        }
        
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &caddr.sin_addr, client_ip, sizeof(client_ip));
        std::cout << "[Control] Accepted connection from " << client_ip 
                  << ":" << ntohs(caddr.sin_port) << std::endl;
        
        handle_client(cfd);
        close(cfd);
    }
}