
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <functional>

// Structures representing config coming from WebApp (binary)

struct ControlRegister {
    uint32_t reg_id;
    uint8_t  function;
    uint8_t  mode;        // 0=read,1=write
    uint16_t address;
    uint16_t quantity;
    uint16_t polling_ms;  // 0 = use device polling
    uint8_t  priority;
};

struct ControlDevice {
    uint32_t device_id;
    std::string host;     // dotted IPv4 for DB + logs
    uint16_t port;
    uint8_t  protocol;    // 0=tcp,1=udp
    uint8_t  unit_id;
    uint16_t polling_ms;  // base polling for this device
    std::vector<ControlRegister> regs;
};

// Binary control protocol over TCP from WebApp backend:
//
// CtrlHeader (little-endian)
//   magic   : 0x31534252 ('RBS1')
//   version : 1
//   msg_type: 0 = FULL_SNAPSHOT_REPLACE
//   reserved: 0
//   length  : payload bytes following header
//
// Payload for msg_type=0:
//   uint16_t device_count
//   For each device:
//     uint32_t device_id
//     uint8_t  protocol   (0=tcp,1=udp)
//     uint8_t  unit_id
//     uint16_t reserved
//     uint32_t ip_be      (IPv4, big-endian, same as inet_pton)
//     uint16_t port
//     uint16_t polling_ms
//     uint16_t reg_count
//     For each register:
//       uint32_t reg_id
//       uint8_t  function
//       uint8_t  mode
//       uint16_t address
//       uint16_t quantity
//       uint16_t polling_ms
//       uint8_t  priority
//       uint8_t  reserved2
//
// WebApp จะเป็นคน map JSON UI -> binary frame format นี้เอง

class ControlServer {
public:
    using SnapshotCallback = std::function<void(const std::vector<ControlDevice>&)>;

    ControlServer(int port, SnapshotCallback cb);
    ~ControlServer();

    // Starts blocking accept() loop. Typically run in its own std::thread.
    void run();

private:
    int port_;
    int listen_fd_{-1};
    SnapshotCallback callback_;

    bool setup_listen_socket();
    void handle_client(int client_fd);
    bool read_exact(int fd, void *buf, std::size_t len);
};
