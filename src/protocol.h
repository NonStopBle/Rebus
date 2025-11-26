
#pragma once
#include <cstdint>

#pragma pack(push, 1)
struct RebusHeader {
    uint8_t  version;     // protocol version
    uint8_t  msg_type;    // 0=READ_REQ,1=WRITE_REQ,2=READ_RESP,3=WRITE_RESP,4=NOTIFY,5=ERROR
    uint16_t flags;       // reserved / type info
    uint32_t length;      // total length including header
    uint32_t corr_id;     // correlation id
};

struct DeviceID {
    uint32_t device_index;  // mapped to (host,port,unit) via config
};
#pragma pack(pop)

enum class RebusMsgType : uint8_t {
    READ_REQ   = 0,
    WRITE_REQ  = 1,
    READ_RESP  = 2,
    WRITE_RESP = 3,
    NOTIFY     = 4,
    ERROR      = 5
};

inline uint16_t rebus_htons(uint16_t v) {
    return static_cast<uint16_t>((v >> 8) | (v << 8));
}
inline uint32_t rebus_htonl(uint32_t v) {
    return  ((v & 0x000000FFu) << 24) |
            ((v & 0x0000FF00u) << 8)  |
            ((v & 0x00FF0000u) >> 8)  |
            ((v & 0xFF000000u) >> 24);
}

inline void rebus_hton_header(RebusHeader &h) {
    h.flags   = rebus_htons(h.flags);
    h.length  = rebus_htonl(h.length);
    h.corr_id = rebus_htonl(h.corr_id);
}

inline void rebus_ntoh_header(RebusHeader &h) {
    rebus_hton_header(h);
}
