
#include "rebus_frame.h"

std::vector<uint8_t> RebusFrameBuilder::build_frame(
    const std::vector<RebusBlock>& blocks,
    uint8_t frame_type)
{
    std::vector<uint8_t> payload;
    payload.reserve(256);
    payload.push_back(frame_type);

    for (const auto& b : blocks) {
        payload.push_back(b.type);
        uint8_t len = static_cast<uint8_t>(b.data.size());
        payload.push_back(len);
        payload.insert(payload.end(), b.data.begin(), b.data.end());
    }

    uint16_t payload_len = static_cast<uint16_t>(payload.size());

    std::vector<uint8_t> out;
    out.reserve(payload_len + 4);
    out.push_back(0x7E);
    out.push_back(payload_len & 0xFF);
    out.push_back((payload_len >> 8) & 0xFF);
    out.insert(out.end(), payload.begin(), payload.end());
    out.push_back(0x7F);

    return out;
}
