
#pragma once
#include <cstdint>
#include <vector>

// Generic block for unified Rebus data frame
struct RebusBlock {
    uint8_t type;
    std::vector<uint8_t> data;
};

// Unified Rebus Data Frame (RDFv3)
//   [0]   0x7E          Start
//   [1-2] payload_len   LE (bytes after this until before Stop)
//   [3]   frame_type    0 = values, 1 = metrics (reserved)
//   [4..] blocks: [type][len][data...]
//   [N]   0x7F          Stop
class RebusFrameBuilder {
public:
    static std::vector<uint8_t> build_frame(
        const std::vector<RebusBlock>& blocks,
        uint8_t frame_type);
};
