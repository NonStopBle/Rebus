
#pragma once
#include <cstdint>
#include <unordered_map>

// Data type for each register (client can auto-decode)
enum class RegDataType : uint8_t {
    INT16  = 1,
    UINT16 = 2,
    INT32  = 3,
    UINT32 = 4,
    FLOAT32= 5,
    RAW    = 6
};

class RebusTypeMap {
public:
    void set(uint32_t reg_id, RegDataType t) {
        table_[reg_id] = t;
    }
    RegDataType get(uint32_t reg_id) const {
        auto it = table_.find(reg_id);
        if (it == table_.end()) return RegDataType::RAW;
        return it->second;
    }
private:
    std::unordered_map<uint32_t, RegDataType> table_;
};
