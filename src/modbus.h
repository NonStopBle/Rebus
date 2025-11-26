
#pragma once
#include <cstdint>
#include <vector>

// Simple Modbus TCP request description
struct ModbusRequest {
    uint8_t  unit_id;
    uint8_t  function;
    uint16_t address;
    uint16_t quantity;
};

// Build Modbus TCP ADU: MBAP(7 bytes) + PDU
std::vector<uint8_t> build_modbus_tcp_adu(const ModbusRequest &req, uint16_t transaction_id);
