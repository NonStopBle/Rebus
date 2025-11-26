
#include "modbus.h"

std::vector<uint8_t> build_modbus_tcp_adu(const ModbusRequest &req, uint16_t transaction_id) {
    std::vector<uint8_t> adu;
    adu.reserve(12);

    // MBAP header
    adu.push_back(static_cast<uint8_t>(transaction_id >> 8));
    adu.push_back(static_cast<uint8_t>(transaction_id & 0xFF));
    adu.push_back(0x00); // protocol id hi
    adu.push_back(0x00); // protocol id lo
    uint16_t len = 1 /*unit*/ + 1 /*func*/ + 2 /*addr*/ + 2 /*qty*/;
    adu.push_back(static_cast<uint8_t>(len >> 8));
    adu.push_back(static_cast<uint8_t>(len & 0xFF));
    adu.push_back(req.unit_id);

    // PDU
    adu.push_back(req.function);
    adu.push_back(static_cast<uint8_t>(req.address >> 8));
    adu.push_back(static_cast<uint8_t>(req.address & 0xFF));
    adu.push_back(static_cast<uint8_t>(req.quantity >> 8));
    adu.push_back(static_cast<uint8_t>(req.quantity & 0xFF));

    return adu;
}
