
#pragma once
#include <string>
#include <vector>

struct DeviceConfig {
    int         id;
    std::string name;       // used only for debug/logging
    std::string protocol;   // "tcp" or "udp"
    std::string host;
    int         port;
    int         unit_id;    // Modbus unit id
    int         polling_ms; // base polling interval in ms
};

struct RegisterConfig {
    int         id;
    int         device_id;
    std::string reg_name;   // used only for debug/logging
    int         address;
    int         quantity;
    int         function_code;
    std::string mode;       // "read" / "write"
    int         priority;
    int         polling_ms; // override per register (0 = use device polling_ms)
};

class DBLoader {
public:
    explicit DBLoader(const std::string &path);
    std::vector<DeviceConfig> load_devices();
    std::vector<RegisterConfig> load_registers_for_device(int device_id);

private:
    std::string db_path_;
};
