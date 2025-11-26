
#include "db_loader.h"
#include <sqlite3.h>
#include <stdexcept>

DBLoader::DBLoader(const std::string &path)
    : db_path_(path) {}

std::vector<DeviceConfig> DBLoader::load_devices() {
    std::vector<DeviceConfig> out;
    sqlite3 *db = nullptr;
    if (sqlite3_open(db_path_.c_str(), &db) != SQLITE_OK) {
        throw std::runtime_error("Failed to open SQLite DB: " + db_path_);
    }

    const char *sql =
        "SELECT id, device_name, protocol, host, port, unit_id, polling_ms "
        "FROM devices";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        throw std::runtime_error("Failed to prepare devices query");
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        DeviceConfig d{};
        d.id         = sqlite3_column_int(stmt, 0);
        d.name       = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        d.protocol   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        d.host       = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        d.port       = sqlite3_column_int(stmt, 4);
        d.unit_id    = sqlite3_column_int(stmt, 5);
        d.polling_ms = sqlite3_column_int(stmt, 6);
        out.push_back(std::move(d));
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return out;
}

std::vector<RegisterConfig> DBLoader::load_registers_for_device(int device_id) {
    std::vector<RegisterConfig> out;
    sqlite3 *db = nullptr;
    if (sqlite3_open(db_path_.c_str(), &db) != SQLITE_OK) {
        throw std::runtime_error("Failed to open SQLite DB: " + db_path_);
    }

    const char *sql =
        "SELECT id, reg_name, address, quantity, function, mode, priority, polling_ms "
        "FROM registers WHERE device_id = ?";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        throw std::runtime_error("Failed to prepare registers query");
    }
    sqlite3_bind_int(stmt, 1, device_id);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        RegisterConfig r{};
        r.id            = sqlite3_column_int(stmt, 0);
        r.device_id     = device_id;
        r.reg_name      = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        r.address       = sqlite3_column_int(stmt, 2);
        r.quantity      = sqlite3_column_int(stmt, 3);
        r.function_code = sqlite3_column_int(stmt, 4);
        r.mode          = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
        r.priority      = sqlite3_column_int(stmt, 6);
        r.polling_ms    = sqlite3_column_int(stmt, 7);
        out.push_back(std::move(r));
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return out;
}
