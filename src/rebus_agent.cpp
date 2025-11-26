
#include "rebus_agent.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sqlite3.h>
#include <cstring>

RebusAgent::RebusAgent(const std::string &sqlite_path,
                       const std::string &redis_uri,
                       const std::string &agent_id,
                       int control_port,
                       std::size_t threads)
    : sqlite_path_(sqlite_path),
      db_(sqlite_path),
      redis_(redis_uri),
      pool_(threads),
      loop_(),
      agent_id_(agent_id),
      control_port_(control_port),
      data_port_(8600),
      out_tcp_(static_cast<uint16_t>(data_port_)) {}

void RebusAgent::load_config_from_sqlite()
{
    auto devs = db_.load_devices();
    std::vector<DeviceRuntime> tmp;

    tmp.reserve(devs.size());
    for (auto &d : devs)
    {
        DeviceRuntime rt;
        rt.config = d;
        auto regs = db_.load_registers_for_device(d.id);
        for (auto &r : regs)
        {
            RegisterRuntime rr;
            rr.cfg = r;
            int base = d.polling_ms;
            if (rr.cfg.polling_ms > 0)
                base = rr.cfg.polling_ms;
            if (base < 10)
                base = 10; // 100Hz max
            rr.effective_poll_ms = base;
            // ไม่มีการ set rr.next_poll_due เลย
            rr.next_poll_due = std::chrono::steady_clock::now();
            rt.regs.push_back(std::move(rr));
        }
        tmp.push_back(std::move(rt));
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        devices_.swap(tmp);
    }

    std::cout << "[RebusAgent] Loaded " << devices_.size()
              << " devices from SQLite\n";
}

void RebusAgent::save_full_config_to_sqlite(const std::vector<ControlDevice> &devs)
{
    sqlite3 *db = nullptr;
    if (sqlite3_open(sqlite_path_.c_str(), &db) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] Failed to open SQLite for save\n";
        return;
    }

    char *errmsg = nullptr;
    if (sqlite3_exec(db, "BEGIN IMMEDIATE TRANSACTION;", nullptr, nullptr, &errmsg) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] BEGIN TX failed: " << (errmsg ? errmsg : "") << "\n";
        sqlite3_free(errmsg);
        sqlite3_close(db);
        return;
    }

    const char *sql_del_regs = "DELETE FROM registers;";
    const char *sql_del_devs = "DELETE FROM devices;";
    if (sqlite3_exec(db, sql_del_regs, nullptr, nullptr, &errmsg) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] DELETE registers failed: " << (errmsg ? errmsg : "") << "\n";
        sqlite3_free(errmsg);
    }
    if (sqlite3_exec(db, sql_del_devs, nullptr, nullptr, &errmsg) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] DELETE devices failed: " << (errmsg ? errmsg : "") << "\n";
        sqlite3_free(errmsg);
    }

    // Devices
    const char *sql_ins_dev =
        "INSERT OR REPLACE INTO devices "
        "(id, device_name, protocol, host, port, unit_id, polling_ms) "
        "VALUES (?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt_dev = nullptr;
    if (sqlite3_prepare_v2(db, sql_ins_dev, -1, &stmt_dev, nullptr) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] prepare dev insert failed\n";
        sqlite3_close(db);
        return;
    }

    const char *sql_ins_reg =
        "INSERT OR REPLACE INTO registers "
        "(id, device_id, reg_name, address, quantity, function, mode, priority, polling_ms) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt_reg = nullptr;
    if (sqlite3_prepare_v2(db, sql_ins_reg, -1, &stmt_reg, nullptr) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] prepare reg insert failed\n";
        sqlite3_finalize(stmt_dev);
        sqlite3_close(db);
        return;
    }

    for (const auto &d : devs)
    {
        sqlite3_reset(stmt_dev);
        sqlite3_clear_bindings(stmt_dev);

        sqlite3_bind_int(stmt_dev, 1, (int)d.device_id);

        std::string dev_name = "dev_" + std::to_string(d.device_id);
        sqlite3_bind_text(stmt_dev, 2, dev_name.c_str(), -1, SQLITE_TRANSIENT);

        const char *proto = (d.protocol == 1) ? "udp" : "tcp";
        sqlite3_bind_text(stmt_dev, 3, proto, -1, SQLITE_TRANSIENT);

        sqlite3_bind_text(stmt_dev, 4, d.host.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt_dev, 5, (int)d.port);
        sqlite3_bind_int(stmt_dev, 6, (int)d.unit_id);
        sqlite3_bind_int(stmt_dev, 7, (int)d.polling_ms);

        if (sqlite3_step(stmt_dev) != SQLITE_DONE)
        {
            std::cerr << "[RebusAgent] insert device failed\n";
        }

        for (const auto &r : d.regs)
        {
            sqlite3_reset(stmt_reg);
            sqlite3_clear_bindings(stmt_reg);

            sqlite3_bind_int(stmt_reg, 1, (int)r.reg_id);
            sqlite3_bind_int(stmt_reg, 2, (int)d.device_id);

            std::string reg_name = "reg_" + std::to_string(r.reg_id);
            sqlite3_bind_text(stmt_reg, 3, reg_name.c_str(), -1, SQLITE_TRANSIENT);

            sqlite3_bind_int(stmt_reg, 4, (int)r.address);
            sqlite3_bind_int(stmt_reg, 5, (int)r.quantity);
            sqlite3_bind_int(stmt_reg, 6, (int)r.function);

            const char *mode = (r.mode == 1) ? "write" : "read";
            sqlite3_bind_text(stmt_reg, 7, mode, -1, SQLITE_TRANSIENT);

            sqlite3_bind_int(stmt_reg, 8, (int)r.priority);
            sqlite3_bind_int(stmt_reg, 9, (int)r.polling_ms);

            if (sqlite3_step(stmt_reg) != SQLITE_DONE)
            {
                std::cerr << "[RebusAgent] insert register failed\n";
            }
        }
    }

    sqlite3_finalize(stmt_reg);
    sqlite3_finalize(stmt_dev);

    if (sqlite3_exec(db, "COMMIT;", nullptr, nullptr, &errmsg) != SQLITE_OK)
    {
        std::cerr << "[RebusAgent] COMMIT failed: " << (errmsg ? errmsg : "") << "\n";
        sqlite3_free(errmsg);
    }

    sqlite3_close(db);
    std::cout << "[RebusAgent] Saved full config to SQLite (" << devs.size() << " devices)\n";
}

void RebusAgent::rebuild_runtime_from_control(const std::vector<ControlDevice> &devs)
{
    std::vector<DeviceRuntime> tmp;
    tmp.reserve(devs.size());

    auto now = std::chrono::steady_clock::now();

    for (const auto &d : devs)
    {
        DeviceRuntime dr{};
        dr.config.id = (int)d.device_id;
        dr.config.name = "dev_" + std::to_string(d.device_id);
        dr.config.protocol = (d.protocol == 1) ? "udp" : "tcp";
        dr.config.host = d.host;
        dr.config.port = d.port;
        dr.config.unit_id = d.unit_id;
        dr.config.polling_ms = d.polling_ms;

        for (const auto &r : d.regs)
        {
            RegisterRuntime rr{};
            rr.cfg.id = (int)r.reg_id;
            rr.cfg.device_id = (int)d.device_id;
            rr.cfg.reg_name = "reg_" + std::to_string(r.reg_id);
            rr.cfg.address = (int)r.address;
            rr.cfg.quantity = (int)r.quantity;
            rr.cfg.function_code = (int)r.function;
            rr.cfg.mode = (r.mode == 1) ? "write" : "read";
            rr.cfg.priority = (int)r.priority;
            rr.cfg.polling_ms = (int)r.polling_ms;

            int base = dr.config.polling_ms;
            if (rr.cfg.polling_ms > 0)
                base = rr.cfg.polling_ms;
            if (base < 10)
                base = 10;
            rr.effective_poll_ms = base;
            rr.next_poll_due = now;

            dr.regs.push_back(std::move(rr));
        }
        tmp.push_back(std::move(dr));
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        devices_.swap(tmp);
    }

    std::cout << "[RebusAgent] Runtime config rebuilt from control snapshot\n";
}

void RebusAgent::apply_full_snapshot(const std::vector<ControlDevice> &devices)
{
    // 1) Save binary config to SQLite (persistent)
    save_full_config_to_sqlite(devices);
    // 2) Rebuild in-memory runtime config for scheduler
    rebuild_runtime_from_control(devices);
}

void RebusAgent::poll_if_due(DeviceRuntime &dev,
                             RegisterRuntime &reg,
                             const std::chrono::steady_clock::time_point &now)
{
    if (now < reg.next_poll_due)
    {
        // std::cout << "now has been lower that \n";
        return;
    }

    reg.next_poll_due = now + std::chrono::milliseconds(reg.effective_poll_ms);

    pool_.enqueue([this, &dev, &reg]()
                  {
        std::cout << "astronbee handle_poll dev=" << dev.config.id
                  << " reg=" << reg.cfg.id << "\n";
        handle_poll(dev, reg); });
}

void RebusAgent::handle_poll(DeviceRuntime &dev, RegisterRuntime &reg)
{
    if (reg.cfg.mode != "read")
    {
        std::cout << "Not reading method" << std::endl;
        return;
    }

    ModbusRequest req{};
    req.unit_id = static_cast<uint8_t>(dev.config.unit_id);
    req.function = static_cast<uint8_t>(reg.cfg.function_code);
    req.address = static_cast<uint16_t>(reg.cfg.address);
    req.quantity = static_cast<uint16_t>(reg.cfg.quantity);

    static std::atomic<uint16_t> transaction_id{1};
    uint16_t tid = transaction_id.fetch_add(1, std::memory_order_relaxed);

    auto adu = build_modbus_tcp_adu(req, tid);

    auto t_start = std::chrono::steady_clock::now();

    int sock = -1;
    if (dev.config.protocol == "udp")
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
    }
    else
    {
        sock = socket(AF_INET, SOCK_STREAM, 0);
    }
    if (sock < 0)
    {
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(dev.config.port));
    if (inet_pton(AF_INET, dev.config.host.c_str(), &addr.sin_addr) <= 0)
    {
        close(sock);
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    ssize_t sent = 0;
    ssize_t n = 0;
    uint8_t buf[260];

    if (dev.config.protocol == "udp")
    {
        sent = sendto(sock, adu.data(), adu.size(), 0,
                      (sockaddr *)&addr, sizeof(addr));
        if (sent != (ssize_t)adu.size())
        {
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_sent.fetch_add((uint64_t)sent, std::memory_order_relaxed);

        socklen_t alen = sizeof(addr);
        n = recvfrom(sock, buf, sizeof(buf), 0, (sockaddr *)&addr, &alen);
        if (n <= 0)
        {
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_recv.fetch_add((uint64_t)n, std::memory_order_relaxed);
    }
    else
    {
        if (connect(sock, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        sent = send(sock, adu.data(), adu.size(), 0);
        if (sent != (ssize_t)adu.size())
        {
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_sent.fetch_add((uint64_t)sent, std::memory_order_relaxed);

        n = recv(sock, buf, sizeof(buf), 0);
        if (n <= 0)
        {
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_recv.fetch_add((uint64_t)n, std::memory_order_relaxed);
    }

    if (dev.config.protocol == "tcp")
    {
        close(sock);
    }
    else
    {
        close(sock);
    }

    if (n < 9)
    {
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    uint8_t func = buf[7];
    uint8_t byte_count = buf[8];
    if (func & 0x80)
    {
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (9 + byte_count > n)
    {
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    const uint8_t *data = buf + 9;
    std::size_t data_len = byte_count;

    // field = 8 bytes: [device_id(4)][reg_id(4)] LE
    std::vector<uint8_t> field(8);
    uint32_t did = (uint32_t)dev.config.id;
    uint32_t rid = (uint32_t)reg.cfg.id;
    std::memcpy(field.data(), &did, 4);
    std::memcpy(field.data() + 4, &rid, 4);

    std::vector<uint8_t> value(data, data + data_len);
    redis_.hset_binary("rebus:values", field, value);
    // build unified binary frame for TCP subscribers
    try
    {
        std::vector<RebusBlock> blocks;

        // Device header block
        RebusBlock dev_block;
        dev_block.type = 0x00; // device header
        dev_block.data.resize(4);
        std::memcpy(dev_block.data.data(), &did, 4);
        blocks.push_back(dev_block);

        // Register value block
        RebusBlock reg_block;
        reg_block.type = 0x01; // register value
        reg_block.data.reserve(4 + 1 + 1 + data_len);
        reg_block.data.resize(4);
        std::memcpy(reg_block.data.data(), &rid, 4);

        RegDataType dtype = RegDataType::RAW;
        if (reg.cfg.quantity == 1)
        {
            dtype = RegDataType::INT16;
        }
        else if (reg.cfg.quantity == 2)
        {
            dtype = RegDataType::FLOAT32;
        }

        reg_block.data.push_back(static_cast<uint8_t>(dtype));
        reg_block.data.push_back(static_cast<uint8_t>(data_len));
        reg_block.data.insert(reg_block.data.end(), data, data + data_len);
        blocks.push_back(reg_block);

        auto frame = RebusFrameBuilder::build_frame(blocks, 0);
        out_tcp_.broadcast(frame);

        std::cout << "Polling \n";
    }
    catch (...)
    {
        // never throw from polling path
    }

    auto t_end = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
    metrics_.last_latency_us.store((uint64_t)us, std::memory_order_relaxed);
    uint64_t prev_max = metrics_.max_latency_us.load(std::memory_order_relaxed);
    while ((uint64_t)us > prev_max &&
           !metrics_.max_latency_us.compare_exchange_weak(prev_max, (uint64_t)us,
                                                          std::memory_order_relaxed))
    {
    }

    metrics_.polls.fetch_add(1, std::memory_order_relaxed);
}

void RebusAgent::metrics_loop()
{
    while (true)
    {
        auto blob = build_metrics_blob(metrics_);

        uint32_t aid = 0;
        for (char c : agent_id_)
        {
            aid = aid * 131u + (uint8_t)c;
        }
        std::vector<uint8_t> field(4);
        std::memcpy(field.data(), &aid, 4);

        redis_.hset_binary("rebus:metrics", field, blob);
        redis_.xadd_binary("rebus:metrics_stream", "m", blob);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RebusAgent::main_loop()
{
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            for (auto &dev : devices_)
            {
                for (auto &reg : dev.regs)
                {
                    poll_if_due(dev, reg, now);
                    // std::cout << "ahh \n";
                }
            }
            // std::cout << "inside this";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RebusAgent::run()
{
    if (!redis_.connect())
    {
        std::cerr << "[RebusAgent] Failed to connect Redis\n";
    }

    // load initial config from SQLite
    load_config_from_sqlite();
    // start TCP broadcaster for unified data frames
    out_tcp_.start();

    // start metrics thread
    std::thread mthr(&RebusAgent::metrics_loop, this);
    mthr.detach();

    // start control server thread
    ControlServer ctrl(control_port_,
                       [this](const std::vector<ControlDevice> &devs)
                       {
                           this->apply_full_snapshot(devs);
                       });
    std::thread cthr([&ctrl]()
                     { ctrl.run(); });
    cthr.detach();

    // start main polling loop (blocking)
    main_loop();
}