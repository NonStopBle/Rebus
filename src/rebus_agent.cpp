// ============================================
// rebus_agent.cpp - Minimal debug version
// Purpose: Diagnose why polling not working
// ============================================
#include "rebus_agent.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <set>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

// ============================================
// Load configuration from SQLite database
// ============================================
void RebusAgent::load_config_from_sqlite()
{
    auto devs = db_.load_devices();
    std::cout << "[CONFIG] Loading from database: " << devs.size() << " devices found\n";
    
    if (devs.empty()) {
        std::cout << "[CONFIG] WARNING: No devices in database\n";
        return;
    }
    
    std::vector<DeviceRuntime> tmp;
    tmp.reserve(devs.size());
    
    for (auto &d : devs)
    {
        std::cout << "[CONFIG] Device " << d.id 
                  << ": host=" << d.host 
                  << " port=" << d.port 
                  << " protocol=" << d.protocol << "\n";
        
        DeviceRuntime rt;
        rt.config = d;
        auto regs = db_.load_registers_for_device(d.id);
        
        std::cout << "[CONFIG]   Registers found: " << regs.size() << "\n";
        
        if (regs.empty()) {
            std::cout << "[CONFIG]   WARNING: No registers for device " << d.id << "\n";
        }
        
        for (auto &r : regs)
        {
            std::cout << "[CONFIG]   Reg " << r.id 
                      << ": addr=" << r.address
                      << " qty=" << r.quantity
                      << " mode=" << r.mode
                      << " poll_ms=" << r.polling_ms << "\n";
            
            RegisterRuntime rr;
            rr.cfg = r;
            
            // Calculate effective polling interval
            int base = d.polling_ms;
            if (rr.cfg.polling_ms > 0)
                base = rr.cfg.polling_ms;
            if (base < 10)
                base = 10;
            rr.effective_poll_ms = base;
            
            // CRITICAL: Initialize next_poll_due to NOW
            rr.next_poll_due = std::chrono::steady_clock::now();
            
            rt.regs.push_back(std::move(rr));
        }
        tmp.push_back(std::move(rt));
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        devices_.swap(tmp);
    }

    std::cout << "[CONFIG] Configuration loaded: " << devices_.size() << " devices total\n";
}

// ============================================
// Rebuild runtime from control message
// ============================================
void RebusAgent::rebuild_runtime_from_control(const std::vector<ControlDevice> &ctrl_devs)
{
    std::cout << "[CONTROL] Rebuilding runtime from control message: " 
              << ctrl_devs.size() << " devices\n";
    
    std::vector<DeviceRuntime> tmp;
    tmp.reserve(ctrl_devs.size());

    for (auto &cd : ctrl_devs)
    {
        DeviceRuntime rt;
        rt.config.id = cd.device_id;
        rt.config.protocol = (cd.protocol == 0) ? "tcp" : 
                            (cd.protocol == 1) ? "udp" : "tcp_rtu";
        rt.config.unit_id = cd.unit_id;
        rt.config.host = cd.host;
        rt.config.port = cd.port;
        rt.config.polling_ms = cd.polling_ms;

        std::cout << "[CONTROL] Device " << cd.device_id 
                  << ": " << cd.host << ":" << cd.port 
                  << " protocol=" << rt.config.protocol << "\n";

        for (auto &cr : cd.regs)
        {
            RegisterRuntime rr;
            rr.cfg.id = cr.reg_id;
            rr.cfg.device_id = cd.device_id;
            rr.cfg.function_code = cr.function;
            rr.cfg.mode = (cr.mode == 0) ? "read" : "write";
            rr.cfg.address = cr.address;
            rr.cfg.quantity = cr.quantity;
            rr.cfg.polling_ms = cr.polling_ms;
            rr.cfg.priority = cr.priority;

            int base = cd.polling_ms;
            if (rr.cfg.polling_ms > 0)
                base = rr.cfg.polling_ms;
            if (base < 10)
                base = 10;
            rr.effective_poll_ms = base;
            
            // CRITICAL: Initialize next_poll_due to NOW
            rr.next_poll_due = std::chrono::steady_clock::now();

            rt.regs.push_back(std::move(rr));
        }
        tmp.push_back(std::move(rt));
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        devices_.swap(tmp);
    }

    std::cout << "[CONTROL] Runtime rebuilt: " << devices_.size() << " devices\n";
}

// ============================================
// Main polling loop
// This runs continuously checking for due polls
// ============================================
void RebusAgent::main_loop()
{
    std::cout << "[LOOP] Starting main polling loop\n";
    
    // Initial check
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        std::cout << "[LOOP] Initial device count: " << devices_.size() << "\n";
        
        for (const auto &dev : devices_) {
            std::cout << "[LOOP]   Device " << dev.config.id 
                      << " has " << dev.regs.size() << " registers\n";
        }
    }
    
    int loop_iteration = 0;
    int last_poll_count = 0;
    
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        
        // Debug output every 5000 iterations (roughly 5 seconds)
        if (loop_iteration % 5000 == 0) {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            
            int current_polls = metrics_.polls.load();
            int polls_delta = current_polls - last_poll_count;
            last_poll_count = current_polls;
            
            std::cout << "[LOOP] Iteration " << loop_iteration 
                      << " | Devices: " << devices_.size()
                      << " | Total polls: " << current_polls
                      << " | Polls last 5s: " << polls_delta
                      << " | Errors: " << metrics_.errors.load() << "\n";
            
            // Check if any devices/registers loaded
            if (devices_.empty()) {
                std::cout << "[LOOP] WARNING: No devices loaded. Check database or control input.\n";
            } else {
                for (const auto &dev : devices_) {
                    if (dev.regs.empty()) {
                        std::cout << "[LOOP] WARNING: Device " << dev.config.id 
                                  << " has no registers\n";
                    }
                }
            }
        }
        
        // Check all devices and registers for due polls
        {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            for (auto &dev : devices_)
            {
                for (auto &reg : dev.regs)
                {
                    poll_if_due(dev, reg, now);
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        loop_iteration++;
    }
}

// ============================================
// Check if register is due for polling
// If due, schedule it to thread pool
// ============================================
void RebusAgent::poll_if_due(DeviceRuntime &dev,
                             RegisterRuntime &reg,
                             const std::chrono::steady_clock::time_point &now)
{
    // Check if polling is due
    if (now < reg.next_poll_due)
    {
        return;
    }

    // Debug first poll for each register (only once)
    static std::set<int> first_poll_logged;
    if (first_poll_logged.find(reg.cfg.id) == first_poll_logged.end()) {
        std::cout << "[POLL] First poll scheduled for device " << dev.config.id 
                  << " register " << reg.cfg.id << "\n";
        first_poll_logged.insert(reg.cfg.id);
    }

    // Schedule next poll
    reg.next_poll_due = now + std::chrono::milliseconds(reg.effective_poll_ms);

    // Enqueue to thread pool
    pool_.enqueue([this, &dev, &reg]()
    {
        handle_poll(dev, reg);
    });
}

// ============================================
// CRC16 calculation for Modbus RTU
// ============================================
uint16_t modbus_crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// ============================================
// Build Modbus RTU frame
// Format: [Unit][Func][Addr_H][Addr_L][Qty_H][Qty_L][CRC_L][CRC_H]
// ============================================
std::vector<uint8_t> build_modbus_rtu_adu(const ModbusRequest& req)
{
    std::vector<uint8_t> adu;
    adu.reserve(8);
    
    adu.push_back(req.unit_id);
    adu.push_back(req.function);
    adu.push_back((req.address >> 8) & 0xFF);
    adu.push_back(req.address & 0xFF);
    adu.push_back((req.quantity >> 8) & 0xFF);
    adu.push_back(req.quantity & 0xFF);
    
    uint16_t crc = modbus_crc16(adu.data(), adu.size());
    adu.push_back(crc & 0xFF);
    adu.push_back((crc >> 8) & 0xFF);
    
    return adu;
}

// ============================================
// Handle actual poll operation
// This runs in thread pool worker
// ============================================
void RebusAgent::handle_poll(DeviceRuntime &dev, RegisterRuntime &reg)
{
    // Check mode
    if (reg.cfg.mode != "read")
    {
        return;
    }

    // Build Modbus request
    ModbusRequest req{};
    req.unit_id = static_cast<uint8_t>(dev.config.unit_id);
    req.function = static_cast<uint8_t>(reg.cfg.function_code);
    req.address = static_cast<uint16_t>(reg.cfg.address);
    req.quantity = static_cast<uint16_t>(reg.cfg.quantity);

    // Choose protocol: tcp, udp, or tcp_rtu
    std::vector<uint8_t> adu;
    bool is_rtu = (dev.config.protocol == "tcp_rtu" || dev.config.protocol == "rtu");
    
    if (is_rtu)
    {
        adu = build_modbus_rtu_adu(req);
    }
    else
    {
        static std::atomic<uint16_t> transaction_id{1};
        uint16_t tid = transaction_id.fetch_add(1, std::memory_order_relaxed);
        adu = build_modbus_tcp_adu(req, tid);
    }

    auto t_start = std::chrono::steady_clock::now();

    // Create socket
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
        std::cerr << "[POLL] Socket creation failed for device " << dev.config.id << "\n";
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Set timeouts
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // Resolve address
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(dev.config.port));
    
    if (inet_pton(AF_INET, dev.config.host.c_str(), &addr.sin_addr) <= 0)
    {
        std::cerr << "[POLL] Invalid IP for device " << dev.config.id 
                  << ": " << dev.config.host << "\n";
        close(sock);
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    ssize_t sent = 0;
    ssize_t n = 0;
    uint8_t buf[260];

    // Send and receive based on protocol
    if (dev.config.protocol == "udp")
    {
        sent = sendto(sock, adu.data(), adu.size(), 0,
                      (sockaddr *)&addr, sizeof(addr));
        if (sent != (ssize_t)adu.size())
        {
            std::cerr << "[POLL] UDP send failed for device " << dev.config.id << "\n";
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_sent.fetch_add((uint64_t)sent, std::memory_order_relaxed);

        socklen_t alen = sizeof(addr);
        n = recvfrom(sock, buf, sizeof(buf), 0, (sockaddr *)&addr, &alen);
        if (n <= 0)
        {
            std::cerr << "[POLL] UDP recv failed for device " << dev.config.id << "\n";
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_recv.fetch_add((uint64_t)n, std::memory_order_relaxed);
    }
    else
    {
        // TCP connection
        if (connect(sock, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            std::cerr << "[POLL] TCP connect failed for device " << dev.config.id 
                      << " at " << dev.config.host << ":" << dev.config.port 
                      << " errno=" << errno << "\n";
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        sent = send(sock, adu.data(), adu.size(), 0);
        if (sent != (ssize_t)adu.size())
        {
            std::cerr << "[POLL] TCP send failed for device " << dev.config.id << "\n";
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_sent.fetch_add((uint64_t)sent, std::memory_order_relaxed);

        n = recv(sock, buf, sizeof(buf), 0);
        if (n <= 0)
        {
            std::cerr << "[POLL] TCP recv failed for device " << dev.config.id << "\n";
            close(sock);
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        metrics_.bytes_recv.fetch_add((uint64_t)n, std::memory_order_relaxed);
    }

    close(sock);

    // Parse response based on protocol
    const uint8_t *data = nullptr;
    std::size_t data_len = 0;

    if (is_rtu)
    {
        // Modbus RTU response: [Unit][Func][ByteCount][Data...][CRC_L][CRC_H]
        if (n < 5)
        {
            std::cerr << "[POLL] RTU response too short from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        uint8_t func = buf[1];
        uint8_t byte_count = buf[2];
        
        if (func & 0x80)
        {
            std::cerr << "[POLL] RTU exception from device " << dev.config.id 
                      << " code=" << (int)buf[2] << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        if (n < 3 + byte_count + 2)
        {
            std::cerr << "[POLL] Incomplete RTU response from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        // Verify CRC
        uint16_t received_crc = buf[n - 2] | (buf[n - 1] << 8);
        uint16_t calculated_crc = modbus_crc16(buf, n - 2);
        
        if (received_crc != calculated_crc)
        {
            std::cerr << "[POLL] CRC error from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        data = buf + 3;
        data_len = byte_count;
    }
    else
    {
        // Modbus TCP response
        if (n < 9)
        {
            std::cerr << "[POLL] TCP response too short from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        uint8_t func = buf[7];
        uint8_t byte_count = buf[8];
        
        if (func & 0x80)
        {
            std::cerr << "[POLL] TCP exception from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        if (9 + byte_count > n)
        {
            std::cerr << "[POLL] Incomplete TCP response from device " << dev.config.id << "\n";
            metrics_.errors.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        data = buf + 9;
        data_len = byte_count;
    }

    // Store to Redis
    std::vector<uint8_t> field(8);
    uint32_t did = (uint32_t)dev.config.id;
    uint32_t rid = (uint32_t)reg.cfg.id;
    std::memcpy(field.data(), &did, 4);
    std::memcpy(field.data() + 4, &rid, 4);

    std::vector<uint8_t> value(data, data + data_len);
    
    try {
        redis_.hset_binary("rebus:values", field, value);
    } catch (const std::exception &e) {
        std::cerr << "[POLL] Redis error for device " << dev.config.id 
                  << ": " << e.what() << "\n";
        metrics_.errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Build and broadcast frame
    try
    {
        std::vector<RebusBlock> blocks;

        RebusBlock dev_block;
        dev_block.type = 0x00;
        dev_block.data.resize(4);
        std::memcpy(dev_block.data.data(), &did, 4);
        blocks.push_back(dev_block);

        RebusBlock reg_block;
        reg_block.type = 0x01;
        reg_block.data.reserve(4 + 1 + 1 + data_len);
        reg_block.data.resize(4);
        std::memcpy(reg_block.data.data(), &rid, 4);

        RegDataType dtype = RegDataType::RAW;
        if (reg.cfg.quantity == 1)
            dtype = RegDataType::INT16;
        else if (reg.cfg.quantity == 2)
            dtype = RegDataType::FLOAT32;

        reg_block.data.push_back(static_cast<uint8_t>(dtype));
        reg_block.data.push_back(static_cast<uint8_t>(data_len));
        reg_block.data.insert(reg_block.data.end(), data, data + data_len);
        blocks.push_back(reg_block);

        auto frame = RebusFrameBuilder::build_frame(blocks, 0);
        out_tcp_.broadcast(frame);
    }
    catch (...) {}

    // Update metrics
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