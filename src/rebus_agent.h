
#pragma once
#include "db_loader.h"
#include "thread_pool.h"
#include "epoll_loop.h"
#include "modbus.h"
#include "redis_connector.h"
#include "metrics.h"
#include "control_server.h"
#include "tcp_broadcaster.h"
#include "rebus_types.h"
#include "rebus_frame.h"

#include <string>
#include <vector>
#include <chrono>
#include <mutex>

struct RegisterRuntime {
    RegisterConfig cfg;
    int  effective_poll_ms{100};
    std::chrono::steady_clock::time_point next_poll_due{
        std::chrono::steady_clock::time_point::min()
    };
    bool enabled{true};
};

struct DeviceRuntime {
    DeviceConfig                config;
    std::vector<RegisterRuntime> regs;
};

class RebusAgent {
public:
    RebusAgent(const std::string &sqlite_path,
               const std::string &redis_uri,
               const std::string &agent_id,
               int control_port,
               std::size_t threads);

    void run(); // blocking

    // Called by control server thread with full snapshot of config.
    void apply_full_snapshot(const std::vector<ControlDevice> &devices);

private:
    void load_config_from_sqlite();
    void rebuild_runtime_from_control(const std::vector<ControlDevice> &devices);
    void save_full_config_to_sqlite(const std::vector<ControlDevice> &devices);

    void main_loop();
    void poll_if_due(DeviceRuntime &dev,
                     RegisterRuntime &reg,
                     const std::chrono::steady_clock::time_point &now);
    void handle_poll(DeviceRuntime &dev, RegisterRuntime &reg);

    void metrics_loop();

    std::string sqlite_path_;
    DBLoader    db_;
    RedisConnector redis_;
    ThreadPool  pool_;
    EpollLoop   loop_; // reserved

    std::string agent_id_;
    int control_port_;

    std::mutex devices_mutex_;
    std::vector<DeviceRuntime> devices_;
    AgentMetrics metrics_;

    // Unified TCP binary transport
    int data_port_;
    TcpBroadcaster out_tcp_;
    RebusTypeMap type_map_;
};