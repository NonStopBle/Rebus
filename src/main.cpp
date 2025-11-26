
#include "rebus_agent.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// Minimal JSON parser for a flat config.json:
//
// {
//   "sqlite_path": "rebus_agent.db",
//   "redis_uri": "redis://127.0.0.1:6379",
//   "agent_id": "agent_1",
//   "threads": 16,
//   "control_port": 7510
// }

static void load_config_json(const std::string &path,
                             std::string &sqlite_path,
                             std::string &redis_uri,
                             std::string &agent_id,
                             std::size_t &threads,
                             int &control_port) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Config] Can't open " << path << ", using defaults.\n";
        return;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::string text = buffer.str();

    auto get_string = [&](const std::string &key, std::string &out) {
        auto pos = text.find("\"" + key + "\"");
        if (pos == std::string::npos) return;
        pos = text.find(':', pos);
        if (pos == std::string::npos) return;
        pos++;
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) pos++;
        if (pos >= text.size() || text[pos] != '"') return;
        pos++;
        std::string val;
        while (pos < text.size() && text[pos] != '"') {
            val.push_back(text[pos]);
            pos++;
        }
        out = val;
    };

    auto get_int = [&](const std::string &key, long long &out) {
        auto pos = text.find("\"" + key + "\"");
        if (pos == std::string::npos) return;
        pos = text.find(':', pos);
        if (pos == std::string::npos) return;
        pos++;
        std::string num;
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) pos++;
        while (pos < text.size() && ((text[pos] >= '0' && text[pos] <= '9'))) {
            num.push_back(text[pos]);
            pos++;
        }
        if (!num.empty()) {
            out = std::stoll(num);
        }
    };

    long long tthreads = (long long)threads;
    long long tport    = (long long)control_port;

    get_string("sqlite_path", sqlite_path);
    get_string("redis_uri",   redis_uri);
    get_string("agent_id",    agent_id);
    get_int("threads",        tthreads);
    get_int("control_port",   tport);

    threads = (std::size_t)tthreads;
    control_port = (int)tport;
}

int main(int argc, char **argv) {
    std::string sqlite_path = "rebus_agent.db";
    std::string redis_uri   = "redis://127.0.0.1:6379";
    std::string agent_id    = "agent_1";
    std::size_t threads     = 8;
    int control_port        = 7510;

    if (argc > 1) {
        std::string cfg = argv[1];
        load_config_json(cfg, sqlite_path, redis_uri, agent_id, threads, control_port);
    }

    std::cout << "[Rebus-Agent] sqlite=" << sqlite_path
              << " redis=" << redis_uri
              << " agent_id=" << agent_id
              << " threads=" << threads
              << " control_port=" << control_port
              << "\n";

    try {
        RebusAgent agent(sqlite_path, redis_uri, agent_id, control_port, threads);
        agent.run();
    } catch (const std::exception &ex) {
        std::cerr << "Fatal error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
