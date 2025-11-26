
#include "metrics.h"
#include <unistd.h>
#include <fstream>
#include <string>
#include <cstring>

static uint64_t read_rss_kb() {
    std::ifstream f("/proc/self/statm");
    if (!f.is_open()) return 0;
    long size = 0, resident = 0;
    f >> size >> resident;
    long page_size = sysconf(_SC_PAGESIZE);
    return (uint64_t)resident * (uint64_t)page_size / 1024ULL;
}

static uint64_t read_cpu_ticks() {
    std::ifstream f("/proc/self/stat");
    if (!f.is_open()) return 0;
    std::string tmp;
    long utime = 0, stime = 0;
    for (int i = 1; i <= 13; ++i) {
        f >> tmp;
    }
    f >> utime >> stime;
    return (uint64_t)(utime + stime);
}

std::vector<uint8_t> build_metrics_blob(const AgentMetrics &m) {
    uint64_t rss_kb    = read_rss_kb();
    uint64_t cpu_ticks = read_cpu_ticks();

    uint64_t fields[8];
    fields[0] = m.polls.load(std::memory_order_relaxed);
    fields[1] = m.bytes_sent.load(std::memory_order_relaxed);
    fields[2] = m.bytes_recv.load(std::memory_order_relaxed);
    fields[3] = m.errors.load(std::memory_order_relaxed);
    fields[4] = m.last_latency_us.load(std::memory_order_relaxed);
    fields[5] = m.max_latency_us.load(std::memory_order_relaxed);
    fields[6] = rss_kb;
    fields[7] = cpu_ticks;

    std::vector<uint8_t> out(sizeof(fields));
    std::memcpy(out.data(), fields, sizeof(fields));
    return out;
}
