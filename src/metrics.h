
#pragma once
#include <cstdint>
#include <atomic>
#include <vector>

// Simple metrics, will be serialized as binary 8 * uint64_t

struct AgentMetrics {
    std::atomic<uint64_t> polls{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> bytes_recv{0};
    std::atomic<uint64_t> errors{0};
    std::atomic<uint64_t> last_latency_us{0};
    std::atomic<uint64_t> max_latency_us{0};
};

// Returns binary blob:
// [0] polls
// [1] bytes_sent
// [2] bytes_recv
// [3] errors
// [4] last_latency_us
// [5] max_latency_us
// [6] rss_kb
// [7] cpu_ticks
std::vector<uint8_t> build_metrics_blob(const AgentMetrics &m);
