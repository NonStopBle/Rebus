
#pragma once
#include <string>
#include <vector>
#include <hiredis/hiredis.h>

// Redis connector using hiredis (synchronous).
// All payloads in hot path are binary, no JSON.

class RedisConnector {
public:
    explicit RedisConnector(const std::string &uri);
    ~RedisConnector();

    bool connect();

    // HSET with binary field and binary value
    void hset_binary(const std::string &key,
                     const std::vector<uint8_t> &field,
                     const std::vector<uint8_t> &value);

    // XADD with single binary field (e.g., metrics stream)
    void xadd_binary(const std::string &stream,
                     const std::string &field_name,
                     const std::vector<uint8_t> &data);

private:
    std::string uri_;
    redisContext *ctx_{nullptr};

    bool parse_uri(const std::string &uri, std::string &host, int &port);
};
