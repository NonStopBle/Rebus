
#include "redis_connector.h"
#include <iostream>

RedisConnector::RedisConnector(const std::string &uri)
    : uri_(uri) {}

RedisConnector::~RedisConnector() {
    if (ctx_) {
        redisFree(ctx_);
    }
}

bool RedisConnector::parse_uri(const std::string &uri, std::string &host, int &port) {
    std::string tmp = uri;
    const std::string prefix = "redis://";
    if (tmp.rfind(prefix, 0) == 0) {
        tmp = tmp.substr(prefix.size());
    }
    auto pos = tmp.find(':');
    if (pos == std::string::npos) {
        host = tmp;
        port = 6379;
    } else {
        host = tmp.substr(0, pos);
        port = std::stoi(tmp.substr(pos + 1));
    }
    return true;
}

bool RedisConnector::connect() {
    std::string host;
    int port;
    parse_uri(uri_, host, port);
    ctx_ = redisConnect(host.c_str(), port);
    if (!ctx_ || ctx_->err) {
        if (ctx_) {
            std::cerr << "[Redis] Connection error: " << ctx_->errstr << std::endl;
            redisFree(ctx_);
            ctx_ = nullptr;
        } else {
            std::cerr << "[Redis] Can't allocate redis context\n";
        }
        return false;
    }
    std::cout << "[Redis] Connected to " << host << ":" << port << std::endl;
    return true;
}

void RedisConnector::hset_binary(const std::string &key,
                                 const std::vector<uint8_t> &field,
                                 const std::vector<uint8_t> &value) {
    if (!ctx_) return;
    redisReply *reply = (redisReply*)redisCommand(ctx_, "HSET %s %b %b",
                                                  key.c_str(),
                                                  field.data(), (int)field.size(),
                                                  value.data(), (int)value.size());
    if (!reply) {
        std::cerr << "[Redis] HSET failed\n";
        return;
    }
    freeReplyObject(reply);
}

void RedisConnector::xadd_binary(const std::string &stream,
                                 const std::string &field_name,
                                 const std::vector<uint8_t> &data) {
    if (!ctx_) return;
    redisReply *reply = (redisReply*)redisCommand(ctx_, "XADD %s * %s %b",
                                                  stream.c_str(),
                                                  field_name.c_str(),
                                                  data.data(), (int)data.size());
    if (!reply) {
        std::cerr << "[Redis] XADD failed\n";
        return;
    }
    freeReplyObject(reply);
}
