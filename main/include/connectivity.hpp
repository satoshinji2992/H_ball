#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

struct NetworkConfig {
    std::string ssid;
    std::string password;
    int stream_port = 8000;
    int web_port = 8080;
    int jpeg_quality = 75;
    std::string recordings_dir = "./recordings";
};

NetworkConfig load_network_config(const std::string &path);
std::string resolve_config_path(const std::string &config_path, const std::string &value);

class WifiManager {
public:
    explicit WifiManager(std::string config_path);
    ~WifiManager();

    WifiManager(const WifiManager &) = delete;
    WifiManager &operator=(const WifiManager &) = delete;

    void request_reconnect();
    bool connected() const;
    std::string ip() const;
    std::string display_status() const;
    std::string web_status_json() const;

private:
    void worker_main();
    void set_status(const std::string &state, const std::string &detail,
                    const std::string &ip, bool connected);

    std::string config_path_;
    std::atomic<bool> stopping_{false};
    std::atomic<bool> reconnect_requested_{true};
    std::atomic<bool> connected_{false};
    mutable std::mutex status_mutex_;
    std::string state_ = "STARTING";
    std::string detail_;
    std::string ip_;
    std::thread worker_;
};
