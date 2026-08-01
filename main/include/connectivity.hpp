#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <string>

class WifiManager {
public:
    WifiManager();
    ~WifiManager();

    WifiManager(const WifiManager &) = delete;
    WifiManager &operator=(const WifiManager &) = delete;

    bool connected() const;
    std::string ip() const;
    std::string display_status() const;
    std::string web_status_json() const;

private:
    void worker_main();
    void set_status(const std::string &state, const std::string &detail,
                    const std::string &ip, bool connected);

    std::atomic<bool> stopping_{false};
    std::atomic<bool> connected_{false};
    mutable std::mutex status_mutex_;
    std::string state_ = "CHECKING";
    std::string detail_;
    std::string ip_;
    std::thread worker_;
};
