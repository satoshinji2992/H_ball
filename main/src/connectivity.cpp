#include "connectivity.hpp"

#include "maix_basic.hpp"
#include "maix_wifi.hpp"

#include <sstream>
#include <vector>

using namespace maix;

namespace {

std::string json_escape(const std::string &value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const char ch : value) {
        if (ch == '\\') out += "\\\\";
        else if (ch == '"') out += "\\\"";
        else out += ch;
    }
    return out;
}

void interruptible_sleep(const std::atomic<bool> &stopping, int milliseconds) {
    for (int elapsed = 0; elapsed < milliseconds && !stopping.load(); elapsed += 100)
        time::sleep_ms(100);
}

} // namespace

WifiManager::WifiManager() : worker_(&WifiManager::worker_main, this) {}

WifiManager::~WifiManager() {
    stopping_.store(true);
    if (worker_.joinable()) worker_.join();
}

bool WifiManager::connected() const { return connected_.load(); }

std::string WifiManager::ip() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return ip_;
}

std::string WifiManager::display_status() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return connected_.load() ? "NET " + ip_ : "NET " + state_;
}

std::string WifiManager::web_status_json() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::ostringstream out;
    out << "{\"connected\":" << (connected_.load() ? "true" : "false")
        << ",\"state\":\"" << json_escape(state_) << "\""
        << ",\"detail\":\"" << json_escape(detail_) << "\""
        << ",\"ip\":\"" << json_escape(ip_) << "\"}";
    return out.str();
}

void WifiManager::set_status(const std::string &state, const std::string &detail,
                             const std::string &ip, bool connected) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        state_ = state;
        detail_ = detail;
        ip_ = ip;
    }
    connected_.store(connected);
}

void WifiManager::worker_main() {
    try {
        const std::vector<std::string> interfaces = network::wifi::list_devices();
        if (interfaces.empty()) {
            set_status("NO-WIFI", "no interface", "", false);
            return;
        }
        network::wifi::Wifi wifi(interfaces.front());
        while (!stopping_.load()) {
            const std::string current_ip = wifi.get_ip();
            const bool online = !current_ip.empty();
            set_status(online ? "CONNECTED" : "OFFLINE", "", current_ip, online);
            interruptible_sleep(stopping_, 1000);
        }
    } catch (const std::exception &e) {
        set_status("ERROR", e.what(), "", false);
        log::error("WiFi status monitor stopped: %s", e.what());
    }
}
