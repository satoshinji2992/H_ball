#include "connectivity.hpp"

#include "maix_basic.hpp"
#include "maix_wifi.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

using namespace maix;

namespace {

std::string trim(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string json_escape(const std::string &value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += ch; break;
        }
    }
    return out;
}

void interruptible_sleep(const std::atomic<bool> &stopping,
                         const std::atomic<bool> &wake_requested, int milliseconds) {
    for (int elapsed = 0;
         elapsed < milliseconds && !stopping.load() && !wake_requested.load(); elapsed += 100)
        time::sleep_ms(100);
}

} // namespace

NetworkConfig load_network_config(const std::string &path) {
    NetworkConfig cfg;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        try {
            if (key == "ssid") cfg.ssid = value;
            else if (key == "password") cfg.password = value;
            else if (key == "stream_port") cfg.stream_port = std::clamp(std::stoi(value), 1, 65535);
            else if (key == "web_port") cfg.web_port = std::clamp(std::stoi(value), 1, 65535);
            else if (key == "jpeg_quality") cfg.jpeg_quality = std::clamp(std::stoi(value), 35, 95);
            else if (key == "recordings_dir") cfg.recordings_dir = value;
        } catch (const std::exception &e) {
            log::warn("ignored invalid network config %s: %s", key.c_str(), e.what());
        }
    }
    return cfg;
}

std::string resolve_config_path(const std::string &config_path, const std::string &value) {
    if (value.empty() || value.front() == '/') return value;
    const size_t slash = config_path.find_last_of('/');
    if (slash == std::string::npos) return value;
    return config_path.substr(0, slash + 1) + value;
}

WifiManager::WifiManager(std::string config_path)
    : config_path_(std::move(config_path)), worker_(&WifiManager::worker_main, this) {}

WifiManager::~WifiManager() {
    stopping_.store(true);
    reconnect_requested_.store(true);
    if (worker_.joinable()) worker_.join();
}

void WifiManager::request_reconnect() {
    reconnect_requested_.store(true);
    set_status("REQUESTED", "reload config", "", false);
}

bool WifiManager::connected() const { return connected_.load(); }

std::string WifiManager::ip() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return ip_;
}

std::string WifiManager::display_status() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (connected_.load()) return "NET " + ip_;
    if (detail_.empty()) return "NET " + state_;
    std::string text = "NET " + state_ + " " + detail_;
    if (text.size() > 27) text.resize(27);
    return text;
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
        bool first_attempt = true;
        while (!stopping_.load()) {
            const NetworkConfig cfg = load_network_config(config_path_);
            const bool forced = reconnect_requested_.exchange(false);
            if (cfg.ssid.empty()) {
                set_status("NO-CONFIG", "set network.cfg", "", false);
                interruptible_sleep(stopping_, reconnect_requested_, 2000);
                continue;
            }

            if (!forced && wifi.is_connected() && wifi.get_ssid(false) == cfg.ssid) {
                const std::string current_ip = wifi.get_ip();
                set_status("CONNECTED", cfg.ssid, current_ip, !current_ip.empty());
                interruptible_sleep(stopping_, reconnect_requested_, 1500);
                continue;
            }

            if (wifi.is_connected()) wifi.disconnect();
            set_status("SCANNING", cfg.ssid, "", false);
            bool found = false;
            if (wifi.start_scan() == err::ERR_NONE) {
                interruptible_sleep(stopping_, reconnect_requested_, 1800);
                for (auto &ap : wifi.get_scan_result()) {
                    if (ap.ssid_str() == cfg.ssid) {
                        found = true;
                        break;
                    }
                }
                wifi.stop_scan();
            }
            if (stopping_.load()) break;
            if (!found) {
                set_status("NOT-FOUND", cfg.ssid, "", false);
                interruptible_sleep(stopping_, reconnect_requested_, 10000);
                continue;
            }

            set_status("CONNECTING", cfg.ssid, "", false);
            const err::Err result = wifi.connect(cfg.ssid, cfg.password, true, 15);
            const std::string current_ip = result == err::ERR_NONE ? wifi.get_ip() : "";
            if (result == err::ERR_NONE && !current_ip.empty()) {
                set_status("CONNECTED", cfg.ssid, current_ip, true);
                log::info("WiFi connected: %s, monitor http://%s:%d", cfg.ssid.c_str(),
                          current_ip.c_str(), cfg.web_port);
            } else {
                set_status("FAILED", cfg.ssid, "", false);
                if (first_attempt) log::warn("WiFi connection failed; retrying in background");
            }
            first_attempt = false;
            interruptible_sleep(stopping_, reconnect_requested_, 5000);
        }
    } catch (const std::exception &e) {
        set_status("ERROR", e.what(), "", false);
        log::error("WiFi worker stopped: %s", e.what());
    }
}
