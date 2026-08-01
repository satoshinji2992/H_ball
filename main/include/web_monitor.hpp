#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace httplib { class Server; }

class WebMonitor {
public:
    WebMonitor(int port, int stream_port, std::string recordings_dir,
               std::function<std::string()> wifi_json_provider);
    ~WebMonitor();

    WebMonitor(const WebMonitor &) = delete;
    WebMonitor &operator=(const WebMonitor &) = delete;

    bool take_start_record_request();
    bool take_stop_record_request();
    void update_recording(bool recording, const std::string &filename, uint32_t frame_count);

private:
    std::string status_json() const;
    std::string files_json() const;
    void server_main();

    int port_;
    int stream_port_;
    std::string recordings_dir_;
    std::function<std::string()> wifi_json_provider_;
    std::unique_ptr<httplib::Server> server_;
    std::thread thread_;
    std::atomic<bool> start_record_requested_{false};
    std::atomic<bool> stop_record_requested_{false};
    std::atomic<bool> recording_{false};
    std::atomic<uint32_t> record_frames_{0};
    mutable std::mutex state_mutex_;
    std::string recording_filename_;
};
