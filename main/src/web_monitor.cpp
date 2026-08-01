#include "web_monitor.hpp"

#include "httplib.h"
#include "maix_basic.hpp"

#include <algorithm>
#include <dirent.h>
#include <sstream>
#include <sys/stat.h>
#include <utility>
#include <vector>

using namespace maix;

namespace {

bool safe_filename(const std::string &name) {
    if (name.empty() || name == "." || name == "..") return false;
    return name.find('/') == std::string::npos && name.find('\\') == std::string::npos &&
           name.find("..") == std::string::npos;
}

std::string json_escape(const std::string &value) {
    std::string out;
    for (const char ch : value) {
        if (ch == '\\') out += "\\\\";
        else if (ch == '"') out += "\\\"";
        else out += ch;
    }
    return out;
}

const char *INDEX_HTML = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H题滚球监控</title><style>
body{margin:0;background:#101418;color:#e9eef2;font:15px system-ui,sans-serif}main{max-width:780px;margin:auto;padding:16px}
h1{font-size:21px;margin:0 0 12px}.card{background:#1a2026;border:1px solid #303943;border-radius:10px;padding:12px;margin:10px 0}
img{display:block;width:100%;image-rendering:auto;background:#000;border-radius:6px}button,a.dl{border:0;border-radius:6px;padding:9px 13px;margin:4px;background:#2878d0;color:#fff;text-decoration:none;display:inline-block}
button.stop{background:#b73a3a}.muted{color:#9ba8b3}.file{display:flex;align-items:center;justify-content:space-between;border-top:1px solid #303943;padding:7px 0}
</style></head><body><main><h1>H题滚球监控 · 15 FPS</h1>
<div class="card"><div id="net">读取连接状态…</div><img id="stream" alt="实时识别画面"></div>
<div class="card"><button onclick="rec('start')">开始录像</button><button class="stop" onclick="rec('stop')">停止录像</button><span id="rec" class="muted"></span></div>
<div class="card"><b>已完成录像</b><div id="files" class="muted">正在读取…</div></div>
<script>
document.getElementById('stream').src='http://'+location.hostname+':__STREAM_PORT__/stream';
async function update(){let n=document.getElementById('net'),r=document.getElementById('rec'),d=document.getElementById('files');try{let s=await(await fetch('/api/status')).json();n.textContent=s.wifi.connected?'已连接 '+s.wifi.ip:'网络 '+s.wifi.state+' '+s.wifi.detail;r.textContent=s.recording?'录像中：'+s.file+'（'+s.frames+' 帧）':'未录像';let f=await(await fetch('/api/files')).json();d.innerHTML=f.length?f.map(x=>'<div class="file"><span>'+x.name+' · '+x.size_mb+' MB</span><a class="dl" href="/recordings/'+encodeURIComponent(x.name)+'" download>下载</a></div>').join(''):'暂无已完成录像';}catch(e){n.textContent='监控状态暂不可用';}}
async function rec(a){await fetch('/api/record/'+a,{method:'POST'});setTimeout(update,300)}
update();setInterval(update,1500);
</script></main></body></html>)HTML";

} // namespace

WebMonitor::WebMonitor(int port, int stream_port, std::string recordings_dir,
                       std::function<std::string()> wifi_json_provider)
    : port_(port), stream_port_(stream_port), recordings_dir_(std::move(recordings_dir)),
      wifi_json_provider_(std::move(wifi_json_provider)),
      server_(std::make_unique<httplib::Server>()), thread_(&WebMonitor::server_main, this) {}

WebMonitor::~WebMonitor() {
    if (server_) server_->stop();
    if (thread_.joinable()) thread_.join();
}

bool WebMonitor::take_start_record_request() { return start_record_requested_.exchange(false); }
bool WebMonitor::take_stop_record_request() { return stop_record_requested_.exchange(false); }

void WebMonitor::update_recording(bool recording, const std::string &filename, uint32_t frames) {
    recording_.store(recording);
    record_frames_.store(frames);
    std::lock_guard<std::mutex> lock(state_mutex_);
    recording_filename_ = filename;
}

std::string WebMonitor::status_json() const {
    std::string filename;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        filename = recording_filename_;
    }
    std::ostringstream out;
    out << "{\"wifi\":" << wifi_json_provider_()
        << ",\"recording\":" << (recording_.load() ? "true" : "false")
        << ",\"file\":\"" << json_escape(filename) << "\""
        << ",\"frames\":" << record_frames_.load() << '}';
    return out.str();
}

std::string WebMonitor::files_json() const {
    struct File { std::string name; long long size; time_t modified; };
    std::vector<File> files;
    if (DIR *dir = opendir(recordings_dir_.c_str())) {
        while (dirent *entry = readdir(dir)) {
            const std::string name(entry->d_name);
            if (!safe_filename(name) || name.size() < 4 || name.substr(name.size() - 4) != ".avi") continue;
            const std::string path = recordings_dir_ + '/' + name;
            struct stat info{};
            if (stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode))
                files.push_back({name, static_cast<long long>(info.st_size), info.st_mtime});
        }
        closedir(dir);
    }
    std::sort(files.begin(), files.end(), [](const File &a, const File &b) {
        return a.modified > b.modified;
    });
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < files.size(); ++i) {
        if (i) out << ',';
        out << "{\"name\":\"" << json_escape(files[i].name) << "\",\"size_mb\":"
            << (files[i].size * 10 / (1024 * 1024)) / 10.0 << '}';
    }
    out << ']';
    return out.str();
}

void WebMonitor::server_main() {
    std::string html(INDEX_HTML);
    const std::string marker = "__STREAM_PORT__";
    html.replace(html.find(marker), marker.size(), std::to_string(stream_port_));
    server_->Get("/", [html](const httplib::Request &, httplib::Response &res) {
        res.set_content(html, "text/html; charset=utf-8");
    });
    server_->Get("/api/status", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(status_json(), "application/json");
    });
    server_->Get("/api/files", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(files_json(), "application/json");
    });
    server_->Post("/api/record/start", [this](const httplib::Request &, httplib::Response &res) {
        stop_record_requested_.store(false);
        start_record_requested_.store(true);
        res.set_content("{\"ok\":true}", "application/json");
    });
    server_->Post("/api/record/stop", [this](const httplib::Request &, httplib::Response &res) {
        start_record_requested_.store(false);
        stop_record_requested_.store(true);
        res.set_content("{\"ok\":true}", "application/json");
    });
    server_->Get(R"(/recordings/([A-Za-z0-9_.-]+))",
                 [this](const httplib::Request &req, httplib::Response &res) {
        const std::string name = req.matches[1];
        if (!safe_filename(name) || name.size() < 4 || name.substr(name.size() - 4) != ".avi") {
            res.status = 400;
            return;
        }
        res.set_header("Content-Disposition", "attachment; filename=\"" + name + "\"");
        res.set_file_content(recordings_dir_ + '/' + name, "video/x-msvideo");
    });
    log::info("browser monitor listening on 0.0.0.0:%d", port_);
    if (!server_->listen("0.0.0.0", port_))
        log::error("browser monitor failed to listen on port %d", port_);
}
