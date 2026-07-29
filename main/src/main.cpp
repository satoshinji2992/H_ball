#include "maix_basic.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_nn_yolo11.hpp"
#include "maix_rtsp.hpp"
#include "maix_touchscreen.hpp"
#include "maix_uart.hpp"
#include "main.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using namespace maix;

namespace {

constexpr int CAM_W = 320;
constexpr int CAM_H = 240;
constexpr int STREAM_W = 640;
constexpr int STREAM_H = 480;
constexpr int STREAM_FPS = 20;
constexpr uint8_t FRAME_HEAD_0 = 0xAA;
constexpr uint8_t FRAME_HEAD_1 = 0xBB;
constexpr uint8_t FRAME_DATA_SIZE = 0x10;

enum class TestMode : uint8_t {
    MONITOR = 0,
    STATIC_SWEEP = 1,
    AB_CENTER = 2,
    LAP_CENTER = 3,
    LAP_CUSTOM = 4,
};

struct Config {
    cv::Point2f left_px{45.0F, 120.0F};
    cv::Point2f right_px{275.0F, 120.0F};
    int roi_x1 = 8;
    int roi_y1 = 96;
    int roi_x2 = 312;
    int roi_y2 = 142;
    float left_cm = -10.0F;
    float right_cm = 10.0F;
    float confidence = 0.40F;
    float iou = 0.45F;
};

struct Button {
    int x, y, w, h;
    const char *text;
};

struct Estimate {
    bool valid = false;
    cv::Point2f center{};
    float position_cm = 0.0F;
    float velocity_cm_s = 0.0F;
    float score = 0.0F;
};

template <typename T>
static void append_le(std::vector<uint8_t> &out, T value) {
    static_assert(std::is_integral<T>::value, "integral type required");
    using U = typename std::make_unsigned<T>::type;
    U v = static_cast<U>(value);
    for (size_t i = 0; i < sizeof(T); ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

static void append_double_le(std::vector<uint8_t> &out, double value) {
    static_assert(sizeof(double) == sizeof(uint64_t), "64-bit IEEE-754 double required");
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_le<uint64_t>(out, bits);
}

static Config load_config(const std::string &path) {
    Config cfg;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        if (key == "left_px" || key == "right_px") {
            float x = 0, y = 0;
            if (std::sscanf(value.c_str(), "%f,%f", &x, &y) == 2) {
                (key == "left_px" ? cfg.left_px : cfg.right_px) = {x, y};
            }
        } else if (key == "left_cm") cfg.left_cm = std::stof(value);
        else if (key == "right_cm") cfg.right_cm = std::stof(value);
        else if (key == "roi_x1") cfg.roi_x1 = std::stoi(value);
        else if (key == "roi_y1") cfg.roi_y1 = std::stoi(value);
        else if (key == "roi_x2") cfg.roi_x2 = std::stoi(value);
        else if (key == "roi_y2") cfg.roi_y2 = std::stoi(value);
        else if (key == "confidence") cfg.confidence = std::stof(value);
        else if (key == "iou") cfg.iou = std::stof(value);
    }
    return cfg;
}

static void save_config(const std::string &path, const Config &cfg) {
    std::ofstream file(path, std::ios::trunc);
    file << "# Camera calibration for the 25 cm grooved rod\n"
         << "left_px=" << cfg.left_px.x << ',' << cfg.left_px.y << "\n"
         << "right_px=" << cfg.right_px.x << ',' << cfg.right_px.y << "\n"
         << "left_cm=" << cfg.left_cm << "\nright_cm=" << cfg.right_cm
         << "\nroi_x1=" << cfg.roi_x1 << "\nroi_y1=" << cfg.roi_y1
         << "\nroi_x2=" << cfg.roi_x2 << "\nroi_y2=" << cfg.roi_y2
         << "\nconfidence=" << cfg.confidence << "\niou=" << cfg.iou << "\n";
}

static bool inside(const Button &b, int x, int y) {
    return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
}

static void draw_button(image::Image &img, const Button &b, const image::Color &color) {
    img.draw_rect(b.x, b.y, b.w, b.h, color, 2);
    img.draw_string(b.x + 4, b.y + 7, b.text, color, 0.8F);
}

static const char *mode_name(TestMode mode) {
    switch (mode) {
    case TestMode::MONITOR: return "MON";
    case TestMode::STATIC_SWEEP: return "SWEEP";
    case TestMode::AB_CENTER: return "AB-O";
    case TestMode::LAP_CENTER: return "LAP-O";
    case TestMode::LAP_CUSTOM: return "LAP-X";
    }
    return "?";
}

class PositionFilter {
public:
    Estimate update(bool measured, cv::Point2f center, float position, float score, uint64_t now_ms) {
        if (!measured) {
            if (initialized_ && now_ms - last_seen_ms_ <= 250) {
                position_ += velocity_ * static_cast<float>(now_ms - last_ms_) * 0.001F;
                last_ms_ = now_ms;
                return {true, last_center_, position_, velocity_, 0.0F};
            }
            initialized_ = false;
            return {};
        }
        if (!initialized_ || now_ms <= last_ms_ || now_ms - last_ms_ > 300) {
            position_ = position;
            velocity_ = 0.0F;
            initialized_ = true;
        } else {
            const float dt = std::max(0.001F, static_cast<float>(now_ms - last_ms_) * 0.001F);
            const float predicted = position_ + velocity_ * dt;
            const float residual = position - predicted;
            position_ = predicted + 0.70F * residual;
            velocity_ = std::clamp(velocity_ + 0.18F * residual / dt, -100.0F, 100.0F);
        }
        last_center_ = center;
        last_ms_ = last_seen_ms_ = now_ms;
        return {true, center, position_, velocity_, score};
    }

private:
    bool initialized_ = false;
    float position_ = 0.0F;
    float velocity_ = 0.0F;
    cv::Point2f last_center_{};
    uint64_t last_ms_ = 0;
    uint64_t last_seen_ms_ = 0;
};

class SerialLink {
public:
    explicit SerialLink(const std::string &port) {
        if (port.empty() || port == "none") return;
        try {
            uart_ = std::make_unique<peripheral::uart::UART>(port, 115200);
            log::info("vision UART: %s @ 115200 8N1", port.c_str());
        } catch (const std::exception &e) {
            log::warn("UART disabled: %s", e.what());
        }
    }

    void send(const Estimate &e, float target_cm) {
        if (!uart_ || !uart_->is_open()) return;
        std::vector<uint8_t> frame;
        frame.reserve(20);
        frame.push_back(FRAME_HEAD_0);
        frame.push_back(FRAME_HEAD_1);
        frame.push_back(FRAME_DATA_SIZE);

        const double invalid = std::numeric_limits<double>::quiet_NaN();
        const double delta_x = e.valid ? static_cast<double>(target_cm - e.position_cm) : invalid;
        const double delta_y = e.valid ? static_cast<double>(e.velocity_cm_s) : invalid;
        append_double_le(frame, delta_x);
        append_double_le(frame, delta_y);

        uint8_t checksum = 0;
        for (size_t i = 3; i < frame.size(); ++i)
            checksum = static_cast<uint8_t>(checksum + frame[i]);
        frame.push_back(checksum);
        if (uart_->write(frame.data(), static_cast<int>(frame.size())) < 0)
            log::warn("UART vision packet write failed");
    }

private:
    std::unique_ptr<peripheral::uart::UART> uart_;
};

static float point_to_cm(cv::Point2f p, const Config &cfg, float *perpendicular_px = nullptr) {
    const cv::Point2f axis = cfg.right_px - cfg.left_px;
    const float len2 = axis.dot(axis);
    if (len2 < 100.0F) return 0.0F;
    const cv::Point2f rel = p - cfg.left_px;
    const float t = rel.dot(axis) / len2;
    if (perpendicular_px)
        *perpendicular_px = std::abs(rel.x * axis.y - rel.y * axis.x) / std::sqrt(len2);
    return cfg.left_cm + t * (cfg.right_cm - cfg.left_cm);
}

static const nn::Object *select_ball(nn::Objects &objects, const Config &cfg) {
    const nn::Object *best = nullptr;
    float best_value = -1.0F;
    const float axis_len = cv::norm(cfg.right_px - cfg.left_px);
    for (size_t i = 0; i < objects.size(); ++i) {
        const nn::Object &o = objects.at(static_cast<int>(i));
        const cv::Point2f center(o.x + o.w * 0.5F, o.y + o.h * 0.5F);
        if (center.x < cfg.roi_x1 || center.x > cfg.roi_x2 ||
            center.y < cfg.roi_y1 || center.y > cfg.roi_y2) continue;
        float perpendicular = 0.0F;
        const float cm = point_to_cm(center, cfg, &perpendicular);
        const float cm_lo = std::min(cfg.left_cm, cfg.right_cm) - 3.0F;
        const float cm_hi = std::max(cfg.left_cm, cfg.right_cm) + 3.0F;
        if (perpendicular > std::max(25.0F, axis_len * 0.18F) || cm < cm_lo || cm > cm_hi) continue;
        const float value = o.score - 0.002F * perpendicular;
        if (value > best_value) { best_value = value; best = &o; }
    }
    return best;
}

int app_main(int argc, char **argv) {
    if (argc < 2) throw err::Exception(err::ERR_ARGS,
        "usage: h_ball_balance MODEL.mud [uart=/dev/ttyS0|none] [config]");
    const std::string model_path = argv[1];
    const std::string uart_port = argc > 2 ? argv[2] : "/dev/ttyS0";
    const std::string config_path = argc > 3 ? argv[3] : "./balance_calibration.cfg";
    Config cfg = load_config(config_path);

    nn::YOLO11 detector("", false);
    err::check_raise(detector.load(model_path), "failed to load steel-ball model");
    const image::Format detector_format = detector.input_format();

    camera::Camera stream_camera(STREAM_W, STREAM_H, image::FMT_YVU420SP, nullptr, STREAM_FPS, 3);
    std::unique_ptr<camera::Camera> camera(
        stream_camera.add_channel(CAM_W, CAM_H, detector_format, STREAM_FPS, 3));
    err::check_null_raise(camera.get(), "failed to create detector channel");
    display::Display display;
    touchscreen::TouchScreen touch;
    touch.clear();
    SerialLink serial(uart_port);

    rtsp::Rtsp rtsp("", 8554, STREAM_FPS, rtsp::RTSP_STREAM_H264, 2 * 1000 * 1000);
    err::check_raise(rtsp.bind_camera(&stream_camera), "RTSP camera bind failed");
    rtsp::Region *overlay_region = rtsp.add_region(0, 0, STREAM_W, STREAM_H, image::FMT_BGRA8888);
    err::check_null_raise(overlay_region, "RTSP overlay creation failed");
    err::check_raise(rtsp.start(), "RTSP start failed");
    for (const auto &url : rtsp.get_urls()) log::info("RTSP: %s", url.c_str());

    const Button cal_l{4, 202, 51, 34, "CAL-L"};
    const Button cal_r{58, 202, 51, 34, "CAL-R"};
    const Button mode_b{112, 202, 61, 34, "MODE"};
    const Button target_down{176, 202, 37, 34, "T-"};
    const Button target_up{216, 202, 37, 34, "T+"};
    const Button exit_b{258, 4, 58, 36, "EXIT"};
    TestMode mode = TestMode::MONITOR;
    float target_cm = 0.0F;
    bool prev_pressed = false;
    PositionFilter filter;
    Estimate estimate;
    uint64_t fps_window_start_ms = time::ticks_ms();
    uint32_t fps_frame_count = 0;

    while (!app::need_exit()) {
        const uint64_t now = time::ticks_ms();
        std::unique_ptr<image::Image> frame(camera->read());
        if (!frame) continue;
        std::unique_ptr<nn::Objects> objects(
            detector.detect(*frame, cfg.confidence, cfg.iou, image::FIT_CONTAIN));
        if (!objects) continue;
        const nn::Object *ball = select_ball(*objects, cfg);
        if (ball) {
            const cv::Point2f center(ball->x + ball->w * 0.5F, ball->y + ball->h * 0.5F);
            estimate = filter.update(true, center, point_to_cm(center, cfg), ball->score, now);
        } else {
            estimate = filter.update(false, {}, 0.0F, 0.0F, now);
        }

        int tx = 0, ty = 0;
        bool pressed = false;
        if (touch.read(tx, ty, pressed) == err::ERR_NONE) {
            const auto mapped = image::resize_map_pos(display.width(), display.height(), CAM_W, CAM_H,
                                                       image::FIT_CONTAIN, tx, ty, 1, 1);
            const int x = mapped.empty() ? -1 : mapped[0];
            const int y = mapped.size() < 2 ? -1 : mapped[1];
            if (pressed && !prev_pressed) {
                if (inside(exit_b, x, y)) break;
                if (inside(cal_l, x, y) && ball) {
                    cfg.left_px = estimate.center; save_config(config_path, cfg);
                    log::info("captured %.1fcm point at (%.1f, %.1f)", cfg.left_cm,
                              cfg.left_px.x, cfg.left_px.y);
                } else if (inside(cal_r, x, y) && ball) {
                    cfg.right_px = estimate.center; save_config(config_path, cfg);
                    log::info("captured %.1fcm point at (%.1f, %.1f)", cfg.right_cm,
                              cfg.right_px.x, cfg.right_px.y);
                } else if (inside(mode_b, x, y)) {
                    mode = static_cast<TestMode>((static_cast<int>(mode) + 1) % 5);
                    if (mode != TestMode::LAP_CUSTOM) target_cm = 0.0F;
                } else if (inside(target_down, x, y)) {
                    target_cm = std::max(-10.0F, target_cm - 0.5F);
                    mode = TestMode::LAP_CUSTOM;
                } else if (inside(target_up, x, y)) {
                    target_cm = std::min(10.0F, target_cm + 0.5F);
                    mode = TestMode::LAP_CUSTOM;
                }
            }
            prev_pressed = pressed;
        }

        serial.send(estimate, target_cm);

        const image::Color valid_color = estimate.valid ? image::COLOR_GREEN : image::COLOR_RED;
        frame->draw_rect(cfg.roi_x1, cfg.roi_y1, cfg.roi_x2 - cfg.roi_x1,
                         cfg.roi_y2 - cfg.roi_y1, image::COLOR_YELLOW, 1);
        frame->draw_line(static_cast<int>(cfg.left_px.x), static_cast<int>(cfg.left_px.y),
                         static_cast<int>(cfg.right_px.x), static_cast<int>(cfg.right_px.y),
                         image::COLOR_BLUE, 2);
        frame->draw_cross(static_cast<int>(cfg.left_px.x), static_cast<int>(cfg.left_px.y),
                          image::COLOR_BLUE, 8, 2);
        frame->draw_cross(static_cast<int>(cfg.right_px.x), static_cast<int>(cfg.right_px.y),
                          image::COLOR_BLUE, 8, 2);
        if (ball) {
            frame->draw_rect(ball->x, ball->y, ball->w, ball->h, valid_color, 3);
            frame->draw_cross(static_cast<int>(estimate.center.x), static_cast<int>(estimate.center.y),
                              image::COLOR_RED, 10, 2);
        }
        char status[160];
        std::snprintf(status, sizeof(status), "%s x:%+.2fcm v:%+.1f T:%+.1f",
                      mode_name(mode), estimate.position_cm, estimate.velocity_cm_s, target_cm);
        frame->draw_string(4, 4, status, valid_color, 1.0F);
        draw_button(*frame, cal_l, image::COLOR_YELLOW);
        draw_button(*frame, cal_r, image::COLOR_YELLOW);
        draw_button(*frame, mode_b, image::COLOR_GREEN);
        draw_button(*frame, target_down, image::COLOR_BLUE);
        draw_button(*frame, target_up, image::COLOR_BLUE);
        draw_button(*frame, exit_b, image::COLOR_RED);

        std::unique_ptr<image::Image> overlay(overlay_region->get_canvas());
        if (overlay) {
            overlay->draw_string(8, 8, status, valid_color, 2.0F);
            overlay->draw_line(static_cast<int>(cfg.left_px.x * 2), static_cast<int>(cfg.left_px.y * 2),
                               static_cast<int>(cfg.right_px.x * 2), static_cast<int>(cfg.right_px.y * 2),
                               image::COLOR_BLUE, 4);
            if (ball) {
                overlay->draw_rect(ball->x * 2, ball->y * 2, ball->w * 2, ball->h * 2,
                                   image::COLOR_GREEN, 6);
                overlay->draw_cross(static_cast<int>(estimate.center.x * 2),
                                    static_cast<int>(estimate.center.y * 2),
                                    image::COLOR_RED, 20, 4);
            }
            overlay_region->update_canvas();
        }
        display.show(*frame);

        ++fps_frame_count;
        const uint64_t fps_now_ms = time::ticks_ms();
        const uint64_t fps_elapsed_ms = fps_now_ms - fps_window_start_ms;
        if (fps_elapsed_ms >= 1000) {
            const float runtime_fps = static_cast<float>(fps_frame_count) * 1000.0F /
                                      static_cast<float>(fps_elapsed_ms);
            log::info("runtime FPS: %.2f (%u frames / %llu ms)", runtime_fps,
                      static_cast<unsigned>(fps_frame_count),
                      static_cast<unsigned long long>(fps_elapsed_ms));
            fps_frame_count = 0;
            fps_window_start_ms = fps_now_ms;
        }
    }
    rtsp.stop();
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    sys::register_default_signal_handle();
    CATCH_EXCEPTION_RUN_RETURN(app_main, -1, argc, argv);
}
