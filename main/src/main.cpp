#include "maix_basic.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_nn_yolo11.hpp"
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
#include <memory>
#include <sstream>
#include <string>

using namespace maix;

namespace {

constexpr int CAM_W = 320;
constexpr int CAM_H = 240;
constexpr int CAMERA_FPS = 90;
constexpr int INFERENCE_ROI_X = 0;
constexpr int INFERENCE_ROI_Y = 87;
constexpr int INFERENCE_ROI_W = 320;
constexpr int INFERENCE_ROI_H = 64;
constexpr uint64_t UI_UPDATE_INTERVAL_MS = 100;
constexpr size_t VISION_FRAME_SIZE = 11;
constexpr uint64_t CAMERA_HALF_FRAME_MS = (500 + CAMERA_FPS / 2) / CAMERA_FPS;
constexpr uint64_t MAX_LATENCY_COMPENSATION_MS = 100;
constexpr uint64_t MAX_MISSING_PREDICTION_MS = 100;
constexpr float THREE_POINT_POSITION_TOLERANCE_CM = 1.5F;
constexpr float THREE_POINT_VELOCITY_TOLERANCE_CM_S = 2.0F;
constexpr uint64_t THREE_POINT_STABLE_MS = 500;

enum class BallMode : uint8_t {
    CENTER_BALANCE = 0,
    THREE_POINT = 1,
    FIXED_POINT = 2,
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

struct BallDetection {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    float score = 0.0F;
};

static Estimate extrapolate_estimate(const Estimate &estimate, uint64_t delay_ms) {
    if (!estimate.valid) return estimate;
    Estimate predicted = estimate;
    const uint64_t bounded_delay_ms = std::min(delay_ms, MAX_LATENCY_COMPENSATION_MS);
    predicted.position_cm += predicted.velocity_cm_s *
                             static_cast<float>(bounded_delay_ms) * 0.001F;
    return predicted;
}

class ThreePointSequence {
public:
    void reset() {
        phase_ = 0;
        stable_since_ms_ = 0;
        complete_ = false;
    }

    float target_cm() const { return TARGETS_CM[phase_]; }
    bool complete() const { return complete_; }

    void update(const Estimate &estimate, uint64_t now_ms) {
        if (complete_ || !estimate.valid || estimate.score <= 0.0F ||
            std::abs(target_cm() - estimate.position_cm) > THREE_POINT_POSITION_TOLERANCE_CM) {
            stable_since_ms_ = 0;
            return;
        }

        // +5 cm is only a pass-through waypoint: switch to -5 cm as soon as
        // the position enters its tolerance band, regardless of velocity.
        if (phase_ == 0) {
            ++phase_;
            stable_since_ms_ = 0;
            log::info("3-POINT target advanced to %+.1fcm", target_cm());
            return;
        }

        // The final -5 cm target still requires a stationary 500 ms hold.
        if (std::abs(estimate.velocity_cm_s) > THREE_POINT_VELOCITY_TOLERANCE_CM_S) {
            stable_since_ms_ = 0;
            return;
        }
        if (stable_since_ms_ == 0) {
            stable_since_ms_ = now_ms;
            return;
        }
        if (now_ms - stable_since_ms_ < THREE_POINT_STABLE_MS) return;
        stable_since_ms_ = 0;
        complete_ = true;
        log::info("3-POINT sequence complete");
    }

private:
    // The ball is held at O while waiting for START. The active sequence then
    // moves directly to +5 cm and finally -5 cm.
    static constexpr std::array<float, 2> TARGETS_CM{5.0F, -5.0F};
    size_t phase_ = 0;
    uint64_t stable_since_ms_ = 0;
    bool complete_ = false;
};

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

static const char *mode_name(BallMode mode) {
    switch (mode) {
    case BallMode::CENTER_BALANCE: return "CENTER";
    case BallMode::THREE_POINT: return "3-POINT";
    case BallMode::FIXED_POINT: return "FIXED";
    }
    return "?";
}

class PositionFilter {
public:
    Estimate update(bool measured, cv::Point2f center, float position, float score, uint64_t now_ms) {
        if (!measured) {
            if (initialized_ && now_ms - last_seen_ms_ <= MAX_MISSING_PREDICTION_MS) {
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
        std::array<uint8_t, VISION_FRAME_SIZE> frame{
            '[', 'N', 'a', 'N', ' ', 'N', 'a', 'N', ' ', '*', ']'
        };

        if (e.valid) {
            const int error_mm = std::clamp(
                static_cast<int>(std::lround((target_cm - e.position_cm) * 10.0F)), -999, 999);
            const int velocity_mm_s = std::clamp(
                static_cast<int>(std::lround(e.velocity_cm_s * 10.0F)), -999, 999);
            char error[5] = {};
            char velocity[5] = {};
            std::snprintf(error, sizeof(error), "%+04d", error_mm);
            std::snprintf(velocity, sizeof(velocity), "%+04d", velocity_mm_s);
            std::memcpy(&frame[1], error, 4);
            std::memcpy(&frame[5], velocity, 4);
        }

        const int written = uart_->write(frame.data(), static_cast<int>(frame.size()));
        if (written != static_cast<int>(frame.size()))
            log::warn("UART vision packet short write: %d/%u", written,
                      static_cast<unsigned>(frame.size()));
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

static cv::Point2f cm_to_point(float cm, const Config &cfg) {
    const float span_cm = cfg.right_cm - cfg.left_cm;
    if (std::abs(span_cm) < 0.001F) return cfg.left_px;
    const float t = (cm - cfg.left_cm) / span_cm;
    return cfg.left_px + (cfg.right_px - cfg.left_px) * t;
}

static bool select_ball(nn::Objects &objects, const Config &cfg, BallDetection &best) {
    bool found = false;
    float best_value = -1.0F;
    const float axis_len = cv::norm(cfg.right_px - cfg.left_px);
    for (size_t i = 0; i < objects.size(); ++i) {
        const nn::Object &o = objects.at(static_cast<int>(i));
        const BallDetection candidate{o.x + INFERENCE_ROI_X, o.y + INFERENCE_ROI_Y,
                                      o.w, o.h, o.score};
        const cv::Point2f center(candidate.x + candidate.w * 0.5F,
                                 candidate.y + candidate.h * 0.5F);
        if (center.x < cfg.roi_x1 || center.x > cfg.roi_x2 ||
            center.y < cfg.roi_y1 || center.y > cfg.roi_y2) continue;
        float perpendicular = 0.0F;
        const float cm = point_to_cm(center, cfg, &perpendicular);
        const float cm_lo = std::min(cfg.left_cm, cfg.right_cm) - 3.0F;
        const float cm_hi = std::max(cfg.left_cm, cfg.right_cm) + 3.0F;
        if (perpendicular > std::max(25.0F, axis_len * 0.18F) || cm < cm_lo || cm > cm_hi) continue;
        const float value = o.score - 0.002F * perpendicular;
        if (value > best_value) {
            best_value = value;
            best = candidate;
            found = true;
        }
    }
    return found;
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

    camera::Camera camera(CAM_W, CAM_H, detector_format, nullptr, CAMERA_FPS, 1);
    display::Display display;
    touchscreen::TouchScreen touch;
    touch.clear();
    SerialLink serial(uart_port);

    const Button cal_l{4, 202, 51, 34, "CAL-L"};
    const Button cal_r{58, 202, 51, 34, "CAL-R"};
    const Button mode_b{112, 202, 61, 34, "MODE"};
    const Button target_down{176, 202, 37, 34, "T-"};
    const Button target_up{216, 202, 37, 34, "T+"};
    const Button start_b{256, 202, 60, 34, "START"};
    const Button exit_b{258, 4, 58, 36, "EXIT"};
    BallMode mode = BallMode::CENTER_BALANCE;
    float target_cm = 0.0F;
    float fixed_target_cm = 0.0F;
    bool three_point_running = false;
    bool prev_pressed = false;
    PositionFilter filter;
    ThreePointSequence three_point;
    Estimate estimate;
    uint64_t fps_window_start_ms = time::ticks_ms();
    uint64_t last_ui_update_ms = 0;
    uint32_t fps_frame_count = 0;

    while (!app::need_exit()) {
        std::unique_ptr<image::Image> frame(camera.read());
        if (!frame) continue;
        const uint64_t capture_done_ms = time::ticks_ms();
        const uint64_t measurement_ms = capture_done_ms >= CAMERA_HALF_FRAME_MS
                                      ? capture_done_ms - CAMERA_HALF_FRAME_MS : 0;
        std::unique_ptr<image::Image> inference_roi(
            frame->crop(INFERENCE_ROI_X, INFERENCE_ROI_Y, INFERENCE_ROI_W, INFERENCE_ROI_H));
        if (!inference_roi) continue;
        std::unique_ptr<nn::Objects> objects(
            detector.detect(*inference_roi, cfg.confidence, cfg.iou, image::FIT_CONTAIN));
        if (!objects) continue;
        BallDetection ball;
        const bool ball_found = select_ball(*objects, cfg, ball);
        if (ball_found) {
            const cv::Point2f center(ball.x + ball.w * 0.5F, ball.y + ball.h * 0.5F);
            estimate = filter.update(true, center, point_to_cm(center, cfg), ball.score,
                                     measurement_ms);
        } else {
            estimate = filter.update(false, {}, 0.0F, 0.0F, measurement_ms);
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
                if (inside(cal_l, x, y) && ball_found) {
                    cfg.left_px = estimate.center; save_config(config_path, cfg);
                    log::info("captured %.1fcm point at (%.1f, %.1f)", cfg.left_cm,
                              cfg.left_px.x, cfg.left_px.y);
                } else if (inside(cal_r, x, y) && ball_found) {
                    cfg.right_px = estimate.center; save_config(config_path, cfg);
                    log::info("captured %.1fcm point at (%.1f, %.1f)", cfg.right_cm,
                              cfg.right_px.x, cfg.right_px.y);
                } else if (inside(mode_b, x, y)) {
                    mode = static_cast<BallMode>((static_cast<int>(mode) + 1) % 3);
                    if (mode == BallMode::THREE_POINT) {
                        three_point.reset();
                        three_point_running = false;
                        target_cm = 0.0F;
                        log::info("3-POINT armed; touch START to begin");
                    } else if (mode == BallMode::CENTER_BALANCE) {
                        three_point_running = false;
                        target_cm = 0.0F;
                    } else {
                        three_point_running = false;
                        target_cm = fixed_target_cm;
                    }
                } else if (inside(target_down, x, y)) {
                    fixed_target_cm = std::max(-10.0F, fixed_target_cm - 0.5F);
                    target_cm = fixed_target_cm;
                    mode = BallMode::FIXED_POINT;
                    three_point_running = false;
                } else if (inside(target_up, x, y)) {
                    fixed_target_cm = std::min(10.0F, fixed_target_cm + 0.5F);
                    target_cm = fixed_target_cm;
                    mode = BallMode::FIXED_POINT;
                    three_point_running = false;
                } else if (inside(start_b, x, y) && mode == BallMode::THREE_POINT) {
                    three_point.reset();
                    three_point_running = true;
                    target_cm = three_point.target_cm();
                    log::info("3-POINT started; target %+.1fcm", target_cm);
                }
            }
            prev_pressed = pressed;
        }

        const uint64_t send_ms = time::ticks_ms();
        const uint64_t measurement_age_ms = send_ms >= measurement_ms ? send_ms - measurement_ms : 0;
        Estimate control_estimate = extrapolate_estimate(estimate, measurement_age_ms);
        if (mode == BallMode::THREE_POINT && three_point_running) {
            three_point.update(control_estimate, send_ms);
            target_cm = three_point.target_cm();
        }
        serial.send(control_estimate, target_cm);

        if (send_ms - last_ui_update_ms >= UI_UPDATE_INTERVAL_MS) {
            last_ui_update_ms = send_ms;
            const image::Color valid_color = estimate.valid ? image::COLOR_GREEN : image::COLOR_RED;
            frame->draw_rect(cfg.roi_x1, cfg.roi_y1, cfg.roi_x2 - cfg.roi_x1,
                             cfg.roi_y2 - cfg.roi_y1, image::COLOR_YELLOW, 1);
            frame->draw_rect(INFERENCE_ROI_X, INFERENCE_ROI_Y,
                             INFERENCE_ROI_W, INFERENCE_ROI_H, image::COLOR_GRAY, 1);
            frame->draw_line(static_cast<int>(cfg.left_px.x), static_cast<int>(cfg.left_px.y),
                             static_cast<int>(cfg.right_px.x), static_cast<int>(cfg.right_px.y),
                             image::COLOR_BLUE, 2);
            frame->draw_cross(static_cast<int>(cfg.left_px.x), static_cast<int>(cfg.left_px.y),
                              image::COLOR_BLUE, 8, 2);
            frame->draw_cross(static_cast<int>(cfg.right_px.x), static_cast<int>(cfg.right_px.y),
                              image::COLOR_BLUE, 8, 2);
            const cv::Point2f target_px = cm_to_point(target_cm, cfg);
            frame->draw_cross(static_cast<int>(target_px.x), static_cast<int>(target_px.y),
                              image::COLOR_PURPLE, 12, 3);
            frame->draw_string(static_cast<int>(target_px.x) - 12,
                               std::max(22, static_cast<int>(target_px.y) - 24),
                               "TARGET", image::COLOR_PURPLE, 0.7F);
            if (ball_found) {
                frame->draw_rect(ball.x, ball.y, ball.w, ball.h, valid_color, 3);
                frame->draw_cross(static_cast<int>(estimate.center.x),
                                  static_cast<int>(estimate.center.y), image::COLOR_RED, 10, 2);
            }
            char mode_status[24] = {};
            if (mode == BallMode::THREE_POINT) {
                if (!three_point_running) {
                    std::snprintf(mode_status, sizeof(mode_status), "3PT-WAIT");
                } else {
                    std::snprintf(mode_status, sizeof(mode_status), "3PT%+.0f%s",
                                  target_cm, three_point.complete() ? "-DONE" : "");
                }
            } else {
                std::snprintf(mode_status, sizeof(mode_status), "%s", mode_name(mode));
            }
            char status[160];
            std::snprintf(status, sizeof(status), "%s x:%+.2f v:%+.1f T:%+.1f L:%llums",
                          mode_status, control_estimate.position_cm, control_estimate.velocity_cm_s,
                          target_cm, static_cast<unsigned long long>(measurement_age_ms));
            frame->draw_string(4, 4, status, valid_color, 1.0F);
            draw_button(*frame, cal_l, image::COLOR_YELLOW);
            draw_button(*frame, cal_r, image::COLOR_YELLOW);
            draw_button(*frame, mode_b, image::COLOR_GREEN);
            draw_button(*frame, target_down, image::COLOR_BLUE);
            draw_button(*frame, target_up, image::COLOR_BLUE);
            draw_button(*frame, start_b, mode == BallMode::THREE_POINT
                                            ? image::COLOR_GREEN : image::COLOR_GRAY);
            draw_button(*frame, exit_b, image::COLOR_RED);

            display.show(*frame);
        }

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
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    sys::register_default_signal_handle();
    CATCH_EXCEPTION_RUN_RETURN(app_main, -1, argc, argv);
}
