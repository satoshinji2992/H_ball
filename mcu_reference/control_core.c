#include "control_core.h"
#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void control_init(control_state_t *s) {
    memset(s, 0, sizeof(*s));
    s->gains = (control_gains_t){
        .line_kp = 0.55f, .line_ki = 0.02f, .line_kd = 0.08f,
        .ball_kp = 0.18f, .ball_kd = 0.035f, .ball_ki = 0.015f,
        .servo_center_deg = 90.0f, .servo_limit_deg = 10.0f,
        .motor_base = 55.0f, .motor_limit = 100.0f,
    };
}

void control_set_mode(control_state_t *s, uint8_t mode) { s->mode = mode; }

void control_start(control_state_t *s, uint32_t now_ms) {
    s->running = true; s->phase = 0; s->start_ms = s->phase_ms = now_ms;
    s->stable_ms = 0; s->line_i = s->ball_i = 0; s->line_prev = 0;
}

control_output_t control_tick(control_state_t *s, uint32_t now_ms, float dt,
                              float line_error, bool start_line_seen) {
    control_output_t o = {0};
    o.elapsed_s = (now_ms - s->start_ms) * 0.001f;
    o.vision_lost = !s->vision.valid || now_ms - s->vision.received_ms > 250;
    if (!s->running || o.vision_lost) {
        o.servo_deg = s->gains.servo_center_deg; o.stop = true; return o;
    }

    /* MaixCAM sends target-position in cm and ball velocity in cm/s. */
    const float ball_error = (float)s->vision.delta_x * 10.0f;
    const float ball_velocity = (float)s->vision.delta_y * 10.0f;
    s->ball_i = clampf(s->ball_i + ball_error * dt, -80.0f, 80.0f);
    const float tilt = s->gains.ball_kp * ball_error
                     - s->gains.ball_kd * ball_velocity
                     + s->gains.ball_ki * s->ball_i;
    o.servo_deg = s->gains.servo_center_deg
                + clampf(tilt, -s->gains.servo_limit_deg, s->gains.servo_limit_deg);

    /* Car line loop. line_error is normalized to roughly -1..+1 by the IR array. */
    if (s->mode >= 2) {
        s->line_i = clampf(s->line_i + line_error * dt, -2.0f, 2.0f);
        const float d = dt > 0 ? (line_error - s->line_prev) / dt : 0;
        s->line_prev = line_error;
        const float steer = s->gains.line_kp * line_error + s->gains.line_ki * s->line_i
                          + s->gains.line_kd * d;
        o.motor_left = clampf(s->gains.motor_base - steer * 35.0f,
                              -s->gains.motor_limit, s->gains.motor_limit);
        o.motor_right = clampf(s->gains.motor_base + steer * 35.0f,
                               -s->gains.motor_limit, s->gains.motor_limit);
        /* Ignore A for the first second, then stop on the next transverse marker. */
        if (start_line_seen && now_ms - s->start_ms > 1000 && s->mode != 2) {
            s->running = false; o.motor_left = o.motor_right = 0; o.stop = true;
        }
    }
    return o;
}
