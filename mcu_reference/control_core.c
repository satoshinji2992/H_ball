#include "control_core.h"
#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void control_init(control_state_t *s) {
    memset(s, 0, sizeof(*s));
    s->gains = (control_gains_t){
        .ball_kp = 0.18f, .ball_kd = 0.035f, .ball_ki = 0.015f,
        .servo_center_deg = 90.0f, .servo_limit_deg = 10.0f,
    };
}

void control_enable(control_state_t *s, bool enabled) {
    s->enabled = enabled;
    s->ball_i = 0.0f;
}

control_output_t control_tick(control_state_t *s, uint32_t now_ms, float dt) {
    control_output_t o = {0};
    o.vision_lost = !s->vision.valid || now_ms - s->vision.received_ms > 250;
    if (!s->enabled || o.vision_lost) {
        s->ball_i = 0.0f;
        o.servo_deg = s->gains.servo_center_deg;
        return o;
    }

    /* MaixCAM sends target-position error in mm and ball velocity in mm/s. */
    const float ball_error = (float)s->vision.ball_error_mm;
    const float ball_velocity = (float)s->vision.ball_velocity_mm_s;
    const bool dt_valid = dt > 0.0f && dt <= 0.05f;
    if (dt_valid) s->ball_i = clampf(s->ball_i + ball_error * dt, -80.0f, 80.0f);
    const float tilt = s->gains.ball_kp * ball_error
                     - s->gains.ball_kd * ball_velocity
                     + s->gains.ball_ki * s->ball_i;
    o.servo_deg = s->gains.servo_center_deg
                + clampf(tilt, -s->gains.servo_limit_deg, s->gains.servo_limit_deg);
    o.active = true;
    return o;
}
