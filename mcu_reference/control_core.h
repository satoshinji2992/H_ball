#pragma once
#include "vision_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ball_kp, ball_kd, ball_ki;
    float servo_center_deg, servo_limit_deg;
} control_gains_t;

typedef struct {
    control_gains_t gains;
    vision_measurement_t vision;
    bool enabled;
    float ball_i;
} control_state_t;

typedef struct {
    float servo_deg;
    bool active;
    bool vision_lost;
} control_output_t;

void control_init(control_state_t *s);
void control_enable(control_state_t *s, bool enabled);
control_output_t control_tick(control_state_t *s, uint32_t now_ms, float dt_s);
