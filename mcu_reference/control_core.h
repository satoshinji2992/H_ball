#pragma once
#include "vision_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float line_kp, line_ki, line_kd;
    float ball_kp, ball_kd, ball_ki;
    float servo_center_deg, servo_limit_deg;
    float motor_base, motor_limit;
} control_gains_t;

/* Selected locally by the MCU. These modes are never received from MaixCAM. */
typedef enum {
    MCU_MODE_IDLE = 0,
    MCU_MODE_BALL_ONLY = 1,
    MCU_MODE_DRIVE_AB = 2,
    MCU_MODE_DRIVE_LAP = 3,
} mcu_run_mode_t;

typedef struct {
    control_gains_t gains;
    vision_measurement_t vision;
    bool running;
    bool vehicle_finished;
    mcu_run_mode_t mode;
    uint8_t phase;
    uint32_t start_ms, phase_ms, stable_ms;
    float line_i, line_prev, ball_i;
} control_state_t;

typedef struct {
    float motor_left;
    float motor_right;
    float servo_deg;
    float elapsed_s;
    bool stop;
    bool vision_lost;
} control_output_t;

void control_init(control_state_t *s);
void control_set_mode(control_state_t *s, mcu_run_mode_t mode);
void control_start(control_state_t *s, uint32_t now_ms);
control_output_t control_tick(control_state_t *s, uint32_t now_ms, float dt_s,
                              float line_error, bool start_line_seen);
