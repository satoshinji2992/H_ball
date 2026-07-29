#include "control_core.h"
#include <assert.h>
#include <stdio.h>

static void set_vision(control_state_t *state, int16_t error_mm,
                       int16_t velocity_mm_s, uint32_t now_ms) {
    state->vision.ball_error_mm = error_mm;
    state->vision.ball_velocity_mm_s = velocity_mm_s;
    state->vision.valid = 1;
    state->vision.received_ms = now_ms;
}

int main(void) {
    control_state_t state;
    control_init(&state);
    control_start(&state, 0);
    set_vision(&state, 20, 0, 10);

    /* MCU idle mode is safe even if control_start was called accidentally. */
    control_output_t output = control_tick(&state, 10, 0.01f, 0.0f, false);
    assert(output.stop && output.servo_deg == state.gains.servo_center_deg);

    /* MCU ball-only mode controls the rod but never drives the car. */
    control_set_mode(&state, MCU_MODE_BALL_ONLY);
    output = control_tick(&state, 20, 0.01f, 0.0f, false);
    assert(!output.stop);
    assert(output.motor_left == 0.0f && output.motor_right == 0.0f);
    assert(output.servo_deg > state.gains.servo_center_deg);

    /* An invalid control dt must not change the integral. */
    const float integral_before = state.ball_i;
    (void)control_tick(&state, 30, 0.50f, 0.0f, false);
    assert(state.ball_i == integral_before);

    /* Vision timeout centers the rod and clears the ball integral. */
    output = control_tick(&state, 300, 0.01f, 0.0f, false);
    assert(output.stop && output.vision_lost);
    assert(output.servo_deg == state.gains.servo_center_deg);
    assert(state.ball_i == 0.0f);

    /* A/B stops its motors at the next marker but keeps ball control active. */
    control_set_mode(&state, MCU_MODE_DRIVE_AB);
    control_start(&state, 1000);
    set_vision(&state, 20, 0, 2101);
    output = control_tick(&state, 2101, 0.01f, 0.1f, true);
    assert(output.stop && state.vehicle_finished);
    assert(output.motor_left == 0.0f && output.motor_right == 0.0f);

    set_vision(&state, 20, 0, 2111);
    output = control_tick(&state, 2111, 0.01f, 0.1f, false);
    assert(output.stop && !output.vision_lost);
    assert(output.motor_left == 0.0f && output.motor_right == 0.0f);
    assert(output.servo_deg > state.gains.servo_center_deg);

    puts("control core test passed");
    return 0;
}
