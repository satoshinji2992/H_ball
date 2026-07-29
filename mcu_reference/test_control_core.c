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
    set_vision(&state, 20, 0, 10);

    /* Disabled control always holds the rod at its safe center. */
    control_output_t output = control_tick(&state, 10, 0.01f);
    assert(!output.active && output.servo_deg == state.gains.servo_center_deg);

    control_enable(&state, true);
    output = control_tick(&state, 20, 0.01f);
    assert(output.active);
    assert(output.servo_deg > state.gains.servo_center_deg);

    /* An invalid control dt must not change the integral. */
    const float integral_before = state.ball_i;
    (void)control_tick(&state, 30, 0.50f);
    assert(state.ball_i == integral_before);

    /* Vision timeout centers the rod and clears the ball integral. */
    output = control_tick(&state, 300, 0.01f);
    assert(!output.active && output.vision_lost);
    assert(output.servo_deg == state.gains.servo_center_deg);
    assert(state.ball_i == 0.0f);

    puts("control core test passed");
    return 0;
}
