#include "vision_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool feed(vision_parser_t *parser, const char *frame, uint32_t now_ms,
                 vision_measurement_t *measurement) {
    bool done = false;
    for (size_t i = 0; i < VISION_FRAME_SIZE; ++i)
        done |= vision_parser_push(parser, (uint8_t)frame[i], now_ms, measurement);
    return done;
}

int main(void) {
    const char valid[] = "[-110+045*]";
    const char invalid[] = "[NaN NaN *]";
    const char mixed[] = "[NaN +045*]";
    const char malformed[] = "[-110+04x*]";
    _Static_assert(sizeof(valid) - 1 == VISION_FRAME_SIZE, "valid frame size");
    _Static_assert(sizeof(invalid) - 1 == VISION_FRAME_SIZE, "invalid frame size");

    vision_parser_t parser = {0};
    vision_measurement_t m = {0};
    assert(feed(&parser, valid, 99, &m));
    assert(m.ball_error_mm == -110);
    assert(m.ball_velocity_mm_s == 45);
    assert(m.valid == 1 && m.received_ms == 99);

    vision_parser_reset(&parser);
    assert(feed(&parser, invalid, 100, &m));
    assert(!m.valid && m.received_ms == 100);

    vision_parser_reset(&parser);
    assert(!feed(&parser, mixed, 101, &m));
    assert(!feed(&parser, malformed, 102, &m));

    vision_parser_reset(&parser);
    (void)vision_parser_push(&parser, 'x', 0, &m);
    (void)vision_parser_push(&parser, '[', 0, &m);
    (void)vision_parser_push(&parser, '-', 0, &m);
    assert(feed(&parser, valid, 103, &m));
    assert(m.valid && m.received_ms == 103);

    puts("vision protocol test passed");
    return 0;
}
