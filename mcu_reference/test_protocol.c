#include "vision_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void put_double_le(uint8_t *p, double value) {
    uint64_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    for (size_t i = 0; i < 8; ++i) p[i] = (uint8_t)(bits >> (8U * i));
}

int main(void) {
    uint8_t f[VISION_FRAME_SIZE] = {0xaa, 0xbb, 0x10};
    put_double_le(&f[3], 5.25);
    put_double_le(&f[11], -3.5);
    for (size_t i = 3; i < 19; ++i) f[19] = (uint8_t)(f[19] + f[i]);

    vision_parser_t parser = {0}; vision_measurement_t m = {0}; bool done = false;
    for (size_t i = 0; i < sizeof(f); ++i) done |= vision_parser_push(&parser, f[i], 99, &m);
    assert(done && fabs(m.delta_x - 5.25) < 1e-12);
    assert(fabs(m.delta_y + 3.5) < 1e-12);
    assert(m.valid == 1 && m.received_ms == 99);

    f[19] ^= 1;
    vision_parser_reset(&parser);
    done = false;
    for (size_t i = 0; i < sizeof(f); ++i) done |= vision_parser_push(&parser, f[i], 100, &m);
    assert(!done);

    memset(f, 0, sizeof(f));
    f[0] = 0xaa; f[1] = 0xbb; f[2] = 0x10;
    put_double_le(&f[3], NAN);
    put_double_le(&f[11], NAN);
    for (size_t i = 3; i < 19; ++i) f[19] = (uint8_t)(f[19] + f[i]);
    vision_parser_reset(&parser);
    done = false;
    for (size_t i = 0; i < sizeof(f); ++i) done |= vision_parser_push(&parser, f[i], 101, &m);
    assert(done && !m.valid && m.received_ms == 101);

    puts("vision protocol test passed");
    return 0;
}
