#include "vision_protocol.h"
#include <math.h>
#include <string.h>

_Static_assert(sizeof(double) == 8, "protocol requires 64-bit IEEE-754 double");

static double double_le(const uint8_t *p) {
    uint64_t bits = 0;
    for (size_t i = 0; i < 8; ++i) bits |= (uint64_t)p[i] << (8U * i);
    double value = 0.0;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void vision_parser_reset(vision_parser_t *p) { p->size = 0; }

bool vision_parser_push(vision_parser_t *p, uint8_t byte, uint32_t now_ms,
                        vision_measurement_t *out) {
    if (p->size == 0 && byte != VISION_SOF0) return false;
    if (p->size == 1 && byte != VISION_SOF1) { p->size = byte == VISION_SOF0 ? 1 : 0; return false; }
    if (p->size == 2 && byte != VISION_DATA_SIZE) { p->size = byte == VISION_SOF0 ? 1 : 0; return false; }
    if (p->size >= sizeof(p->data)) { p->size = 0; return false; }
    p->data[p->size++] = byte;
    if (p->size < VISION_FRAME_SIZE) return false;

    uint8_t checksum = 0;
    for (size_t i = 3; i < 19; ++i) checksum = (uint8_t)(checksum + p->data[i]);
    const bool ok = checksum == p->data[19];
    if (ok) {
        out->delta_x = double_le(&p->data[3]);
        out->delta_y = double_le(&p->data[11]);
        out->valid = isfinite(out->delta_x) && isfinite(out->delta_y);
        out->received_ms = now_ms;
    }
    p->size = 0;
    return ok;
}
