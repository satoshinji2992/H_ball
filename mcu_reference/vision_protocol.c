#include "vision_protocol.h"
#include <string.h>

static bool parse_field(const uint8_t *field, int16_t *value, bool *is_nan) {
    char text[5];
    size_t begin = 0;
    size_t end = 4;
    memcpy(text, field, 4);
    text[4] = '\0';

    while (begin < end && text[begin] == ' ') ++begin;
    while (end > begin && text[end - 1] == ' ') --end;
    if (end - begin == 3 && memcmp(&text[begin], "NaN", 3) == 0) {
        *value = 0;
        *is_nan = true;
        return true;
    }

    if (begin != 0 || end != 4 || (text[0] != '+' && text[0] != '-') ||
        text[1] < '0' || text[1] > '9' ||
        text[2] < '0' || text[2] > '9' ||
        text[3] < '0' || text[3] > '9') return false;

    int parsed = (text[1] - '0') * 100 + (text[2] - '0') * 10 + (text[3] - '0');
    if (text[0] == '-') parsed = -parsed;
    *value = (int16_t)parsed;
    *is_nan = false;
    return true;
}

void vision_parser_reset(vision_parser_t *p) { p->size = 0; }

bool vision_parser_push(vision_parser_t *p, uint8_t byte, uint32_t now_ms,
                        vision_measurement_t *out) {
    if (byte == VISION_SOF) {
        p->data[0] = byte;
        p->size = 1;
        return false;
    }
    if (p->size == 0) return false;
    p->data[p->size++] = byte;
    if (p->size < VISION_FRAME_SIZE) return false;

    const bool terminator_ok = p->data[9] == VISION_EOF0 && p->data[10] == VISION_EOF1;
    int16_t error = 0;
    int16_t velocity = 0;
    bool error_nan = false;
    bool velocity_nan = false;
    const bool fields_ok = terminator_ok &&
        parse_field(&p->data[1], &error, &error_nan) &&
        parse_field(&p->data[5], &velocity, &velocity_nan) &&
        error_nan == velocity_nan;
    p->size = 0;
    if (!fields_ok) return false;

    out->ball_error_mm = error;
    out->ball_velocity_mm_s = velocity;
    out->valid = error_nan ? 0U : 1U;
    out->received_ms = now_ms;
    return true;
}
