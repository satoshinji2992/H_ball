#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_SOF '['
#define VISION_EOF0 '*'
#define VISION_EOF1 ']'
#define VISION_FRAME_SIZE 11

typedef struct {
    int16_t ball_error_mm;       /* target position - ball position */
    int16_t ball_velocity_mm_s;
    uint8_t valid;
    uint32_t received_ms;
} vision_measurement_t;

typedef struct {
    uint8_t data[VISION_FRAME_SIZE];
    size_t size;
} vision_parser_t;

void vision_parser_reset(vision_parser_t *parser);
bool vision_parser_push(vision_parser_t *parser, uint8_t byte, uint32_t now_ms,
                        vision_measurement_t *output);
