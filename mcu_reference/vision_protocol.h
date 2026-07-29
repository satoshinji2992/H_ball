#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_SOF0 0xAA
#define VISION_SOF1 0xBB
#define VISION_DATA_SIZE 0x10
#define VISION_FRAME_SIZE 20

typedef struct {
    double delta_x; /* target-position, cm */
    double delta_y; /* ball velocity, cm/s */
    uint8_t valid;
    uint32_t received_ms;
} vision_measurement_t;

typedef struct {
    uint8_t data[64];
    size_t size;
} vision_parser_t;

void vision_parser_reset(vision_parser_t *parser);
bool vision_parser_push(vision_parser_t *parser, uint8_t byte, uint32_t now_ms,
                        vision_measurement_t *output);
