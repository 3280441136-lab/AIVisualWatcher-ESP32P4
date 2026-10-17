#ifndef VISION_AI_INTERNAL_H
#define VISION_AI_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool valid;
    bool person_detected;
    int x;
    int y;
    int w;
    int h;
    uint8_t confidence;
    uint32_t frame_seq;
} vision_detection_result_t;

esp_err_t vision_ai_model_init(void);
esp_err_t vision_ai_detect_rgb888(const void *rgb888_data,
                                  int width,
                                  int height,
                                  uint32_t frame_seq,
                                  vision_detection_result_t *out_result);

#ifdef __cplusplus
}
#endif

#endif
