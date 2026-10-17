#include "vision_ai_internal.h"

#include <algorithm>
#include <list>
#include <new>

#include "esp_check.h"
#include "esp_log.h"
#include "pedestrian_detect.hpp"

#define TAG "vision_ai"

/* First hardware validation guard: a pedestrian box that almost touches all
 * image edges is usually a bad input/preprocess symptom, not a useful result.
 */
#define VISION_AI_FULL_FRAME_EDGE_MARGIN_PERCENT 5
#define VISION_AI_FULL_FRAME_AREA_PERCENT 80

static PedestrianDetect *s_pedestrian_detect;

static uint8_t vision_ai_score_to_percent(float score)
{
    if (score <= 0.0f)
    {
        return 0;
    }
    if (score >= 1.0f)
    {
        return 100;
    }
    return (uint8_t)(score * 100.0f + 0.5f);
}

esp_err_t vision_ai_model_init(void)
{
    if (s_pedestrian_detect != nullptr)
    {
        return ESP_OK;
    }

    s_pedestrian_detect = new (std::nothrow) PedestrianDetect();
    ESP_RETURN_ON_FALSE(s_pedestrian_detect != nullptr,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "pedestrian model alloc failed");

    s_pedestrian_detect->set_score_thr(0.70f);
    ESP_LOGI(TAG, "pedestrian detect model initialized");
    return ESP_OK;
}

esp_err_t vision_ai_detect_rgb888(const void *rgb888_data,
                                  int width,
                                  int height,
                                  uint32_t frame_seq,
                                  vision_detection_result_t *out_result)
{
    ESP_RETURN_ON_FALSE(rgb888_data != nullptr,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "rgb888 input is NULL");
    ESP_RETURN_ON_FALSE(out_result != nullptr,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "detection result output is NULL");
    ESP_RETURN_ON_FALSE(width > 0 && width <= UINT16_MAX && height > 0 && height <= UINT16_MAX,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid image size: %dx%d",
                        width,
                        height);
    ESP_RETURN_ON_FALSE(s_pedestrian_detect != nullptr,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "pedestrian model is not initialized");

    *out_result = {};
    out_result->valid = true;
    out_result->frame_seq = frame_seq;

    dl::image::img_t image =
    {
        .data = const_cast<void *>(rgb888_data),
        .width = (uint16_t)width,
        .height = (uint16_t)height,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_BGR888,
    };

    auto &results = s_pedestrian_detect->run(image);
    if (results.empty())
    {
        return ESP_OK;
    }

    const dl::detect::result_t *best_result = nullptr;
    for (const auto &result : results)
    {
        if ((best_result == nullptr) || (result.score > best_result->score))
        {
            best_result = &result;
        }
    }

    if (best_result == nullptr)
    {
        return ESP_OK;
    }

    int x0 = std::max(0, best_result->box[0]);
    int y0 = std::max(0, best_result->box[1]);
    int x1 = std::min(width, best_result->box[2]);
    int y1 = std::min(height, best_result->box[3]);
    if ((x1 <= x0) || (y1 <= y0))
    {
        ESP_LOGW(TAG,
                 "ignore invalid box: score=%d raw=[%d,%d,%d,%d] input=%dx%d",
                 vision_ai_score_to_percent(best_result->score),
                 best_result->box[0],
                 best_result->box[1],
                 best_result->box[2],
                 best_result->box[3],
                 width,
                 height);
        return ESP_OK;
    }

    int box_w = x1 - x0;
    int box_h = y1 - y0;
    int edge_margin_x = width * VISION_AI_FULL_FRAME_EDGE_MARGIN_PERCENT / 100;
    int edge_margin_y = height * VISION_AI_FULL_FRAME_EDGE_MARGIN_PERCENT / 100;
    int64_t box_area = (int64_t)box_w * box_h;
    int64_t image_area = (int64_t)width * height;
    bool touches_all_edges = (x0 <= edge_margin_x) &&
                             (y0 <= edge_margin_y) &&
                             (x1 >= (width - edge_margin_x)) &&
                             (y1 >= (height - edge_margin_y));
    bool too_large = (box_area * 100) >= (image_area * VISION_AI_FULL_FRAME_AREA_PERCENT);
    if (touches_all_edges && too_large)
    {
        ESP_LOGW(TAG,
                 "ignore full-frame-like box: score=%d raw=[%d,%d,%d,%d] clipped=[%d,%d,%d,%d] results=%u",
                 vision_ai_score_to_percent(best_result->score),
                 best_result->box[0],
                 best_result->box[1],
                 best_result->box[2],
                 best_result->box[3],
                 x0,
                 y0,
                 x1,
                 y1,
                 (unsigned)results.size());
        return ESP_OK;
    }

    out_result->person_detected = true;
    out_result->x = x0;
    out_result->y = y0;
    out_result->w = box_w;
    out_result->h = box_h;
    out_result->confidence = vision_ai_score_to_percent(best_result->score);
    return ESP_OK;
}
