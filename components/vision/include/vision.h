#ifndef VISION_H
#define VISION_H

#include <stdbool.h>       //bool类型

#include "esp_err.h"       //esp_err_t返回值类型

esp_err_t vision_init(void);
esp_err_t vision_set_ai_enabled(bool enabled);

#endif
