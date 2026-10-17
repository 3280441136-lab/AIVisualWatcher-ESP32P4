#ifndef _RADAR_H_
#define _RADAR_H_

#include <stdbool.h>       //bool类型

#include "esp_err.h"       //esp_err_t返回值类型
#include "freertos/FreeRTOS.h" //FreeRTOS基础类型和宏
#include "freertos/task.h"     //TaskHandle_t任务句柄类型

esp_err_t radar_init(TaskHandle_t taskhandle);
bool radar_is_active(void);

#endif
