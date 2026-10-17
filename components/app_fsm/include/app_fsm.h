#ifndef APP_FSM_H
#define APP_FSM_H

#include "esp_err.h"       //esp_err_t返回值类型
#include "freertos/FreeRTOS.h" //FreeRTOS基础类型和宏
#include "freertos/task.h"     //TaskHandle_t任务句柄类型

esp_err_t app_fsm_start(TaskHandle_t *out_task_handle);

#endif
