#include "app_fsm.h"       //应用状态机组件：启动状态机任务
#include "pir.h"           //PIR组件：初始化PIR GPIO并读取PIR状态
#include "radar.h"         //雷达组件：初始化雷达GPIO中断并通知状态机任务
#include "vision.h"        //视觉组件：初始化屏幕、摄像头和后续视觉检测入口

//ESP-IDF错误检查
#include "esp_err.h"       //ESP_ERROR_CHECK和esp_err_t错误码

//RTOS任务句柄
#include "freertos/task.h" //TaskHandle_t任务句柄类型

//main入口
void app_main(void)
{
    TaskHandle_t app_fsm_task_handle = NULL;

    ESP_ERROR_CHECK(pir_init());
    ESP_ERROR_CHECK(vision_init());
    ESP_ERROR_CHECK(app_fsm_start(&app_fsm_task_handle));
    ESP_ERROR_CHECK(radar_init(app_fsm_task_handle));
}
