#include "app_fsm.h"       //app_fsm组件对外接口：app_fsm_start()
#include "pir.h"           //读取PIR当前是否触发
#include "radar.h"         //读取雷达当前是否保持触发
#include "vision.h"        //调用视觉检测入口

//ESP-IDF错误检查和日志
#include "esp_check.h"     //ESP_RETURN_ON_FALSE等错误检查宏
#include "esp_err.h"       //esp_err_t、ESP_OK、ESP_ERR_xxx错误码
#include "esp_log.h"       //ESP_LOGI日志输出

//RTOS任务和任务通知
#include "freertos/FreeRTOS.h" //pdTRUE、portMAX_DELAY、pdMS_TO_TICKS等FreeRTOS基础宏
#include "freertos/task.h"     //任务创建、任务通知、任务延时API

#define TAG "app_fsm"

#define APP_FSM_TASK_STACK_SIZE 2048
#define APP_FSM_TASK_PRIORITY 5
#define APP_FSM_TASK_CORE 0

typedef enum
{
    STATE_WAIT_RADAR,
    STATE_CHECK_PIR,
    STATE_COOLDOWN,
} app_fsm_state_t;

static void app_fsm_task(void *arg)
{
    (void)arg;

    app_fsm_state_t state = STATE_WAIT_RADAR;
    while (1)
    {
        switch (state)
        {
            case STATE_WAIT_RADAR:
            {
                ESP_ERROR_CHECK(vision_set_ai_enabled(true));
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                ESP_LOGI(TAG, "radar triggered,go to check PIR");
                state = STATE_CHECK_PIR;
                break;
            }

            case STATE_CHECK_PIR:
            {
                bool radar_active = radar_is_active();
                bool pir_active = pir_is_active();

                if (radar_active && pir_active)
                {
                    ESP_ERROR_CHECK(vision_set_ai_enabled(true));
                    ESP_LOGI(TAG, "radar and pir active,go to COOLDOWN");
                    state = STATE_COOLDOWN;
                }
                else if (radar_active)
                {
                    ESP_ERROR_CHECK(vision_set_ai_enabled(false));
                    ESP_LOGI(TAG, "radar active, pir inactive,go to COOLDOWN");
                    state = STATE_COOLDOWN;
                }
                else
                {
                    ESP_ERROR_CHECK(vision_set_ai_enabled(false));
                    ESP_LOGI(TAG, "radar inactive,go to WAIT");
                    state = STATE_WAIT_RADAR;
                }
                break;
            }

            case STATE_COOLDOWN:
            {
                vTaskDelay(pdMS_TO_TICKS(500));
                ulTaskNotifyTake(pdTRUE, 0);
                if (radar_is_active())
                {
                    ESP_LOGI(TAG, "radar active,go to check PIR");
                    state = STATE_CHECK_PIR;
                }
                else
                {
                    ESP_ERROR_CHECK(vision_set_ai_enabled(false));
                    ESP_LOGI(TAG, "radar inactive,go to WAIT");
                    state = STATE_WAIT_RADAR;
                }
                break;
            }
        }
    }
}

esp_err_t app_fsm_start(TaskHandle_t *out_task_handle)
{
    ESP_RETURN_ON_FALSE(out_task_handle != NULL,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "app fsm task handle output is NULL");

    BaseType_t task_created = xTaskCreatePinnedToCore(app_fsm_task,
                                                     "app_fsm_task",
                                                     APP_FSM_TASK_STACK_SIZE,
                                                     NULL,
                                                     APP_FSM_TASK_PRIORITY,
                                                     out_task_handle,
                                                     APP_FSM_TASK_CORE);
    ESP_RETURN_ON_FALSE(task_created == pdPASS,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "app fsm task create failed");

    return ESP_OK;
}
