#include "radar.h"         //radar组件对外接口：radar_init()和radar_is_active()

//ESP-IDF错误检查
#include "esp_check.h"     //ESP_RETURN_ON_ERROR错误检查宏
#include "esp_err.h"       //esp_err_t和ESP_OK错误码

//GPIO输入和中断
#include "driver/gpio.h"   //GPIO配置、读电平、安装中断服务

//RTOS任务通知和ISR切换
#include "freertos/FreeRTOS.h" //pdTRUE、pdFALSE、BaseType_t等FreeRTOS基础类型和宏
#include "freertos/task.h"     //TaskHandle_t、vTaskNotifyGiveFromISR()、portYIELD_FROM_ISR()

#define RADAR_GPIO GPIO_NUM_15

#define TAG "radar"

static TaskHandle_t main_taskhandle;

bool radar_is_active(void)
{
    return gpio_get_level(RADAR_GPIO);
}

//雷达GPIO中断服务函数
static void IRAM_ATTR radar_gpio_isr_handler(void* arg)
{
    (void)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(main_taskhandle, &high_task_wakeup);
    if(high_task_wakeup == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

//雷达初始化
esp_err_t radar_init(TaskHandle_t taskhandle)
{
    main_taskhandle = taskhandle;

    gpio_config_t radar_gpio_conf = 
    {
        .pin_bit_mask = (1ULL << RADAR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    //配置雷达GPIO
    ESP_RETURN_ON_ERROR(gpio_config(&radar_gpio_conf),
                        TAG,
                        "radar gpio config failed");

    //安装GPIO中断
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0),
                        TAG,
                        "radar gpio install isr service failed");

    //添加雷达GPIO中断服务函数
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(RADAR_GPIO, radar_gpio_isr_handler, NULL),
                        TAG,
                        "radar gpio add isr handler failed");

    return ESP_OK;
}
