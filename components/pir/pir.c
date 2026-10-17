#include "pir.h"           //pir组件对外接口：pir_init()和pir_is_active()

//ESP-IDF错误检查
#include "esp_check.h"     //ESP_RETURN_ON_ERROR错误检查宏
#include "esp_err.h"       //esp_err_t和ESP_OK错误码

//GPIO输入
#include "driver/gpio.h"   //GPIO配置和读取输入电平

#define TAG "pir"

#define PIR_GPIO GPIO_NUM_16

bool pir_is_active(void)
{
    return gpio_get_level(PIR_GPIO);
}

esp_err_t pir_init(void)
{
    gpio_config_t pir_gpio_conf =
    {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE  
    };

    ESP_RETURN_ON_ERROR(gpio_config(&pir_gpio_conf),
                        TAG,
                        "pir gpio config failed");
    
    return ESP_OK;
}
