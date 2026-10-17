/* ==================== 头文件：公共接口与基础能力 ==================== */
#include "vision.h"              // 对外接口：vision_init()、vision_set_ai_enabled()
#include "vision_ai_internal.h"  // vision组件内部AI适配层：ESP-DL行人检测

#include <stdint.h>              // 固定宽度整数：uint8_t、uint16_t、uint32_t
#include <string.h>              // 字符串比较和内存填充：strcmp()、memset()

/* ==================== 头文件：ESP-IDF错误检查与日志 ==================== */
#include "esp_attr.h"            // IRAM_ATTR：让CSI回调函数放入IRAM
#include "esp_check.h"           // ESP_RETURN_ON_ERROR、ESP_RETURN_ON_FALSE错误检查宏
#include "esp_err.h"             // esp_err_t、ESP_OK、ESP_ERR_xxx错误码
#include "esp_log.h"             // ESP_LOGI、ESP_LOGW、ESP_LOGE日志输出
#include "esp_timer.h"           // esp_timer_get_time()：统计单次AI推理耗时

/* ==================== 头文件：MIPI DSI显示屏 ==================== */
#include "esp_ldo_regulator.h"   // ESP-IDF原生LDO：给MIPI物理通道上电
#include "esp_lcd_jd9365_10_1.h" // 微雪JD9365 10.1寸屏幕驱动：创建panel对象
#include "esp_lcd_mipi_dsi.h"    // ESP-IDF原生MIPI DSI LCD驱动：创建DSI bus、DBI IO、DPI panel
#include "esp_lcd_panel_ops.h"   // ESP-IDF原生panel操作：reset、init、display on

/* ==================== 头文件：OV5647摄像头、CSI与ISP ==================== */
#include "driver/i2c_master.h"   // I2C master：复用屏幕组件创建的I2C1总线
#include "esp_sccb_i2c.h"        // SCCB over I2C：把I2C总线包装成摄像头控制接口
#include "esp_sccb_intf.h"       // SCCB接口基础类型
#include "esp_cam_sensor.h"      // sensor设备、格式查询、格式设置和stream控制
#include "esp_cam_sensor_detect.h" // sensor自动探测函数表
#include "esp_cam_ctlr.h"        // 摄像头控制器通用API：回调、enable、start
#include "esp_cam_ctlr_csi.h"    // MIPI CSI控制器创建和配置API
#include "driver/isp.h"          // ISP处理器：RAW8输入转RGB888输出
#include "driver/ppa.h"          // PPA硬件：缩放、格式转换、填充矩形
#include "esp_cache.h"           // esp_cache_msync()：同步CPU cache和实际内存

/* ==================== 头文件：FreeRTOS任务 ==================== */
#include "freertos/FreeRTOS.h"   // FreeRTOS基础类型和pdMS_TO_TICKS()
#include "freertos/semphr.h"     // SemaphoreHandle_t：等待VSYNC和保护PPA SRM
#include "freertos/task.h"       // xTaskCreatePinnedToCore()、vTaskDelay()

#define TAG "vision"

/* ==================== 显示屏配置 ==================== */
#define VISION_LCD_RST_GPIO 27
#define VISION_MIPI_DSI_LANE_NUM 2
#define VISION_MIPI_DSI_PHY_LDO_CHAN 3
#define VISION_MIPI_DSI_PHY_LDO_VOLTAGE_MV 2500
#define VISION_LCD_BITS_PER_PIXEL 24
#define VISION_LCD_PIXEL_FORMAT LCD_COLOR_FMT_RGB888
#define VISION_LCD_H_RES 800
#define VISION_LCD_V_RES 1280
#define VISION_LCD_FB_COUNT 2

/* 调试开关：1只显示红绿蓝条纹，0进入摄像头链路。 */
#define VISION_LCD_COLOR_TEST 0

/* ==================== 摄像头与图像链路配置 ==================== */
#define VISION_CAMERA_SCCB_SCL_GPIO 8
#define VISION_CAMERA_SCCB_SDA_GPIO 7
#define VISION_SHARED_I2C_PORT I2C_NUM_1
#define VISION_CAMERA_SCCB_FREQ_HZ (100 * 1000)
#define VISION_CAMERA_H_RES 800
#define VISION_CAMERA_V_RES 800
#define VISION_CAMERA_LANE_BITRATE_MBPS 200
#define VISION_CAMERA_DATA_LANE_NUM 2
#define VISION_CAMERA_FORMAT_NAME "MIPI_2lane_24Minput_RAW8_800x800_50fps"
#define VISION_CAMERA_FB_COUNT 2
#define VISION_CAMERA_CSI_INPUT_COLOR CAM_CTLR_COLOR_RAW8
#define VISION_CAMERA_CSI_OUTPUT_COLOR CAM_CTLR_COLOR_RAW8
#define VISION_ISP_CLK_HZ (80 * 1000 * 1000)
#define VISION_ISP_INPUT_COLOR ISP_COLOR_RAW8
#define VISION_ISP_OUTPUT_COLOR ISP_COLOR_RGB888
#define VISION_CAMERA_BAYER_ORDER COLOR_RAW_ELEMENT_ORDER_GBRG
#define VISION_RGB888_BYTES_PER_PIXEL 3

/* ==================== AI任务配置 ==================== */
#define VISION_AI_TASK_STACK_SIZE 4096
#define VISION_AI_TASK_PRIORITY 4
#define VISION_AI_TASK_CORE 1
#define VISION_AI_IDLE_DELAY_MS 50
#define VISION_AI_WAIT_FRAME_DELAY_MS 10
#define VISION_AI_INTERVAL_MS 200

/* ==================== 显示合成任务配置 ==================== */
#define VISION_DISPLAY_TASK_STACK_SIZE 4096
#define VISION_DISPLAY_TASK_PRIORITY 3
#define VISION_DISPLAY_TASK_CORE 1
#define VISION_DISPLAY_INTERVAL_MS 50
#define VISION_LCD_DPI_CLOCK_MHZ 40

/* ==================== AI绘制配置 ==================== */
#define VISION_COLOR_RED   0xFF0000
#define VISION_COLOR_GREEN 0x00FF00
#define VISION_COLOR_BLUE  0x0000FF
#define VISION_COLOR_WHITE 0xFFFFFF
#define VISION_AI_BOX_COLOR VISION_COLOR_RED
#define VISION_AI_TEXT_COLOR VISION_COLOR_RED
#define VISION_AI_PANEL_BG_COLOR VISION_COLOR_WHITE
#define VISION_AI_BOX_LINE_WIDTH 4
#define VISION_AI_STATUS_Y VISION_CAMERA_V_RES
#define VISION_AI_STATUS_H 96
#define VISION_AI_TEXT_X 24
#define VISION_AI_TEXT_Y (VISION_AI_STATUS_Y + 24)
#define VISION_AI_TEXT_SCALE 5
#define VISION_AI_TEXT_CLEAR_X (VISION_AI_TEXT_X - 8)
#define VISION_AI_TEXT_CLEAR_Y (VISION_AI_TEXT_Y - 8)
#define VISION_AI_TEXT_CLEAR_W 140
#define VISION_AI_TEXT_CLEAR_H 48

/* ==================== 全局状态：显示屏句柄 ==================== */
static esp_ldo_channel_handle_t phy_ldo_handle;
static esp_lcd_dsi_bus_handle_t dsi_bus_handle;
static esp_lcd_panel_io_handle_t dbi_io_handle;
static esp_lcd_panel_handle_t panel_handle;

/* ==================== 全局状态：frame buffer ==================== */
static void *camera_capture_buffer_a;
static void *camera_capture_buffer_b;
static void *camera_next_write_buffer;
static void *camera_read_buffer;
static void *display_front_buffer;
static void *display_back_buffer;
static size_t camera_frame_buffer_size;
static size_t display_frame_buffer_size;

/* ==================== 全局状态：PPA和VSYNC ==================== */
static ppa_client_handle_t ppa_srm_handle;
static ppa_client_handle_t ppa_fill_handle;
static SemaphoreHandle_t ppa_srm_mutex;
static SemaphoreHandle_t display_vsync_sem;

/* ==================== 全局状态：摄像头、CSI与ISP句柄 ==================== */
static i2c_master_bus_handle_t camera_i2c_bus_handle;
static esp_cam_sensor_device_t *camera_sensor;
static esp_cam_ctlr_handle_t camera_controller_handle;  // P4的CSI控制器
static isp_proc_handle_t isp_processor_handle;          // P4的ISP处理器
static portMUX_TYPE camera_buffer_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t camera_capture_buffer_a_users;
static uint32_t camera_capture_buffer_b_users;
static volatile bool camera_frame_ready;                // CSI至少完成过一帧后置true
static volatile uint32_t camera_frame_seq;              // CSI每完成一帧递增一次，AI任务用它跳过旧帧

/* ==================== 全局状态：AI任务与结果 ==================== */
static TaskHandle_t ai_task_handle;
static TaskHandle_t display_task_handle;
static volatile bool ai_enabled;
static portMUX_TYPE detection_result_lock = portMUX_INITIALIZER_UNLOCKED;
static vision_detection_result_t last_detection_result;


/* ==================== CSI回调：提供下一帧接收缓冲区 ==================== */
static bool IRAM_ATTR vision_camera_get_new_trans(esp_cam_ctlr_handle_t handle,
                                                  esp_cam_ctlr_trans_t *trans,
                                                  void *user_data)
{
    (void)handle;
    (void)user_data;

    /* 步骤1：把当前写入buffer交给CSI控制器。
     * 这块buffer只给CSI/ISP写，显示任务和AI任务读取的是上一块已经完成的camera_read_buffer。
     */
    portENTER_CRITICAL_ISR(&camera_buffer_lock);
    void *candidate_buffer = camera_next_write_buffer;
    void *other_buffer = (candidate_buffer == camera_capture_buffer_a) ?
                         camera_capture_buffer_b :
                         camera_capture_buffer_a;
    bool candidate_in_use = ((candidate_buffer == camera_capture_buffer_a) && (camera_capture_buffer_a_users > 0)) ||
                            ((candidate_buffer == camera_capture_buffer_b) && (camera_capture_buffer_b_users > 0));
    bool other_in_use = ((other_buffer == camera_capture_buffer_a) && (camera_capture_buffer_a_users > 0)) ||
                        ((other_buffer == camera_capture_buffer_b) && (camera_capture_buffer_b_users > 0));

    if (candidate_in_use && !other_in_use)
    {
        candidate_buffer = other_buffer;
    }

    /* 不返回NULL让CSI走内部backup buffer。
     * 当前链路后面接ISP RGB888输出，而CSI内部backup buffer按CSI RAW8大小申请，
     * 走backup路径会导致半帧错位或上下半帧重复。
     * 极端情况下A/B都被读取时，宁愿短暂覆盖一块摄像头buffer，也不走backup路径破坏帧布局。
     */
    trans->buffer = candidate_buffer;
    trans->buflen = camera_frame_buffer_size;
    camera_next_write_buffer = (candidate_buffer == camera_capture_buffer_a) ?
                               camera_capture_buffer_b :
                               camera_capture_buffer_a;
    portEXIT_CRITICAL_ISR(&camera_buffer_lock);

    /* 步骤2：返回false表示不需要从中断里唤醒更高优先级任务。 */
    return false;
}

static void *vision_camera_acquire_read_buffer(uint32_t *frame_seq)
{
    void *buffer = NULL;

    portENTER_CRITICAL(&camera_buffer_lock);
    buffer = camera_read_buffer;
    if (buffer == camera_capture_buffer_a)
    {
        camera_capture_buffer_a_users++;
    }
    else if (buffer == camera_capture_buffer_b)
    {
        camera_capture_buffer_b_users++;
    }
    else
    {
        buffer = NULL;
    }

    if ((buffer != NULL) && (frame_seq != NULL))
    {
        *frame_seq = camera_frame_seq;
    }
    portEXIT_CRITICAL(&camera_buffer_lock);

    return buffer;
}

static void vision_camera_release_read_buffer(void *buffer)
{
    portENTER_CRITICAL(&camera_buffer_lock);
    if ((buffer == camera_capture_buffer_a) && (camera_capture_buffer_a_users > 0))
    {
        camera_capture_buffer_a_users--;
    }
    else if ((buffer == camera_capture_buffer_b) && (camera_capture_buffer_b_users > 0))
    {
        camera_capture_buffer_b_users--;
    }
    portEXIT_CRITICAL(&camera_buffer_lock);
}

/* ==================== CSI回调：一帧传输完成 ==================== */
static bool IRAM_ATTR vision_camera_trans_finished(esp_cam_ctlr_handle_t handle,
                                                   esp_cam_ctlr_trans_t *trans,
                                                   void *user_data)
{
    (void)handle;
    (void)user_data;

    /* 步骤1：刚完成的是A/B摄像头buffer时，才发布给显示任务和AI任务读取。 */
    if ((trans->buffer == camera_capture_buffer_a) || (trans->buffer == camera_capture_buffer_b))
    {
        portENTER_CRITICAL_ISR(&camera_buffer_lock);
        camera_read_buffer = trans->buffer;
        camera_frame_ready = true;
        camera_frame_seq++;
        portEXIT_CRITICAL_ISR(&camera_buffer_lock);
    }

    return false;
}

/* ==================== LCD回调：一帧刷新完成 ==================== */
static bool IRAM_ATTR vision_lcd_refresh_done(esp_lcd_panel_handle_t panel,
                                              esp_lcd_dpi_panel_event_data_t *edata,
                                              void *user_ctx)
{
    (void)panel;
    (void)edata;
    (void)user_ctx;

    BaseType_t task_woken = pdFALSE;
    if (display_vsync_sem != NULL)
    {
        xSemaphoreGiveFromISR(display_vsync_sem, &task_woken);
    }
    return task_woken == pdTRUE;
}

/* ==================== LCD叠加绘制：CPU只写指定显示buffer的小块文字 ==================== */
static void vision_draw_pixel(void *buffer, int x, int y, uint32_t color)
{
    if (x < 0 || x >= VISION_LCD_H_RES || y < 0 || y >= VISION_LCD_V_RES)
    {
        return;
    }

    uint8_t *pixel = (uint8_t *)buffer + ((y * VISION_LCD_H_RES + x) * VISION_RGB888_BYTES_PER_PIXEL);
    pixel[0] = (uint8_t)(color & 0xFF);
    pixel[1] = (uint8_t)((color >> 8) & 0xFF);
    pixel[2] = (uint8_t)((color >> 16) & 0xFF);
}

static void vision_fill_rect_cpu(void *buffer, int x, int y, int w, int h, uint32_t color)
{
    for (int row = 0; row < h; row++)
    {
        for (int col = 0; col < w; col++)
        {
            vision_draw_pixel(buffer, x + col, y + row, color);
        }
    }
}

/* ==================== LCD叠加绘制：极简数字字体 ==================== */
static uint8_t vision_digit_font(char ch, int row)
{
    static const uint8_t digit_font[10][5] =
    {
        {0x07, 0x05, 0x05, 0x05, 0x07}, // 0
        {0x02, 0x06, 0x02, 0x02, 0x07}, // 1
        {0x07, 0x01, 0x07, 0x04, 0x07}, // 2
        {0x07, 0x01, 0x07, 0x01, 0x07}, // 3
        {0x05, 0x05, 0x07, 0x01, 0x01}, // 4
        {0x07, 0x04, 0x07, 0x01, 0x07}, // 5
        {0x07, 0x04, 0x07, 0x05, 0x07}, // 6
        {0x07, 0x01, 0x01, 0x01, 0x01}, // 7
        {0x07, 0x05, 0x07, 0x05, 0x07}, // 8
        {0x07, 0x05, 0x07, 0x01, 0x07}, // 9
    };
    static const uint8_t percent_font[5] = {0x05, 0x01, 0x02, 0x04, 0x05};
    static const uint8_t dash_font[5] = {0x00, 0x00, 0x07, 0x00, 0x00};

    if (ch >= '0' && ch <= '9')
    {
        return digit_font[ch - '0'][row];
    }
    if (ch == '%')
    {
        return percent_font[row];
    }
    if (ch == '-')
    {
        return dash_font[row];
    }
    return 0x00;
}

static void vision_draw_char(void *buffer, int x, int y, char ch, int scale, uint32_t color)
{
    for (int row = 0; row < 5; row++)
    {
        uint8_t bits = vision_digit_font(ch, row);
        for (int col = 0; col < 3; col++)
        {
            if (bits & (1 << (2 - col)))
            {
                vision_fill_rect_cpu(buffer, x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

static void vision_draw_text(void *buffer, int x, int y, const char *text, int scale, uint32_t color)
{
    while (*text)
    {
        vision_draw_char(buffer, x, y, *text, scale, color);
        x += 4 * scale;
        text++;
    }
}

static void vision_draw_confidence(void *buffer, uint8_t confidence)
{
    char text[5];
    text[0] = '0' + confidence / 100;
    text[1] = '0' + (confidence / 10) % 10;
    text[2] = '0' + confidence % 10;
    text[3] = '%';
    text[4] = '\0';

    vision_draw_text(buffer,
                     VISION_AI_TEXT_X,
                     VISION_AI_TEXT_Y,
                     text,
                     VISION_AI_TEXT_SCALE,
                     VISION_AI_TEXT_COLOR);
}

/* ==================== PPA工具：注册硬件客户端和填充矩形 ==================== */
static esp_err_t vision_ppa_init(void)
{
    ppa_client_config_t srm_config =
    {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_RETURN_ON_ERROR(ppa_register_client(&srm_config, &ppa_srm_handle),
                        TAG,
                        "ppa srm client register failed");

    ppa_client_config_t fill_config =
    {
        .oper_type = PPA_OPERATION_FILL,
        .max_pending_trans_num = 1,
    };
    ESP_RETURN_ON_ERROR(ppa_register_client(&fill_config, &ppa_fill_handle),
                        TAG,
                        "ppa fill client register failed");

    ppa_srm_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(ppa_srm_mutex != NULL,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "ppa srm mutex create failed");
    return ESP_OK;
}

static esp_err_t vision_ppa_fill_rect(void *buffer, int x, int y, int w, int h, uint32_t color)
{
    if (w <= 0 || h <= 0)
    {
        return ESP_OK;
    }

    ppa_fill_oper_config_t fill_config =
    {
        .out.buffer = buffer,
        .out.buffer_size = display_frame_buffer_size,
        .out.pic_w = VISION_LCD_H_RES,
        .out.pic_h = VISION_LCD_V_RES,
        .out.block_offset_x = x,
        .out.block_offset_y = y,
        .out.fill_cm = PPA_FILL_COLOR_MODE_RGB888,
        .fill_block_w = w,
        .fill_block_h = h,
        .fill_argb_color =
        {
            .a = 0xFF,
            .r = (uint8_t)((color >> 16) & 0xFF),
            .g = (uint8_t)((color >> 8) & 0xFF),
            .b = (uint8_t)(color & 0xFF),
        },
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_fill(ppa_fill_handle, &fill_config);
}

/* ==================== AI结果显示：PPA画框，CPU只画少量数字 ==================== */
static esp_err_t vision_draw_detection_result(void *buffer, const vision_detection_result_t *result)
{
    if (!result || !result->valid || !result->person_detected)
    {
        return ESP_OK;
    }

    int x0 = result->x;
    int y0 = result->y;
    int x1 = result->x + result->w;
    int y1 = result->y + result->h;

    if (x0 < 0)
    {
        x0 = 0;
    }
    if (y0 < 0)
    {
        y0 = 0;
    }
    if (x1 > VISION_CAMERA_H_RES)
    {
        x1 = VISION_CAMERA_H_RES;
    }
    if (y1 > VISION_CAMERA_V_RES)
    {
        y1 = VISION_CAMERA_V_RES;
    }

    int w = x1 - x0;
    int h = y1 - y0;
    if (w <= 0 || h <= 0)
    {
        return ESP_OK;
    }

    /* 步骤1：PPA在back buffer上填四条红色矩形，组成检测框。 */
    ESP_RETURN_ON_ERROR(vision_ppa_fill_rect(buffer, x0, y0, w, VISION_AI_BOX_LINE_WIDTH, VISION_AI_BOX_COLOR),
                        TAG,
                        "detection top line fill failed");
    ESP_RETURN_ON_ERROR(vision_ppa_fill_rect(buffer, x0, y1 - VISION_AI_BOX_LINE_WIDTH, w, VISION_AI_BOX_LINE_WIDTH, VISION_AI_BOX_COLOR),
                        TAG,
                        "detection bottom line fill failed");
    ESP_RETURN_ON_ERROR(vision_ppa_fill_rect(buffer, x0, y0, VISION_AI_BOX_LINE_WIDTH, h, VISION_AI_BOX_COLOR),
                        TAG,
                        "detection left line fill failed");
    ESP_RETURN_ON_ERROR(vision_ppa_fill_rect(buffer, x1 - VISION_AI_BOX_LINE_WIDTH, y0, VISION_AI_BOX_LINE_WIDTH, h, VISION_AI_BOX_COLOR),
                        TAG,
                        "detection right line fill failed");

    /* 步骤2：置信度数字很小，暂时由CPU写到back buffer，后续可替换成PPA图层blend。 */
    vision_draw_confidence(buffer, result->confidence);

    return ESP_OK;
}

/* ==================== AI任务：启用后持续处理最新画面 ==================== */
static void vision_ai_task(void *arg)
{
    (void)arg;

    uint32_t last_processed_frame_seq = 0;
    uint32_t ai_loop_count = 0;

    while (1)
    {
        if (!ai_enabled)
        {
            vTaskDelay(pdMS_TO_TICKS(VISION_AI_IDLE_DELAY_MS));
            continue;
        }

        uint32_t current_frame_seq = camera_frame_seq;
        if (!camera_frame_ready || current_frame_seq == last_processed_frame_seq)
        {
            vTaskDelay(pdMS_TO_TICKS(VISION_AI_WAIT_FRAME_DELAY_MS));
            continue;
        }

        uint32_t snapshot_frame_seq = 0;
        void *camera_snapshot_buffer = vision_camera_acquire_read_buffer(&snapshot_frame_seq);
        if (camera_snapshot_buffer == NULL)
        {
            ESP_LOGE(TAG, "camera read buffer is NULL");
            vTaskDelay(pdMS_TO_TICKS(VISION_AI_INTERVAL_MS));
            continue;
        }

        esp_err_t ret = esp_cache_msync(camera_snapshot_buffer,
                                        camera_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        if (ret != ESP_OK)
        {
            vision_camera_release_read_buffer(camera_snapshot_buffer);
            ESP_LOGE(TAG, "camera frame buffer cache sync for ai failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(VISION_AI_INTERVAL_MS));
            continue;
        }

        vision_detection_result_t result = {0};
        int64_t ai_start_us = esp_timer_get_time();
        ret = vision_ai_detect_rgb888(camera_snapshot_buffer,
                                      VISION_CAMERA_H_RES,
                                      VISION_CAMERA_V_RES,
                                      snapshot_frame_seq,
                                      &result);
        int64_t ai_elapsed_ms = (esp_timer_get_time() - ai_start_us) / 1000;
        vision_camera_release_read_buffer(camera_snapshot_buffer);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "pedestrian detect failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(VISION_AI_INTERVAL_MS));
            continue;
        }
        last_processed_frame_seq = snapshot_frame_seq;

        if (!ai_enabled)
        {
            continue;
        }

        portENTER_CRITICAL(&detection_result_lock);
        last_detection_result = result;
        portEXIT_CRITICAL(&detection_result_lock);

        if (!ai_enabled)
        {
            portENTER_CRITICAL(&detection_result_lock);
            memset(&last_detection_result, 0, sizeof(last_detection_result));
            portEXIT_CRITICAL(&detection_result_lock);
            continue;
        }

        ai_loop_count++;
        ESP_LOGI(TAG,
                 "ai loop[%lu]: input=%dx%d RGB888, elapsed=%lldms, detected=%d confidence=%u box=[%d,%d,%d,%d] frame=%lu",
                 (unsigned long)ai_loop_count,
                 VISION_CAMERA_H_RES,
                 VISION_CAMERA_V_RES,
                 (long long)ai_elapsed_ms,
                 result.person_detected,
                 result.confidence,
                 result.x,
                 result.y,
                 result.w,
                 result.h,
                 (unsigned long)result.frame_seq);
        vTaskDelay(pdMS_TO_TICKS(VISION_AI_INTERVAL_MS));
    }
}

/* ==================== 显示合成任务：摄像头画面 + 最近一次AI结果 ==================== */
static void vision_display_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_tick = xTaskGetTickCount();

    while (1)
    {
        if (!camera_frame_ready)
        {
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        void *camera_snapshot_buffer = vision_camera_acquire_read_buffer(NULL);
        if (camera_snapshot_buffer == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        if (xSemaphoreTake(ppa_srm_mutex, portMAX_DELAY) != pdTRUE)
        {
            vision_camera_release_read_buffer(camera_snapshot_buffer);
            ESP_LOGE(TAG, "ppa srm mutex take failed");
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        /* 步骤1：camera_snapshot_buffer是一帧已经完成的摄像头buffer，PPA读取前要同步cache状态。 */
        esp_err_t ret = esp_cache_msync(camera_snapshot_buffer,
                                        camera_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        if (ret != ESP_OK)
        {
            xSemaphoreGive(ppa_srm_mutex);
            vision_camera_release_read_buffer(camera_snapshot_buffer);
            ESP_LOGE(TAG, "camera frame buffer cache sync for display failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        /* 步骤2：PPA把摄像头buffer复制到back buffer上方800x800区域。
         * LCD当前读的是front buffer，所以back buffer在合成期间不会被LCD扫描到。
         */
        ppa_srm_oper_config_t camera_to_display_config =
        {
            .in.buffer = camera_snapshot_buffer,
            .in.pic_w = VISION_CAMERA_H_RES,
            .in.pic_h = VISION_CAMERA_V_RES,
            .in.block_w = VISION_CAMERA_H_RES,
            .in.block_h = VISION_CAMERA_V_RES,
            .in.block_offset_x = 0,
            .in.block_offset_y = 0,
            .in.srm_cm = PPA_SRM_COLOR_MODE_RGB888,
            .out.buffer = display_back_buffer,
            .out.buffer_size = display_frame_buffer_size,
            .out.pic_w = VISION_LCD_H_RES,
            .out.pic_h = VISION_LCD_V_RES,
            .out.block_offset_x = 0,
            .out.block_offset_y = 0,
            .out.srm_cm = PPA_SRM_COLOR_MODE_RGB888,
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x = 1.0f,
            .scale_y = 1.0f,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };

        ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &camera_to_display_config);
        xSemaphoreGive(ppa_srm_mutex);
        vision_camera_release_read_buffer(camera_snapshot_buffer);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "camera to display ppa srm failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        /* 步骤3：PPA只清空置信度文字所在的小矩形。
         * 红框区域会被步骤2的新摄像头画面覆盖，不需要额外清除整块状态区。
         */
        ret = vision_ppa_fill_rect(display_back_buffer,
                                   VISION_AI_TEXT_CLEAR_X,
                                   VISION_AI_TEXT_CLEAR_Y,
                                   VISION_AI_TEXT_CLEAR_W,
                                   VISION_AI_TEXT_CLEAR_H,
                                   VISION_AI_PANEL_BG_COLOR);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "display text area fill failed: %s", esp_err_to_name(ret));
        }

        /* 步骤4：读取最近一次AI结果，叠加到当前正在合成的back buffer。 */
        vision_detection_result_t result;
        portENTER_CRITICAL(&detection_result_lock);
        result = last_detection_result;
        portEXIT_CRITICAL(&detection_result_lock);

        int switch_sync_y_start = VISION_LCD_V_RES - 1;
        int switch_sync_y_end = VISION_LCD_V_RES;
        if (ai_enabled && result.valid)
        {
            ret = vision_draw_detection_result(display_back_buffer, &result);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "detection result compose failed: %s", esp_err_to_name(ret));
            }
            else if (result.person_detected)
            {
                switch_sync_y_start = VISION_AI_TEXT_Y;
                switch_sync_y_end = VISION_AI_TEXT_Y + 5 * VISION_AI_TEXT_SCALE;
            }
        }

        /* 步骤5：把back buffer交给DPI panel。
         * 这个buffer来自DPI驱动内部frame buffer，所以本机IDF驱动会切换当前fb索引。
         * y范围只用于驱动做cache写回：有文字时写回文字所在行，没有文字时只写回一行。
         */
        ret = esp_lcd_panel_draw_bitmap(panel_handle,
                                        0,
                                        switch_sync_y_start,
                                        VISION_LCD_H_RES,
                                        switch_sync_y_end,
                                        display_back_buffer);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "display buffer switch failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
            continue;
        }

        void *old_front_buffer = display_front_buffer;
        display_front_buffer = display_back_buffer;
        display_back_buffer = old_front_buffer;

        while (xSemaphoreTake(display_vsync_sem, 0) == pdTRUE)
        {
        }

        /* 步骤6：等一次新的VSYNC，让LCD完成一次刷新后再开始合成下一帧back buffer。
         * 如果一直等不到VSYNC，就先不要复用旧front buffer，避免把撕裂重新引回来。
         */
        while (xSemaphoreTake(display_vsync_sem, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            ESP_LOGW(TAG, "display vsync wait timeout");
        }

        /* 步骤7：限制显示合成频率。
         * LCD仍然按自己的刷新率扫描front buffer，但PPA不再每次VSYNC都合成一整帧。
         */
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(VISION_DISPLAY_INTERVAL_MS));
    }
}

/* ==================== 摄像头sensor初始化：识别OV5647并启动输出 ==================== */
static esp_err_t vision_sensor_init(void)
{
    /* 步骤1：复用JD9365屏幕组件已经创建好的I2C1总线。
     * GPIO7/GPIO8在板子上同时连到DSI/CSI接口的控制通道，所以这里不再新建I2C总线。
     */
    ESP_RETURN_ON_ERROR(i2c_master_get_bus_handle(VISION_SHARED_I2C_PORT, &camera_i2c_bus_handle),
                        TAG,
                        "camera sccb i2c bus get failed");

    /* 步骤2：准备sensor通用配置。
     * reset、pwdn、xclk填-1，表示当前板级连接不由这些GPIO控制。
     */
    esp_cam_sensor_config_t camera_config =
    {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
    };

    /* 步骤3：遍历已编进工程的sensor探测函数。
     * Kconfig里启用的摄像头驱动会把自己的detect函数放进这个数组。
     */
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end;
         ++p)
    {
        /* 步骤3.1：把当前候选sensor地址包装成一个SCCB设备。 */
        sccb_i2c_config_t sccb_i2c_config =
        {
            .scl_speed_hz = VISION_CAMERA_SCCB_FREQ_HZ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };

        ESP_RETURN_ON_ERROR(sccb_new_i2c_io(camera_i2c_bus_handle,
                                            &sccb_i2c_config,
                                            &camera_config.sccb_handle),
                            TAG,
                            "camera sccb io create failed");

        /* 步骤3.2：调用候选驱动的detect函数，读取PID等寄存器确认sensor型号。 */
        camera_config.sensor_port = p->port;
        camera_sensor = p->detect(&camera_config);
        if (camera_sensor != NULL)
        {
            /* 步骤3.3：当前链路只支持MIPI CSI摄像头，排除DVP等其他接口。 */
            ESP_RETURN_ON_FALSE(p->port == ESP_CAM_SENSOR_MIPI_CSI,
                                ESP_ERR_INVALID_ARG,
                                TAG,
                                "detected camera sensor is not on mipi csi port");
            break;
        }

        /* 步骤3.4：当前候选地址/驱动没有探测到sensor，释放这次临时创建的SCCB设备。 */
        ESP_RETURN_ON_ERROR(esp_sccb_del_i2c_io(camera_config.sccb_handle),
                            TAG,
                            "camera sccb io delete failed");
        camera_config.sccb_handle = NULL;
    }

    ESP_RETURN_ON_FALSE(camera_sensor != NULL,
                        ESP_ERR_NOT_FOUND,
                        TAG,
                        "ov5647 camera sensor not detected");

    /* 步骤4：查询当前sensor驱动实际暴露出来的输出格式列表。 */
    esp_cam_sensor_format_array_t camera_format_array = {0};
    esp_cam_sensor_query_format(camera_sensor, &camera_format_array);

    /* 步骤5：从格式列表里选择本工程要用的RAW8 800x800模式。 */
    const esp_cam_sensor_format_t *selected_format = NULL;
    const esp_cam_sensor_format_t *format_array = camera_format_array.format_array;
    for (int i = 0; i < camera_format_array.count; i++)
    {
        ESP_LOGI(TAG, "camera format[%d]: %s", i, format_array[i].name);
        if (strcmp(format_array[i].name, VISION_CAMERA_FORMAT_NAME) == 0)
        {
            selected_format = &format_array[i];
        }
    }

    ESP_RETURN_ON_FALSE(selected_format != NULL,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "camera format not supported: %s",
                        VISION_CAMERA_FORMAT_NAME);

    /* 步骤6：把选中的格式写入sensor寄存器配置。 */
    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(camera_sensor, selected_format),
                        TAG,
                        "camera format set failed");

    /* 步骤7：打开sensor视频流，之后OV5647 sensor 开始通过MIPI CSI持续输出RAW8数据。 */
    int stream_enable = 1;
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(camera_sensor,
                                             ESP_CAM_SENSOR_IOC_S_STREAM,
                                             &stream_enable),
                        TAG,
                        "camera sensor stream start failed");

    ESP_LOGI(TAG, "camera sensor initialized, format=%s", selected_format->name);
    return ESP_OK;
}

/* ==================== 摄像头链路初始化：CSI接收RAW，ISP输出RGB888 ==================== */
static esp_err_t vision_camera_init(void)
{
    /* 步骤1：计算摄像头输出区域和整块屏幕各自需要的RGB888字节数。
     * 摄像头是800x800，屏幕是800x1280，所以摄像头当前只更新屏幕上方800行。
     */
    camera_frame_buffer_size = VISION_CAMERA_H_RES *
                               VISION_CAMERA_V_RES *
                               VISION_RGB888_BYTES_PER_PIXEL;
    display_frame_buffer_size = VISION_LCD_H_RES *
                                VISION_LCD_V_RES *
                                VISION_RGB888_BYTES_PER_PIXEL;

    /* 步骤2：初始化ESP-DL行人检测模型。
     * 当前先让模型内部完成预处理，vision.c只负责提供已经完成的RGB888摄像头帧。
     */
    ESP_RETURN_ON_ERROR(vision_ai_model_init(), TAG, "vision ai model init failed");

    /* 步骤3：初始化PPA硬件客户端。
     * SRM客户端负责缩放/格式转换，FILL客户端负责画状态区和检测框矩形。
     */
    ESP_RETURN_ON_ERROR(vision_ppa_init(), TAG, "vision ppa init failed");

    /* 步骤4：识别OV5647，选择RAW8 800x800格式，并打开sensor stream。 */
    ESP_RETURN_ON_ERROR(vision_sensor_init(), TAG, "vision sensor init failed");

    /* 步骤5：创建MIPI CSI控制器。
     * 这里CSI只负责接RAW8并继续输出RAW8，Bayer解码交给后面的ISP完成。
     */
    esp_cam_ctlr_csi_config_t csi_config =
    {
        .ctlr_id = 0,
        .h_res = VISION_CAMERA_H_RES,
        .v_res = VISION_CAMERA_V_RES,
        .lane_bit_rate_mbps = VISION_CAMERA_LANE_BITRATE_MBPS,
        .input_data_color_type = VISION_CAMERA_CSI_INPUT_COLOR,
        .output_data_color_type = VISION_CAMERA_CSI_OUTPUT_COLOR,
        .data_lane_num = VISION_CAMERA_DATA_LANE_NUM,
        .byte_swap_en = false,
        .queue_items = 1,
    };

    ESP_RETURN_ON_ERROR(esp_cam_new_csi_ctlr(&csi_config, &camera_controller_handle),
                        TAG,
                        "camera csi controller create failed");

    /* 步骤6：为CSI/ISP输出申请两块摄像头frame buffer。
     * CSI/ISP写一块，显示任务和AI任务读另一块已经完成的buffer，避免读写同一帧造成撕裂。
     */
    camera_capture_buffer_a = esp_cam_ctlr_alloc_buffer(camera_controller_handle,
                                                        camera_frame_buffer_size,
                                                        MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(camera_capture_buffer_a != NULL,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "camera capture buffer a alloc failed");

    camera_capture_buffer_b = esp_cam_ctlr_alloc_buffer(camera_controller_handle,
                                                        camera_frame_buffer_size,
                                                        MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(camera_capture_buffer_b != NULL,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "camera capture buffer b alloc failed");

    camera_next_write_buffer = camera_capture_buffer_a;
    camera_read_buffer = NULL;

    /* 步骤7：注册CSI回调，让驱动知道下一帧应该写入哪块buffer。 */
    esp_cam_ctlr_evt_cbs_t camera_callbacks =
    {
        .on_get_new_trans = vision_camera_get_new_trans,
        .on_trans_finished = vision_camera_trans_finished,
    };

    ESP_RETURN_ON_ERROR(esp_cam_ctlr_register_event_callbacks(camera_controller_handle,
                                                              &camera_callbacks,
                                                              NULL),
                        TAG,
                        "camera callbacks register failed");

    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(camera_controller_handle),
                        TAG,
                        "camera controller enable failed");

    /* 步骤8：创建ISP处理器。
     * 输入是OV5647的RAW8 Bayer图，输出是LCD能直接显示的RGB888。
     * Bayer顺序必须和sensor RAW像素排列一致，否则会出现明显串色。
     */
    esp_isp_processor_cfg_t isp_config =
    {
        .clk_hz = VISION_ISP_CLK_HZ,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = VISION_ISP_INPUT_COLOR,
        .output_data_color_type = VISION_ISP_OUTPUT_COLOR,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = VISION_CAMERA_H_RES,
        .v_res = VISION_CAMERA_V_RES,
        .bayer_order = VISION_CAMERA_BAYER_ORDER,
    };

    ESP_RETURN_ON_ERROR(esp_isp_new_processor(&isp_config, &isp_processor_handle),
                        TAG,
                        "isp processor create failed");
    ESP_RETURN_ON_ERROR(esp_isp_enable(isp_processor_handle),
                        TAG,
                        "isp processor enable failed");

    /* 步骤9：先把两块摄像头buffer和两块显示buffer都清成白色。
     * 摄像头buffer等待第一帧写入，两块显示buffer作为front/back初始画面。
     */
    memset(camera_capture_buffer_a, 0xFF, camera_frame_buffer_size);
    ESP_RETURN_ON_ERROR(esp_cache_msync(camera_capture_buffer_a,
                                        camera_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                        TAG,
                        "camera capture buffer a cache sync failed");
    memset(camera_capture_buffer_b, 0xFF, camera_frame_buffer_size);
    ESP_RETURN_ON_ERROR(esp_cache_msync(camera_capture_buffer_b,
                                        camera_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                        TAG,
                        "camera capture buffer b cache sync failed");
    memset(display_front_buffer, 0xFF, display_frame_buffer_size);
    ESP_RETURN_ON_ERROR(esp_cache_msync(display_front_buffer,
                                        display_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                        TAG,
                        "display front buffer cache sync failed");

    memset(display_back_buffer, 0xFF, display_frame_buffer_size);
    ESP_RETURN_ON_ERROR(esp_cache_msync(display_back_buffer,
                                        display_frame_buffer_size,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                        TAG,
                        "display back buffer cache sync failed");

    /* 步骤10：启动CSI控制器，开始接收来自sensor的MIPI图像流。 */
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_start(camera_controller_handle),
                        TAG,
                        "camera controller start failed");

    /* 步骤11：创建AI后台任务。
     * 任务创建后默认空转，只有vision_set_ai_enabled(true)后才会持续处理画面。
     */
    BaseType_t task_created = xTaskCreatePinnedToCore(vision_ai_task,
                                                     "vision_ai_task",
                                                     VISION_AI_TASK_STACK_SIZE,
                                                     NULL,
                                                     VISION_AI_TASK_PRIORITY,
                                                     &ai_task_handle,
                                                     VISION_AI_TASK_CORE);
    ESP_RETURN_ON_FALSE(task_created == pdPASS,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "vision ai task create failed");

    /* 步骤12：创建显示合成任务。
     * 它持续用PPA把camera_buffer和AI结果合成到display_back_buffer，再切给LCD显示。
     */
    task_created = xTaskCreatePinnedToCore(vision_display_task,
                                           "vision_display_task",
                                           VISION_DISPLAY_TASK_STACK_SIZE,
                                           NULL,
                                           VISION_DISPLAY_TASK_PRIORITY,
                                           &display_task_handle,
                                           VISION_DISPLAY_TASK_CORE);
    ESP_RETURN_ON_FALSE(task_created == pdPASS,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "vision display task create failed");

    ESP_LOGI(TAG,
             "camera CSI/ISP pipeline started, camera buffer a=%p, b=%p, display front=%p, back=%p",
             camera_capture_buffer_a,
             camera_capture_buffer_b,
             display_front_buffer,
             display_back_buffer);
    return ESP_OK;
}

/* ==================== 显示屏初始化：创建DSI屏并取得frame buffer ==================== */
static esp_err_t vision_display_init(void)
{
    /* 步骤1：申请P4内部MIPI PHY LDO，给DSI物理层供电。 */
    esp_ldo_channel_config_t ldo_mipi_phy_config =
    {
        .chan_id = VISION_MIPI_DSI_PHY_LDO_CHAN,
        .voltage_mv = VISION_MIPI_DSI_PHY_LDO_VOLTAGE_MV,
    };

    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &phy_ldo_handle),
                        TAG,
                        "mipi dsi phy ldo acquire failed");

    /* 步骤2：创建MIPI DSI bus对象，对应P4到屏幕的高速数据通道。 */
    esp_lcd_dsi_bus_config_t bus_config = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &dsi_bus_handle),
                        TAG,
                        "mipi dsi bus create failed");

    /* 步骤3：创建DBI IO对象，用来给JD9365发送初始化命令。 */
    esp_lcd_dbi_io_config_t dbi_config = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(dsi_bus_handle, &dbi_config, &dbi_io_handle),
                        TAG,
                        "mipi dbi io create failed");

    /* 步骤4：配置DPI像素流。
     * JD9365组件默认给出80MHz像素时钟；这里降到60MHz，减轻LCD/DSI持续读取PSRAM的压力。
     */
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(VISION_LCD_PIXEL_FORMAT);
    dpi_config.dpi_clock_freq_mhz = VISION_LCD_DPI_CLOCK_MHZ;
    dpi_config.num_fbs = VISION_LCD_FB_COUNT;

    /* 步骤5：配置JD9365 vendor参数。
     * vendor_config把ESP-IDF通用LCD框架和JD9365专用驱动连接起来。
     */
    jd9365_vendor_config_t vendor_config =
    {
        .flags =
        {
            .use_mipi_interface = 1,
        },
        .mipi_config =
        {
            .dsi_bus = dsi_bus_handle,
            .dpi_config = &dpi_config,
            .lane_num = VISION_MIPI_DSI_LANE_NUM,
        },
    };

    /* 步骤6：配置panel设备参数。
     * rgb_ele_order和bits_per_pixel必须与frame buffer里写入的RGB888数据一致。
     */
    esp_lcd_panel_dev_config_t panel_config =
    {
        .reset_gpio_num = VISION_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = VISION_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    /* 步骤7：创建JD9365 panel对象。 */
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_jd9365(dbi_io_handle, &panel_config, &panel_handle),
                        TAG,
                        "jd9365 panel create failed");

    /* 步骤8：复位、初始化并点亮屏幕。 */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel_handle),
                        TAG,
                        "jd9365 panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel_handle),
                        TAG,
                        "jd9365 panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel_handle, true),
                        TAG,
                        "jd9365 display on failed");

    /* 步骤9：注册VSYNC回调。
     * 显示任务每次切到新的front buffer后，会等一次VSYNC再复用旧front作为新的back。
     */
    display_vsync_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(display_vsync_sem != NULL,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "display vsync semaphore create failed");

    esp_lcd_dpi_panel_event_callbacks_t dpi_callbacks =
    {
        .on_refresh_done = vision_lcd_refresh_done,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_register_event_callbacks(panel_handle,
                                                                   &dpi_callbacks,
                                                                   NULL),
                        TAG,
                        "display dpi callbacks register failed");

    /* 步骤10：取得DPI panel内部管理的两块显示frame buffer。
     * front buffer给LCD读，back buffer给PPA合成下一帧，VSYNC后两者交换角色。
     */
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(panel_handle,
                                                           VISION_LCD_FB_COUNT,
                                                           &display_front_buffer,
                                                           &display_back_buffer),
                        TAG,
                        "get display frame buffer failed");

    /* 步骤11：可选LCD纯色测试。
     * 这个测试绕过摄像头，只验证屏幕RGB888颜色、字节序和坐标方向。
     */
    if (VISION_LCD_COLOR_TEST)
    {
        for (int y = 0; y < VISION_LCD_V_RES; y++)
        {
            for (int x = 0; x < VISION_LCD_H_RES; x++)
            {
                if (x < VISION_LCD_H_RES / 3)
                {
                    vision_draw_pixel(display_front_buffer, x, y, VISION_COLOR_RED);
                }
                else if (x < VISION_LCD_H_RES * 2 / 3)
                {
                    vision_draw_pixel(display_front_buffer, x, y, VISION_COLOR_GREEN);
                }
                else
                {
                    vision_draw_pixel(display_front_buffer, x, y, VISION_COLOR_BLUE);
                }
            }
        }

        /* 步骤11.1：CPU写完PSRAM后同步cache，确保LCD/DMA读到最新颜色。 */
        ESP_RETURN_ON_ERROR(esp_cache_msync(display_front_buffer,
                                            VISION_LCD_H_RES * VISION_LCD_V_RES * VISION_RGB888_BYTES_PER_PIXEL,
                                            ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                            TAG,
                            "lcd color test cache sync failed");
        ESP_LOGI(TAG, "lcd color test: left=red, middle=green, right=blue");
    }

    ESP_LOGI(TAG,
             "jd9365 display initialized, front buffer=%p, back buffer=%p",
             display_front_buffer,
             display_back_buffer);
    return ESP_OK;
}

/* ==================== vision组件入口：先显示，后摄像头 ==================== */
esp_err_t vision_init(void)
{
    /* 步骤1：先初始化显示屏，因为显示合成任务需要使用DPI panel提供的front/back buffer。 */
    ESP_RETURN_ON_ERROR(vision_display_init(), TAG, "vision display init failed");

    /* 步骤2：如果只做LCD颜色测试，就不启动摄像头链路。 */
    if (VISION_LCD_COLOR_TEST)
    {
        return ESP_OK;
    }

    /* 步骤3：启动OV5647 -> CSI -> ISP -> camera buffer，再由PPA合成到LCD双buffer链路。 */
    ESP_RETURN_ON_ERROR(vision_camera_init(), TAG, "vision camera init failed");
    return ESP_OK;
}

/* ==================== 视觉检测开关：由状态机控制AI后台任务 ==================== */
esp_err_t vision_set_ai_enabled(bool enabled)
{
    if (ai_enabled == enabled)
    {
        return ESP_OK;
    }

    ai_enabled = enabled;

    if (!enabled)
    {
        portENTER_CRITICAL(&detection_result_lock);
        memset(&last_detection_result, 0, sizeof(last_detection_result));
        portEXIT_CRITICAL(&detection_result_lock);
    }

    ESP_LOGI(TAG, "ai detection %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}
