#undef EPS      // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include <esp_log.h>
#include <esp_err.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <esp_task_wdt.h>
#include <iostream>
#include <map>

#include "sys.h"
#include "key.h"
#include "led.h"
#include "sd.h"
#include "wifi.h"
#include "uartp.h"
#include "lcd.h"
#include "camera.h"
#include "ws2812.h"
#include "lvgl_driver.h"
#include "lvgl_ui.h"
#include "device_identity.h"

#include "find_color.h"
#include "color_detection.h"
#include "line_detection.h"
#include "apriltag_recognition.h"
#include "AIdetection.h"
#include "TaskScheduling.h"

extern "C" void app_main(void);
static char TAG[]="main";

static const char *reset_reason_to_str(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_UNKNOWN:    return "UNKNOWN";
    case ESP_RST_POWERON:    return "POWERON";
    case ESP_RST_EXT:        return "EXT";
    case ESP_RST_SW:         return "SW";
    case ESP_RST_PANIC:      return "PANIC";
    case ESP_RST_INT_WDT:    return "INT_WDT";
    case ESP_RST_TASK_WDT:   return "TASK_WDT";
    case ESP_RST_WDT:        return "WDT";
    case ESP_RST_DEEPSLEEP:  return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:   return "BROWNOUT";
    case ESP_RST_SDIO:       return "SDIO";
    default:                 return "UNKNOWN";
    }
}

static void lvgl_loop_task(void *arg)
{
    (void)arg;

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


QueueHandle_t xQueueLCDFrame = NULL; //LCD 预览帧队列
QueueHandle_t xQueueColorLearnFrame = NULL; // 取色帧队列
QueueHandle_t xQueueColorDetectFrame = NULL; // 颜色检测帧队列
QueueHandle_t xQueueAprilTagFrame = NULL; // AprilTag 识别帧队列
QueueHandle_t xQueueLineDetectFrame = NULL; // 巡线检测帧队列
QueueHandle_t xQueueAIDetectFrame = NULL; // AI 识别帧队列
QueueHandle_t xQueueWiFiStreamFrame = NULL; // WiFi 图传帧队列


// 板级连接IO定义
#define UART_PORT    UART_NUM_1
#define UART_RX_GPIO 19
#define UART_TX_GPIO 20
#define UART_BAUD    115200


/* ========= app_main ========= */
extern "C" void app_main(void)
{
    (void)TAG;
    esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %d (%s)", (int)rr, reset_reason_to_str(rr));
    NVS_init();     // 初始化 NVS（用于颜色等参数存储）
    {
        esp_err_t err = device_identity_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "device identity init failed: %s", esp_err_to_name(err));
        }
    }
    key_init();     // 初始化按键
    sd_init();      // 初始化 SD 卡
    lcd_init();     // 初始化 LCD
    LVGL_Init();    // 初始化 LVGL
    Lvgl_Example1(); // 启动 LVGL UI
    TaskScheduling_Init(); // 启动按键驱动的摄像头/LCD 调度
    /* 单独创建 LVGL 任务，优先级高于 LCD 显示任务，避免预览占用 CPU 影响按键响应 */
    xTaskCreatePinnedToCore(lvgl_loop_task, "lvgl_loop", 4096, nullptr, 4, nullptr, 0);

    /* 主任务不再轮询 LVGL，直接挂起 */
    vTaskDelete(NULL);
}
