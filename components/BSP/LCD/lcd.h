#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_spi.h"   
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <stdbool.h>

#include "Vernon_ST7789T.h"
#include "uartp.h"
#include "camera.h"
#include "lvgl_driver.h"


#ifdef __cplusplus
extern "C" {
#endif


// LCD SPI GPIO
// Using SPI2 
#define LCD_HOST  SPI3_HOST

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (16 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           40
#define EXAMPLE_PIN_NUM_MOSI           45
#define EXAMPLE_PIN_NUM_MISO           -1
#define EXAMPLE_PIN_NUM_LCD_DC         41
#define EXAMPLE_PIN_NUM_LCD_RST        39
#define EXAMPLE_PIN_NUM_LCD_CS         42
#define EXAMPLE_PIN_NUM_BK_LIGHT       48
#define EXAMPLE_PIN_NUM_TOUCH_CS       -1
// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              172
#define EXAMPLE_LCD_V_RES              320
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define Offset_X 34
#define Offset_Y 0

#define SRC_W 320
#define SRC_H 240
#define LCD_W 172
#define LCD_H 320

#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define BLACK   0x0000
#define WHITE   0xFFFF



#define LEDC_HS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO       EXAMPLE_PIN_NUM_BK_LIGHT
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_1
#define LEDC_TEST_DUTY         (4000)
#define LEDC_ResolutionRatio   LEDC_TIMER_13_BIT
#define LEDC_MAX_Duty          ((1 << LEDC_ResolutionRatio) - 1)


extern esp_lcd_panel_handle_t panel_handle;

void BK_Init(void);                             // Initialize the LCD backlight, which has been called in the LCD_Init function, ignore it                                                         
void BK_Light(uint8_t Light);                   // Call this function to adjust the brightness of the backlight. The value of the parameter Light ranges from 0 to 100
void lcd_init(void);                     // Call this function to initialize the screen (must be called in the main function) !!!!!

void register_lcd(const QueueHandle_t frame_in, 
    const QueueHandle_t frame_out, 
    uartp_task_t task);   

void lcd_test();

// 在屏幕上绘制十字的便捷 API
void lcd_draw_cross(int cx, int cy, int half_len, uint16_t color);

// 在旋转+等比缩放后显示的图像上，按原图坐标绘制检测框
// src_w, src_h: 原始图像尺寸（例如 320x240）
// (x1,y1)-(x2,y2): 原图坐标下的矩形框（左上-右下，闭区间）
void lcd_draw_bbox_from_src(int src_w, int src_h, int x1, int y1, int x2, int y2, uint16_t color);

// 向 LCD 任务投递一个“按原图坐标绘制矩形框”的叠加命令（非阻塞）
void lcd_overlay_post_bbox_from_src(int src_w, int src_h, int x1, int y1, int x2, int y2, uint16_t color);

// LCD 面板总线互斥，避免与 LVGL 刷新并发访问导致卡死
bool lcd_panel_lock(TickType_t timeout);
void lcd_panel_unlock(void);

// 获取/创建 LCD 叠加绘制队列（供其他任务消费或清空）
QueueHandle_t lcd_get_overlay_queue(void);

// 控制是否接受新的叠加绘制请求
void lcd_set_overlay_enabled(bool enabled);
bool lcd_is_overlay_enabled(void);

// 控制 LCD 显示任务（用于调度开启/挂起）
void lcd_task_suspend(void);
void lcd_task_resume(void);
bool lcd_task_is_created(void);
bool lcd_task_is_suspended(void);
void lcd_task_delete(void);

/* 最近 1 秒的 LCD 帧率（由 lcd 任务统计），用于 UI 显示 */
uint32_t lcd_get_fps(void);
/* 调整 LCD 任务刷屏间隔（毫秒）；缺省 33ms */
void lcd_set_frame_interval_ms(uint32_t interval_ms);


#ifdef __cplusplus
}
#endif



