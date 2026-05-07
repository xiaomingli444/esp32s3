#pragma once
#include <stdint.h>
#include "driver/rmt_encoder.h"
#include "esp_check.h"
#include "driver/rmt_tx.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define WS2812_GPIO_NUM     GPIO_NUM_14
#define WS2812_LED_NUM      1

typedef struct {
    int red;
    int green;
    int blue;
} rgb_struct_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ws2812_strip_t *ws2812_strip_handle_t;


//函数声明
esp_err_t ws2812_init(gpio_num_t gpio,int maxled,ws2812_strip_handle_t* led_handle);
esp_err_t ws2812_deinit(ws2812_strip_handle_t handle);
esp_err_t ws2812_write(ws2812_strip_handle_t handle,uint32_t index,uint32_t r,uint32_t g,uint32_t b);
void register_ws2812(const QueueHandle_t RGBData);

#ifdef __cplusplus
}
#endif