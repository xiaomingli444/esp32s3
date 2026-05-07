#ifndef __FIND_COLOR_H__
#define __FIND_COLOR_H__

#undef EPS      // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sensor.h"
#include "esp_timer.h"
#include "key.h"
#include "lcd.h"
#include "lvgl_ui.h"
#include "uartp.h"

#define HALF_WIN 2      // 1=3x3, 2=5x5, 0=只取中心像素

typedef struct {
    int red;
    int green;
    int blue;
} rgb_values_t;

#if __cplusplus
extern "C" {
#endif

void register_find_color(const QueueHandle_t frame_in, const QueueHandle_t frame_out, const uint8_t color_id);
void unregister_find_color(void);
void find_color_request_learn(uint8_t color_id);

    
#ifdef __cplusplus
}
#endif
#endif
