#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdio.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sensor.h"
#include <esp_timer.h>

#if __cplusplus
extern "C" {
#endif

/*引脚宏定义*/
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D7 16
#define CAM_PIN_D6 17
#define CAM_PIN_D5 18
#define CAM_PIN_D4 12
#define CAM_PIN_D3 10
#define CAM_PIN_D2 8
#define CAM_PIN_D1 9
#define CAM_PIN_D0 11
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define XCLK_FREQ 20000000   // 提升像素时钟以提高采集 FPS（原 12MHz）



void camera_init(const pixformat_t pixel_fromat,
    const framesize_t frame_size,
    const uint8_t fb_count);

void register_camera(const pixformat_t pixel_fromat,
    const framesize_t frame_size,
    const uint8_t fb_count,
    const QueueHandle_t frame_out);

void camera_set_xclk(int xclk_hz);
void camera_reinit(camera_fb_t* frame);
void camera_stop(void);

bool camera_task_is_created(void);

#ifdef __cplusplus
}
#endif
#endif
