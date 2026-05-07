#ifndef __APRILTAG_RECOGNITION_H_
#define __APRILTAG_RECOGNITION_H_

#if __cplusplus
extern "C" {
#endif

#include "apriltag.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tag16h5.h"
#include "tag36h11.h"
#include "image_u8.h"
#include "math.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "uartp.h"

void register_apriltag_detection(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const tagformat_t tag);
void unregister_apriltag_detection(void);

#ifdef __cplusplus
}
#endif
#endif
