#undef EPS      // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#define EPS 192

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "uartp.h"
#include "camera.h"
#include "sys.h"



#ifdef __cplusplus
extern "C" {
#endif

// 注册颜色检测任务：从 xQueueFrameIn 取摄像头帧，
// 使用 OpenCV 按 NVS 存储的颜色 ID 阈值检测，
// 然后把原始帧指针转发到 xQueueFrameOut（LCD 显示）。
void register_color_detection(QueueHandle_t frame_in, QueueHandle_t frame_out, uint8_t color_id);
void unregister_color_detection(void);

#ifdef __cplusplus
}
#endif









