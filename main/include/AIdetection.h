#pragma once
#include <stdint.h>
#include <vector>
#include <new>

// ESP-IDF
#include "esp_err.h"
#include "esp_log.h"

// esp-dl 头文件（按组件管理器默认 include 路径）
#include "dl_image.hpp"
#include "dl_image_define.hpp"
#include "dl_detect_base.hpp"
#include "dl_model_base.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_detect_espdet_postprocessor.hpp"

#include "esp_camera.h"
#include "lcd.h"

// FreeRTOS 队列句柄（用于注册检测任务）
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 单个检测框
typedef struct {
    int   cls;        // COCO 类别 id (0..79)
    float score;      // 置信度 0..1
    int   x1, y1;     // 左上角（原图坐标）
    int   x2, y2;     // 右下角（原图坐标）
} y11_box_t;


int y11_init_from_sd(const char* espdl_path, float conf_thresh, float iou_thresh);

int y11_detect_rgb565(const uint8_t* rgb565, int width, int height, int stride_bytes,
                      y11_box_t* out, int max_out);

/** @brief  释放模型与相关资源 */
void y11_deinit(void);

void y11_test(void);

// 注册 AI 检测任务：从 frame_in 取帧，推理，画框到旋转缩放后的 LCD，并把原始帧转发到 frame_out
// model_path: 例如 "/sdcard/models/demo.espdl"
void register_AIdetection(QueueHandle_t frame_in, QueueHandle_t frame_out, const char *model_path, float conf_thresh);
void unregister_AIdetection(void);



#ifdef __cplusplus
}
#endif


