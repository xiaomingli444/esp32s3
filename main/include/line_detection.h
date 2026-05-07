#pragma once

#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#define EPS 192

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 智能巡线任务：
// - 从 frame_in 读取 camera_fb_t* 帧（建议 RGB565/QVGA）
// - 基于 color_id 从 NVS 读取颜色阈值并在 HSV 空间做掩膜
// - 计算路径中心偏差（-1..+1）、路径角度（deg）、线宽百分比
// - 粗略识别交叉口类型（T/十字/无），并通过 uartp_post_line_follow 上报
// - 将原始帧转发到 frame_out（用于 LCD 预览）
void register_line_detection(QueueHandle_t frame_in,
                             QueueHandle_t frame_out,
                             uint8_t color_id);
void unregister_line_detection(void);

#ifdef __cplusplus
}
#endif
