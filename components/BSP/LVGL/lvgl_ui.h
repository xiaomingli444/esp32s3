#pragma once

#include "lvgl.h"
#include "lvgl_driver.h"

#include <esp_log.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建主 UI（下拉框 + 控制按钮 + 3 个操作按钮） */
void Lvgl_Example1(void);

/* 读取并清除 Open/Close 按钮点击标志（供调度层轮询） */
bool lvgl_ui_consume_open_clicked(void);
bool lvgl_ui_consume_close_clicked(void);
bool lvgl_ui_consume_web_clicked(void);

typedef enum {
    LVGL_UI_TASK_NONE = 0,
    LVGL_UI_TASK_FIND_COLOR,
    LVGL_UI_TASK_COLOR_DETECT,
    LVGL_UI_TASK_APRILTAG,
    LVGL_UI_TASK_LINE_DETECT,
    LVGL_UI_TASK_AI_DETECT,
} lvgl_ui_task_t;

typedef enum {
    LVGL_UI_TAG_NONE = 0,
    LVGL_UI_TAG_16H5,
    LVGL_UI_TAG_36H11,
} lvgl_ui_tag_dict_t;

typedef enum {
    LVGL_UI_LED_NONE = 0,
    LVGL_UI_LED_ON,
    LVGL_UI_LED_OFF,
} lvgl_ui_led_mode_t;

typedef enum {
    LVGL_UI_FRAME_NONE = 0,
    LVGL_UI_FRAME_ON,
    LVGL_UI_FRAME_OFF,
} lvgl_ui_frame_mode_t;

typedef enum {
    LVGL_UI_STREAM_NONE = 0,
    LVGL_UI_STREAM_SINGLE,   // 单次发送当前帧
    LVGL_UI_STREAM_STREAM,   // 连续流
} lvgl_ui_stream_mode_t;

lvgl_ui_task_t lvgl_ui_get_selected_task(void);
uint8_t        lvgl_ui_get_selected_color_profile(void);
lvgl_ui_tag_dict_t lvgl_ui_get_selected_tag_dict(void);
lvgl_ui_led_mode_t   lvgl_ui_get_selected_led_mode(void);
lvgl_ui_frame_mode_t lvgl_ui_get_selected_frame_mode(void);
lvgl_ui_stream_mode_t lvgl_ui_get_selected_stream_mode(void);
bool lvgl_ui_get_selected_model(char *buf, size_t len);
bool lvgl_ui_get_selected_ai_score(float *out_score);
void lvgl_ui_show_hint(const char *text);
void lvgl_ui_show_hint_ms(const char *text, uint32_t duration_ms);
bool lvgl_ui_is_main_screen_active(void);
void lvgl_ui_set_selected_task(lvgl_ui_task_t task);
void lvgl_ui_set_selected_color_profile(uint8_t color_id);
void lvgl_ui_set_selected_tag_dict(lvgl_ui_tag_dict_t tag_dict);
void lvgl_ui_set_selected_ai_model(uint8_t model_index);
void lvgl_ui_set_selected_ai_score(uint8_t score_index);
void lvgl_ui_set_selected_led_mode(lvgl_ui_led_mode_t mode);
void lvgl_ui_set_selected_frame_mode(lvgl_ui_frame_mode_t mode);
void lvgl_ui_set_selected_msroi_enabled(bool enable);
void lvgl_ui_set_selected_msroi_modules(bool module1_enable, bool module2_enable, bool module3_enable);

/* Wi-Fi 配置界面状态更新 */
void lvgl_ui_wifi_set_status(const char *text);
void lvgl_ui_wifi_on_sta_connected(const char *ip);
void lvgl_ui_wifi_on_sta_disconnected(const char *reason);

/* HTTP 模型上传弹窗与进度 UI */
void lvgl_ui_upload_show_request(const char *request_id,
                                const char *filename,
                                uint32_t size_bytes,
                                const char *type,
                                const char *sha256);
void lvgl_ui_upload_set_waiting(const char *request_id);
void lvgl_ui_upload_update_progress(const char *request_id, uint32_t received, uint32_t total);
void lvgl_ui_upload_finish(const char *request_id, bool success, const char *message);
void lvgl_ui_refresh_model_list(void);

/* STA 自动更新弹窗/进度（由固件更新模块调用） */
void lvgl_ui_fw_update_show_prompt(const char *current_ver, const char *latest_ver);
void lvgl_ui_fw_update_set_progress(int percent, const char *message);
void lvgl_ui_fw_update_finish(bool success, const char *message);

/* 跨线程安全：投递回调到 LVGL 线程执行（回调中禁止长时间阻塞） */
bool lvgl_ui_async_call(void (*cb)(void *), void *user_data);

#ifdef __cplusplus
}
#endif
