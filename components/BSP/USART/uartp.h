#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

// ========== 协议常量 ==========
#define UARTP_STX              0xFE
#define UARTP_DEV_DEVICE       0x00   // 本设备（视觉模块）
#define UARTP_DEV_HOST         0xFF   // 上位机

// MSG（方向用 DEV 区分；各任务共用各自 ID）
#define UARTP_MSG_D1H          0xD1   // 颜色学习
#define UARTP_MSG_D2H          0xD2   // 颜色检测
#define UARTP_MSG_D3H          0xD3   // AprilTag
#define UARTP_MSG_D4H          0xD4   // 智能巡线（新增）
#define UARTP_MSG_D5H          0xD5   // AI检测
#define UARTP_MSG_D6H          0xD6   // frame overlay control
#define UARTP_MSG_D7H          0xD7   // fill light control
#define UARTP_MSG_D8H          0xD8   // stop current task
#define UARTP_MSG_D9H          0xD9   // empty task (camera preview)
#define UARTP_MSG_DAH          0xDA   // msroi/module switch control

// 颜色 ID 范围（Excel：1~7）
#define UARTP_COLOR_ID_MIN     1
#define UARTP_COLOR_ID_MAX     7

// ========== 当前任务枚举 ==========
typedef enum {
    UARTP_TASK_NONE = 0,
    UARTP_TASK_COLOR_LEARN,   // D1H
    UARTP_TASK_COLOR_DETECT,  // D2H
    UARTP_TASK_APRILTAG,      // D3H
    UARTP_TASK_LINE_FOLLOW,   // D4H
    UARTP_TASK_AI_DETECT,     // D5H
} uartp_task_t;

// ========== 上行（设备->上位机）结构 ==========
typedef struct { uint8_t R, G, B; } uartp_d1h_color_learn_rsp_t;

typedef struct {
    uint8_t valid;   // 0/1
    float   cx;      // -1~+1，左负右正
    float   cy;      // -1~+1，下负上正
    float   area_pct;// 0~100 %
} uartp_d2h_color_detect_rsp_t;

typedef struct {
    uint8_t valid;   // 0/1
    int16_t tag_id;  // NOTE: 文档写 int8 但范围到 586，这里用 int16_t 覆盖 0~586
    float   cx;
    float   cy;
    float   area_pct;
    float   yaw_deg; // 未识别用 -1
    float   dist_cm; // 未识别用 -1
} uartp_d3h_apriltag_rsp_t;

// —— 交叉口类型 —— 
typedef enum {
    UARTP_INTXN_NONE  = 0,  // 无交叉
    UARTP_INTXN_T     = 1,  // T 型
    UARTP_INTXN_CROSS = 2,  // 十字
} uartp_intersection_t;


// —— D4H 巡线状态上报 ——
// valid + offset_x + heading_err_deg + line_width_pct + intersect_type + intersect_conf
typedef struct {
    uint8_t               valid;            // 0/1
    float                 offset_x;         // -1..+1
    float                 heading_err_deg;  // -180..+180
    float                 line_width_pct;   // 0..100
    uint8_t               intersect_type;   // uartp_intersection_t
    float                 intersect_conf;   // 0..1
} uartp_d4h_line_follow_rsp_t;

// ---------- D5H AI检测上报 ----------
// valid + num_dets + N*(class_id + score + x1 + y1 + x2 + y2)
#define UARTP_D5H_MAX_DETS     19
typedef struct {
    uint8_t  class_id;  // 0..255
    float    score;     // 0..1
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
} uartp_d5h_det_t;

typedef struct {
    uint8_t         valid;     // 0/1
    uint8_t         num_dets;  // <= UARTP_D5H_MAX_DETS
    uartp_d5h_det_t dets[UARTP_D5H_MAX_DETS];
} uartp_d5h_ai_detect_rsp_t;

// ========== 回调（Host->Device 请求） ==========
// 收到请求就会切换当前任务，并触发相应回调（color_id 仅 D1H/D2H/D4H 会携带）
typedef void (*uartp_on_color_learn_req_cb)(uint8_t color_id);
typedef void (*uartp_on_color_detect_req_cb)(uint8_t color_id);
typedef void (*uartp_on_apriltag_req_cb)(void);
typedef void (*uartp_on_line_follow_req_cb)(uint8_t color_id);  // D4H
typedef void (*uartp_on_ai_detect_req_cb)(uint8_t model_id, uint8_t score_id); // D5H
typedef void (*uartp_on_frame_overlay_req_cb)(bool enable); // D6H
typedef void (*uartp_on_fill_light_req_cb)(bool enable); // D7H
typedef void (*uartp_on_stop_req_cb)(uint8_t reason); // D8H
typedef void (*uartp_on_empty_task_req_cb)(bool enable); // D9H
typedef void (*uartp_on_msroi_ctrl_req_cb)(bool msroi_enable, bool module1_enable, bool module2_enable, bool module3_enable); // DAH

void uartp_set_handlers(uartp_on_color_learn_req_cb d1_cb,
                        uartp_on_color_detect_req_cb d2_cb,
                        uartp_on_apriltag_req_cb    d3_cb,
                        uartp_on_line_follow_req_cb d4_cb,
                        uartp_on_ai_detect_req_cb   d5_cb,
                        uartp_on_frame_overlay_req_cb d6_cb,
                        uartp_on_fill_light_req_cb  d7_cb,
                        uartp_on_stop_req_cb        d8_cb,
                        uartp_on_empty_task_req_cb  d9_cb,
                        uartp_on_msroi_ctrl_req_cb  da_cb);

// ========== 启动/停止（含 UART 初始化） ==========
typedef struct {
    uart_port_t port;     // UART_NUM_0/1/2...
    int         tx_gpio;  // TXD GPIO
    int         rx_gpio;  // RXD GPIO
    int         baud;     // 115200
    int         rx_buf;   // >= 2048
    int         tx_buf;   // >= 2048
    int         rx_task_core; // -1 不指定
    int         tx_task_core; // -1 不指定
    int         rx_task_prio; // 默认 10
    int         tx_task_prio; // 默认 9
} uartp_config_t;

esp_err_t uartp_start(const uartp_config_t *cfg);
void      uartp_stop(void);

// ========== 视觉任务上报（Device->Host） ==========
bool uartp_post_color_learn(uint8_t R, uint8_t G, uint8_t B);
bool uartp_post_color_detect(bool valid, float cx, float cy, float area_pct);
bool uartp_post_apriltag(bool valid, int tag_id, float cx, float cy,
                         float area_pct, float yaw_deg, float dist_cm);
bool uartp_post_line_follow(bool valid, float offset_x, float heading_err_deg,
                            float line_width_pct, uint8_t intersect_type, float intersect_conf);
bool uartp_post_ai_detect(uint8_t valid, uint8_t num_dets, const uartp_d5h_det_t *dets);

// ========== 任务状态查询 ==========
typedef struct {
    uartp_task_t current_task;
    uint8_t      current_color_id; // 对于 D1H/D2H/D4H 有意义
    tagformat_t tag_dict; // 当前 AprilTag 字典（仅对 APRILTAG 任务有效）
} uartp_task_state_t;

void uartp_get_current_task(uartp_task_state_t *out_state);
void uartp_set_task_state(uartp_task_t task, uint8_t color_id, tagformat_t tag);

#ifdef __cplusplus
}
#endif
