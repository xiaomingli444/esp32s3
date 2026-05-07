#ifndef _WIFI_H_
#define _WIFI_H_

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"

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

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_RUN_MODE_NONE = 0,
    WIFI_RUN_MODE_AP,
    WIFI_RUN_MODE_STA,
} wifi_run_mode_t;

void register_wifi(QueueHandle_t frame_q);

bool wifi_stream_has_client(void);
void wifi_stream_update_frame_queue(QueueHandle_t frame_q);
bool wifi_stream_start_continuous(void);
void wifi_stream_stop_continuous(void);
bool wifi_stream_is_running(void);
void wifi_stream_request_single_shot(void);
bool wifi_stream_consume_single_shot(void);
bool wifi_stream_send_single_jpeg(const camera_fb_t *fb);
bool wifi_stream_send_single_rgb565(const camera_fb_t *fb);
bool wifi_stream_push_if_running(const camera_fb_t *fb);

/* Wi-Fi 模式控制与扫描 */
bool wifi_start_ap_mode(void);
bool wifi_connect_sta(const char *ssid, const char *password);
bool wifi_scan_networks(wifi_ap_record_t *records, uint16_t max_records, uint16_t *out_count);
wifi_run_mode_t wifi_get_run_mode(void);
bool wifi_upload_is_busy(void);

/* Wi-Fi 密码缓存（仅保存最近一次成功连接的 STA SSID/Password，用于 UI 自动填充） */
bool wifi_saved_password_get(const char *ssid, char *out, size_t out_len);

/* UI 只读查询接口：不触发连接/扫描/模式切换，仅返回当前已知的 STA/AP 信息 */
typedef struct {
    char sta_ssid[33];
    char sta_ip[20];
    char ap_ssid[33];
    char ap_ip[20];
    char ap_password[65];
} wifi_ui_info_t;

bool wifi_ui_get_info(wifi_ui_info_t *out);

typedef enum {
    WIFI_UPLOAD_DECISION_REJECT = 0,
    WIFI_UPLOAD_DECISION_ACCEPT = 1,
    WIFI_UPLOAD_DECISION_TIMEOUT = 2,
    WIFI_UPLOAD_DECISION_CANCEL = 3,
} wifi_upload_decision_t;

bool wifi_upload_user_decide(const char *request_id, wifi_upload_decision_t decision);

#ifdef __cplusplus
}
#endif

#endif
