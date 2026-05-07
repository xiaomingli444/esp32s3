#include "wifi.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "camera.h"
#include "cJSON.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "device_identity.h"
#include "fw_update.h"
#include "lvgl_ui.h"
#include "mbedtls/sha256.h"
#include "sd.h"
#include "esp_vfs_fat.h"
#include "TaskScheduling.h"

static const char *TAG = "WIFI_CAM";

#define EXAMPLE_WIFI_AP_SSID      "KPUAV"
#define EXAMPLE_WIFI_AP_PASS      "12345678"
#define EXAMPLE_WIFI_AP_CHANNEL   1
#define EXAMPLE_WIFI_AP_MAX_CONN  4

static httpd_handle_t s_httpd = NULL;
static int s_ws_client_fd = -1;
static volatile bool s_ws_connected = false;
static volatile bool s_streaming_on = false;
static volatile bool s_single_shot_pending = false;
static volatile bool s_ws_job_pending = false;
static bool s_wifi_started = false;
static wifi_run_mode_t s_run_mode = WIFI_RUN_MODE_NONE;
static bool s_sta_connecting = false;
static bool s_sta_connected = false;
static esp_netif_t *s_netif_ap = NULL;
static esp_netif_t *s_netif_sta = NULL;
static bool s_event_handler_registered = false;
static QueueHandle_t xQueueFrameIn = NULL; /* 兼容字段，当前仅用于记录 */

/* Wi-Fi 操作互斥锁：
 * - 开机后台自动启 AP（任务）
 * - 用户点击“扫描/连接/进入”（UI 线程）
 * 若并发调用 esp_wifi_stop/set_mode/start/scan，会造成不稳定甚至崩溃。
 */
static portMUX_TYPE s_wifi_api_init_mux = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_wifi_api_mu_buf;
static SemaphoreHandle_t s_wifi_api_mu = NULL;

static void wifi_api_lock_init(void)
{
    if (s_wifi_api_mu) return;
    portENTER_CRITICAL(&s_wifi_api_init_mux);
    if (!s_wifi_api_mu) {
        s_wifi_api_mu = xSemaphoreCreateMutexStatic(&s_wifi_api_mu_buf);
    }
    portEXIT_CRITICAL(&s_wifi_api_init_mux);
}

static void wifi_api_lock(void)
{
    wifi_api_lock_init();
    if (s_wifi_api_mu) {
        (void)xSemaphoreTake(s_wifi_api_mu, portMAX_DELAY);
    }
}

static void wifi_api_unlock(void)
{
    if (s_wifi_api_mu) {
        (void)xSemaphoreGive(s_wifi_api_mu);
    }
}

#define WIFI_CRED_NVS_NS       "wifi_cred"
#define WIFI_CRED_KEY_SSID     "ssid"
#define WIFI_CRED_KEY_PASSWORD "pass"

static void wifi_saved_password_store_current_sta(void)
{
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
        return;
    }

    char ssid[33] = {0};
    size_t ssid_len = strnlen((const char *)cfg.sta.ssid, sizeof(cfg.sta.ssid));
    if (ssid_len == 0) return;
    memcpy(ssid, cfg.sta.ssid, ssid_len);
    ssid[ssid_len] = '\0';

    char password[65] = {0};
    size_t pass_len = strnlen((const char *)cfg.sta.password, sizeof(cfg.sta.password));
    if (pass_len > 0) {
        memcpy(password, cfg.sta.password, pass_len);
        password[pass_len] = '\0';
    }

    nvs_handle_t h;
    if (nvs_open(WIFI_CRED_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_set_str(h, WIFI_CRED_KEY_SSID, ssid);
    (void)nvs_set_str(h, WIFI_CRED_KEY_PASSWORD, password);
    (void)nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "saved STA password for SSID=%s", ssid);
}

bool wifi_saved_password_get(const char *ssid, char *out, size_t out_len)
{
    if (!ssid || ssid[0] == '\0' || !out || out_len == 0) return false;
    out[0] = '\0';

    nvs_handle_t h;
    if (nvs_open(WIFI_CRED_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }

    char stored_ssid[33] = {0};
    size_t ssid_size = sizeof(stored_ssid);
    esp_err_t r = nvs_get_str(h, WIFI_CRED_KEY_SSID, stored_ssid, &ssid_size);
    if (r != ESP_OK) {
        nvs_close(h);
        return false;
    }

    if (strcmp(stored_ssid, ssid) != 0) {
        nvs_close(h);
        return false;
    }

    size_t pass_size = out_len;
    r = nvs_get_str(h, WIFI_CRED_KEY_PASSWORD, out, &pass_size);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        out[0] = '\0';
        nvs_close(h);
        return true;
    }
    if (r != ESP_OK) {
        out[0] = '\0';
        nvs_close(h);
        return false;
    }

    nvs_close(h);
    return true;
}

typedef enum {
    WIFI_UPLOAD_STATE_IDLE = 0,
    WIFI_UPLOAD_STATE_PENDING_CONFIRM,
    WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD,
    WIFI_UPLOAD_STATE_UPLOADING,
    WIFI_UPLOAD_STATE_SUCCESS,
    WIFI_UPLOAD_STATE_FAILED,
    WIFI_UPLOAD_STATE_REJECTED,
    WIFI_UPLOAD_STATE_EXPIRED,
} wifi_upload_state_t;

typedef struct {
    wifi_upload_state_t state;
    char request_id[40];
    char token[40];
    char orig_filename[260];
    char safe_filename[260];
    char type[16];
    char sha256_hex[80];
    bool has_sha256;
    uint32_t total_size;
    uint32_t received;
    char requester_ip[64];
    int64_t created_ts_us;
    int64_t accepted_ts_us;
    char reason[48];
    char part_path[384];
    char final_path[384];
} wifi_upload_session_t;

typedef struct {
    wifi_upload_decision_t decision;
    char request_id[40];
} wifi_upload_evt_t;

static SemaphoreHandle_t s_upload_mu = NULL;
static QueueHandle_t s_upload_evt_q = NULL;
static TaskHandle_t s_upload_task = NULL;
static esp_timer_handle_t s_upload_resume_timer = NULL;
static esp_timer_handle_t s_ota_restart_timer = NULL;
static wifi_upload_session_t s_upload = {0};
static volatile bool s_upload_abort_requested = false;

static void wifi_upload_task(void *arg);
static void wifi_upload_abort_and_cleanup(const char *reason);
static void wifi_upload_schedule_resume_delayed(uint32_t delay_ms);
static void wifi_upload_resume_timer_cb(void *arg);
static void ota_restart_timer_cb(void *arg);

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
static void httpd_stop_server(void);
static esp_err_t ota_upload_data_handler(httpd_req_t *req);

static const char *httpd_method_to_str(httpd_method_t m)
{
    switch (m) {
    case HTTP_GET:     return "GET";
    case HTTP_POST:    return "POST";
    case HTTP_PUT:     return "PUT";
    case HTTP_DELETE:  return "DELETE";
    case HTTP_HEAD:    return "HEAD";
    case HTTP_OPTIONS: return "OPTIONS";
    default:           return "OTHER";
    }
}

static inline bool ws_ready(void)
{
    return s_httpd && s_ws_connected && s_ws_client_fd >= 0;
}

static inline void ws_mark_disconnected(void)
{
    s_ws_connected = false;
    s_streaming_on = false;
    s_ws_client_fd = -1;
    s_single_shot_pending = false;
    s_ws_job_pending = false;
}

typedef struct {
    uint8_t *data;
    size_t   len;
} ws_send_job_t;

static void ws_send_work(void *arg)
{
    ws_send_job_t *job = (ws_send_job_t *)arg;
    if (!job) {
        s_ws_job_pending = false;
        return;
    }

    if (ws_ready() && job->data && job->len > 0) {
        httpd_ws_frame_t pkt = {
            .payload = job->data,
            .len     = job->len,
            .type    = HTTPD_WS_TYPE_BINARY,
            .final   = true,
        };
        esp_err_t r = httpd_ws_send_frame_async(s_httpd, s_ws_client_fd, &pkt);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "ws send fail: %s(0x%x)", esp_err_to_name(r), r);
            ws_mark_disconnected();
        }
    }

    if (job->data) free(job->data);
    free(job);
    s_ws_job_pending = false;
}

/* 调度到 httpd 线程发送，避免在 LCD 任务里阻塞/触发 WDT */
static bool ws_send_binary_take(uint8_t *data, size_t len)
{
    if (!data || len == 0 || !ws_ready() || !s_httpd) {
        if (data) free(data);
        return false;
    }
    /* 只允许一个待发送帧，避免 STA 下堆积导致内存/看门狗问题 */
    if (s_ws_job_pending) {
        free(data);
        return false;
    }
    ws_send_job_t *job = (ws_send_job_t *)malloc(sizeof(ws_send_job_t));
    if (!job) {
        free(data);
        return false;
    }
    job->data = data;
    job->len  = len;
    s_ws_job_pending = true;
    esp_err_t r = httpd_queue_work(s_httpd, ws_send_work, job);
    if (r != ESP_OK) {
        s_ws_job_pending = false;
        free(data);
        free(job);
        return false;
    }
    return true;
}

// ===================== Model Upload =====================
static const uint32_t WIFI_UPLOAD_MAX_SIZE = 32 * 1024 * 1024;
static const uint32_t WIFI_UPLOAD_FREE_MARGIN = 128 * 1024;
static const uint32_t WIFI_UPLOAD_ACCEPT_WAIT_MS = 60 * 1000;
static const uint32_t WIFI_UPLOAD_PROGRESS_UI_MS = 120;
static const uint32_t WIFI_UPLOAD_IO_CHUNK = 4096;
static const uint32_t WIFI_UPLOAD_REQ_JSON_MAX = 512;

static inline int64_t now_us(void)
{
    return esp_timer_get_time();
}

static const char *upload_state_to_str(wifi_upload_state_t st)
{
    switch (st) {
    case WIFI_UPLOAD_STATE_PENDING_CONFIRM:      return "pending";
    case WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD: return "accepted";
    case WIFI_UPLOAD_STATE_UPLOADING:            return "uploading";
    case WIFI_UPLOAD_STATE_SUCCESS:              return "success";
    case WIFI_UPLOAD_STATE_FAILED:               return "failed";
    case WIFI_UPLOAD_STATE_REJECTED:             return "rejected";
    case WIFI_UPLOAD_STATE_EXPIRED:              return "expired";
    default:                                     return "idle";
    }
}

static bool str_eq_casei(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return false;
    }
    return *a == '\0' && *b == '\0';
}

static bool ends_with_casei(const char *s, const char *suffix)
{
    if (!s || !suffix) return false;
    size_t n = strlen(s);
    size_t m = strlen(suffix);
    if (m > n) return false;
    for (size_t i = 0; i < m; ++i) {
        char a = s[n - m + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

static bool is_hex64(const char *s)
{
    if (!s) return false;
    size_t n = strlen(s);
    if (n != 64) return false;
    for (size_t i = 0; i < n; ++i) {
        char c = s[i];
        bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!ok) return false;
    }
    return true;
}

static bool get_peer_ip(httpd_req_t *req, char *out, size_t out_len)
{
    if (!req || !out || out_len == 0) return false;
    out[0] = '\0';
    int sock = httpd_req_to_sockfd(req);
    struct sockaddr_storage addr;
    socklen_t alen = sizeof(addr);
    if (getpeername(sock, (struct sockaddr *)&addr, &alen) != 0) {
        return false;
    }
    if (addr.ss_family == AF_INET) {
        struct sockaddr_in *a = (struct sockaddr_in *)&addr;
        snprintf(out, out_len, "%s", inet_ntoa(a->sin_addr));
        return true;
    }
    return false;
}

static void random_hex(char *out, size_t out_len, size_t bytes)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (out_len < (bytes * 2 + 1)) return;

    uint8_t tmp[32];
    if (bytes > sizeof(tmp)) bytes = sizeof(tmp);
    esp_fill_random(tmp, bytes);

    static const char *hex = "0123456789abcdef";
    for (size_t i = 0; i < bytes; ++i) {
        out[i * 2 + 0] = hex[tmp[i] >> 4];
        out[i * 2 + 1] = hex[tmp[i] & 0x0F];
    }
    out[bytes * 2] = '\0';
}

static bool sanitize_filename_ascii(const char *in, char *out, size_t out_len, char *reason, size_t reason_len)
{
    if (reason && reason_len) reason[0] = '\0';
    if (!in || !out || out_len == 0) {
        if (reason && reason_len) snprintf(reason, reason_len, "invalid_filename");
        return false;
    }
    if (in[0] == '\0') {
        if (reason && reason_len) snprintf(reason, reason_len, "empty_filename");
        return false;
    }
    /* 兼容：允许传入路径（如 "build/00_basic.bin" 或 "C:\\fakepath\\00_basic.bin"），取 basename */
    const char *base = in;
    const char *slash = strrchr(base, '/');
    const char *bslash = strrchr(base, '\\');
    if (slash && bslash) {
        base = (slash > bslash) ? (slash + 1) : (bslash + 1);
    } else if (slash) {
        base = slash + 1;
    } else if (bslash) {
        base = bslash + 1;
    }

    if (!base || base[0] == '\0') {
        if (reason && reason_len) snprintf(reason, reason_len, "empty_filename");
        return false;
    }

    /* 拒绝路径穿越与目录分隔符（basename 里还出现这些，说明输入异常） */
    if (strstr(base, "/") || strstr(base, "\\") || strstr(base, "..")) {
        if (reason && reason_len) snprintf(reason, reason_len, "invalid_filename");
        return false;
    }

    size_t pos = 0;
    for (const unsigned char *p = (const unsigned char *)base; *p; ++p) {
        unsigned char c = *p;
        if (c < 0x20 || c == 0x7F) {
            if (reason && reason_len) snprintf(reason, reason_len, "invalid_filename");
            return false;
        }
        bool ok = (c >= '0' && c <= '9') ||
                  (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z') ||
                  c == '.' || c == '_' || c == '-' || c == ' ';
        if (!ok) {
            if (reason && reason_len) snprintf(reason, reason_len, "invalid_filename");
            return false;
        }
        if (pos + 1 >= out_len) {
            if (reason && reason_len) snprintf(reason, reason_len, "filename_too_long");
            return false;
        }
        out[pos++] = (char)c;
    }
    out[pos] = '\0';
    return true;
}

static void str_hex_preview_32(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!in) return;

    const size_t n = strnlen(in, 32);
    size_t pos = 0;
    for (size_t i = 0; i < n; ++i) {
        if (pos + 3 >= out_len) break;
        int w = snprintf(out + pos, out_len - pos, "%02X", (unsigned char)in[i]);
        if (w <= 0) break;
        pos += (size_t)w;
        if (i + 1 < n && pos + 1 < out_len) {
            out[pos++] = ' ';
            out[pos] = '\0';
        }
    }
}

static size_t utf8_decode_codepoint_z(const unsigned char *s, uint32_t *out_cp)
{
    if (!s || !out_cp) return 0;
    const unsigned char c0 = s[0];
    if (c0 == '\0') return 0;

    if (c0 < 0x80) {
        *out_cp = (uint32_t)c0;
        return 1;
    }

    if ((c0 & 0xE0) == 0xC0) {
        const unsigned char c1 = s[1];
        if (c1 == '\0' || (c1 & 0xC0) != 0x80) return 0;
        const uint32_t cp = ((uint32_t)(c0 & 0x1F) << 6) | (uint32_t)(c1 & 0x3F);
        if (cp < 0x80) return 0;
        *out_cp = cp;
        return 2;
    }

    if ((c0 & 0xF0) == 0xE0) {
        const unsigned char c1 = s[1];
        const unsigned char c2 = s[2];
        if (c1 == '\0' || c2 == '\0') return 0;
        if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80) return 0;
        const uint32_t cp = ((uint32_t)(c0 & 0x0F) << 12) |
                            ((uint32_t)(c1 & 0x3F) << 6) |
                            (uint32_t)(c2 & 0x3F);
        if (cp < 0x800) return 0;
        if (cp >= 0xD800 && cp <= 0xDFFF) return 0;
        *out_cp = cp;
        return 3;
    }

    if ((c0 & 0xF8) == 0xF0) {
        const unsigned char c1 = s[1];
        const unsigned char c2 = s[2];
        const unsigned char c3 = s[3];
        if (c1 == '\0' || c2 == '\0' || c3 == '\0') return 0;
        if ((c1 & 0xC0) != 0x80 || (c2 & 0xC0) != 0x80 || (c3 & 0xC0) != 0x80) return 0;
        const uint32_t cp = ((uint32_t)(c0 & 0x07) << 18) |
                            ((uint32_t)(c1 & 0x3F) << 12) |
                            ((uint32_t)(c2 & 0x3F) << 6) |
                            (uint32_t)(c3 & 0x3F);
        if (cp < 0x10000 || cp > 0x10FFFF) return 0;
        *out_cp = cp;
        return 4;
    }

    return 0;
}

static void copy_basename_for_display(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!in || in[0] == '\0') return;

    const char *base = in;
    const char *slash = strrchr(in, '/');
    const char *bslash = strrchr(in, '\\');
    if (slash && bslash) {
        base = (slash > bslash) ? (slash + 1) : (bslash + 1);
    } else if (slash) {
        base = slash + 1;
    } else if (bslash) {
        base = bslash + 1;
    }
    if (!base || base[0] == '\0') {
        base = in;
    }

    /* UI 展示用：
     * - 目标：不乱码、可读、与字体字库（0x20-0x7F）兼容。
     * - 策略：按 UTF-8 解码，尽量映射到 ASCII（含全角/常见连字符），其余替换为 '_'。
     */
    size_t pos = 0;
    const unsigned char *p_utf8 = (const unsigned char *)base;
    while (*p_utf8 && (pos + 1) < out_len) {
        uint32_t cp = 0;
        size_t n = utf8_decode_codepoint_z(p_utf8, &cp);
        if (n == 0) {
            out[pos++] = '_';
            p_utf8 += 1;
            continue;
        }

        char c = 0;
        if (cp >= 0x20 && cp <= 0x7E) {
            c = (char)cp;
        } else if (cp >= 0xFF10 && cp <= 0xFF19) {
            c = (char)('0' + (cp - 0xFF10));
        } else if (cp >= 0xFF21 && cp <= 0xFF3A) {
            c = (char)('A' + (cp - 0xFF21));
        } else if (cp >= 0xFF41 && cp <= 0xFF5A) {
            c = (char)('a' + (cp - 0xFF41));
        } else if (cp >= 0x0660 && cp <= 0x0669) {
            c = (char)('0' + (cp - 0x0660));
        } else if (cp >= 0x06F0 && cp <= 0x06F9) {
            c = (char)('0' + (cp - 0x06F0));
        } else if (cp == 0xFF0E || cp == 0x3002) {
            c = '.';
        } else if (cp == 0xFF3F) {
            c = '_';
        } else if (cp == 0xFF0D || cp == 0x2212 || cp == 0x2010 || cp == 0x2011 ||
                   cp == 0x2012 || cp == 0x2013 || cp == 0x2014) {
            c = '-';
        } else if (cp == 0x3000) {
            c = ' ';
        }

        if (!( (c >= '0' && c <= '9') ||
               (c >= 'A' && c <= 'Z') ||
               (c >= 'a' && c <= 'z') ||
               c == '.' || c == '_' || c == '-' || c == ' ')) {
            c = '_';
        }

        out[pos++] = c;
        p_utf8 += n;
    }
    out[pos] = '\0';

#if 0
    for (const unsigned char *p = (const unsigned char *)base; *p && (pos + 1) < out_len; ++p) {
        unsigned char c = *p;
        /* 控制字符替换为 '_'，避免 UI/日志异常 */
        if (c < 0x20 || c == 0x7F) {
            out[pos++] = '_';
            continue;
        }

        /* 兼容“全角”文件名：用户可能在 PC 上用中文输入法命名，看起来像 ASCII 实际是全角字母/数字 */
        if (c == 0xEF && p[1] && p[2]) {
            const unsigned char c1 = p[1];
            const unsigned char c2 = p[2];
            /* 全角数字 U+FF10..U+FF19 => EF BC 90..99 */
            if (c1 == 0xBC && c2 >= 0x90 && c2 <= 0x99) {
                out[pos++] = (char)('0' + (c2 - 0x90));
                p += 2;
                continue;
            }
            /* 全角大写 U+FF21..U+FF3A => EF BC A1..BA */
            if (c1 == 0xBC && c2 >= 0xA1 && c2 <= 0xBA) {
                out[pos++] = (char)('A' + (c2 - 0xA1));
                p += 2;
                continue;
            }
            /* 全角小写 U+FF41..U+FF5A => EF BD 81..9A */
            if (c1 == 0xBD && c2 >= 0x81 && c2 <= 0x9A) {
                out[pos++] = (char)('a' + (c2 - 0x81));
                p += 2;
                continue;
            }
            /* 常见标点：．＿－ */
            if (c1 == 0xBC && c2 == 0x8E) { /* U+FF0E */
                out[pos++] = '.';
                p += 2;
                continue;
            }
            if (c1 == 0xBC && c2 == 0xBF) { /* U+FF3F */
                out[pos++] = '_';
                p += 2;
                continue;
            }
            if (c1 == 0xBC && c2 == 0x8D) { /* U+FF0D */
                out[pos++] = '-';
                p += 2;
                continue;
            }
        } else if (c == 0xE3 && p[1] && p[2] && p[1] == 0x80 && p[2] == 0x80) {
            /* 全角空格 U+3000 => E3 80 80 */
            out[pos++] = ' ';
            p += 2;
            continue;
        }

        /* 非 ASCII（例如中文/Emoji/其它符号）：固件名只用于显示，统一降级为 '_'，避免 UI 乱码/缺字形导致“看不见” */
        if (c >= 0x80) {
            out[pos++] = '_';
            continue;
        }
        out[pos++] = (char)c;
    }
    out[pos] = '\0';
#endif

    if (out[0] == '\0') {
        snprintf(out, out_len, "%s", "firmware.bin");
    }
}

static bool sd_models_precheck(uint32_t required_bytes, char *reason, size_t reason_len)
{
    if (reason && reason_len) reason[0] = '\0';

    sd_init();

    struct stat st;
    if (stat("/sdcard", &st) != 0) {
        if (reason && reason_len) snprintf(reason, reason_len, "no_sd");
        return false;
    }

    mkdir("/sdcard/models", 0775);

    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;
    esp_err_t r = esp_vfs_fat_info("/sdcard", &total_bytes, &free_bytes);
    if (r != ESP_OK) {
        if (reason && reason_len) snprintf(reason, reason_len, "mount_fail");
        return false;
    }
    if (free_bytes < (uint64_t)required_bytes + WIFI_UPLOAD_FREE_MARGIN) {
        if (reason && reason_len) snprintf(reason, reason_len, "no_space");
        return false;
    }

    const char *test_path = "/sdcard/models/.wtest";
    FILE *f = fopen(test_path, "wb");
    if (!f) {
        if (reason && reason_len) snprintf(reason, reason_len, "read_only");
        return false;
    }
    uint8_t b = 0;
    size_t w = fwrite(&b, 1, 1, f);
    fflush(f);
    fclose(f);
    unlink(test_path);
    if (w != 1) {
        if (reason && reason_len) snprintf(reason, reason_len, "io_error");
        return false;
    }
    return true;
}

static void upload_session_reset(wifi_upload_session_t *s)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->state = WIFI_UPLOAD_STATE_IDLE;
}

static void set_cors_headers(httpd_req_t *req, char *allow_hdrs, size_t allow_hdrs_len)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");

    /* 更兼容的 CORS：优先回显预检请求的 header 列表，避免不同浏览器/环境额外附带头导致预检失败。 */
    const char *allow = "Content-Type,X-Upload-Token";
    if (allow_hdrs && allow_hdrs_len) {
        allow_hdrs[0] = '\0';
        if (httpd_req_get_hdr_value_str(req, "Access-Control-Request-Headers", allow_hdrs, allow_hdrs_len) == ESP_OK &&
            allow_hdrs[0] != '\0') {
            allow = allow_hdrs;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", allow);

    /* Chrome Private Network Access 兼容：HTTPS/安全上下文访问 192.168.x.x 时可能需要。 */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "600");
}

static esp_err_t send_json(httpd_req_t *req, const char *status, const char *json)
{
    char allow_hdrs[256] = {0};
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    set_cors_headers(req, allow_hdrs, sizeof(allow_hdrs));
    return httpd_resp_send(req, json ? json : "{}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_no_content(httpd_req_t *req)
{
    char allow_hdrs[256] = {0};
    httpd_resp_set_status(req, "204 No Content");
    set_cors_headers(req, allow_hdrs, sizeof(allow_hdrs));
    return httpd_resp_send(req, NULL, 0);
}

static void ota_restart_timer_cb(void *arg)
{
    (void)arg;
    esp_restart();
}

static void ota_schedule_restart_delayed(uint32_t delay_ms)
{
    if (!s_ota_restart_timer) return;
    esp_timer_stop(s_ota_restart_timer);
    esp_timer_start_once(s_ota_restart_timer, (uint64_t)delay_ms * 1000ULL);
}

static esp_err_t httpd_cors_err_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (!req) return ESP_FAIL;

    if (req->method == HTTP_OPTIONS) {
        return send_no_content(req);
    }

    const char *status = "500 Internal Server Error";
    const char *reason = "http_error";
    switch (err) {
    case HTTPD_404_NOT_FOUND:
        status = "404 Not Found";
        reason = "not_found";
        break;
    case HTTPD_405_METHOD_NOT_ALLOWED:
        status = "405 Method Not Allowed";
        reason = "method_not_allowed";
        break;
    case HTTPD_408_REQ_TIMEOUT:
        status = "408 Request Timeout";
        reason = "timeout";
        break;
    case HTTPD_411_LENGTH_REQUIRED:
        status = "411 Length Required";
        reason = "length_required";
        break;
    case HTTPD_413_CONTENT_TOO_LARGE:
        status = "413 Content Too Large";
        reason = "content_too_large";
        break;
    case HTTPD_414_URI_TOO_LONG:
        status = "414 URI Too Long";
        reason = "uri_too_long";
        break;
    case HTTPD_431_REQ_HDR_FIELDS_TOO_LARGE:
        status = "431 Request Header Fields Too Large";
        reason = "hdr_too_large";
        break;
    default:
        break;
    }

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"failed\",\"reason\":\"%s\"}", reason);
    return send_json(req, status, resp);
}

static bool get_query_param(httpd_req_t *req, const char *key, char *out, size_t out_len)
{
    if (!req || !key || !out || out_len == 0) return false;
    out[0] = '\0';
    const size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen == 0) return false;
    char *q = (char *)malloc(qlen + 1);
    if (!q) return false;
    if (httpd_req_get_url_query_str(req, q, qlen + 1) != ESP_OK) {
        free(q);
        return false;
    }
    esp_err_t r = httpd_query_key_value(q, key, out, out_len);
    free(q);
    return r == ESP_OK && out[0] != '\0';
}

static bool upload_is_busy_state(wifi_upload_state_t st)
{
    return st == WIFI_UPLOAD_STATE_PENDING_CONFIRM ||
           st == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD ||
           st == WIFI_UPLOAD_STATE_UPLOADING;
}

static bool ota_partition_precheck(uint32_t required_bytes, char *reason, size_t reason_len)
{
    if (!reason || reason_len == 0) return false;
    reason[0] = '\0';

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        snprintf(reason, reason_len, "%s", "no_ota_partition");
        return false;
    }

    if (required_bytes == 0 || required_bytes > update->size) {
        snprintf(reason, reason_len, "%s", "too_large");
        return false;
    }

    return true;
}

// ===================== HTTP 处理 =====================
static esp_err_t ws_handler(httpd_req_t *req)
{
    int sock = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        s_ws_client_fd = sock;
        s_ws_connected = true;
        lvgl_ui_show_hint("Web Connected");
        ESP_LOGI(TAG, "WS handshake ok, fd=%d", sock);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ws recv hdr fail fd=%d: %s", sock, esp_err_to_name(ret));
        ws_mark_disconnected();
        return ret;
    }

    if (frame.len > 0) {
        frame.payload = (uint8_t *)malloc(frame.len);
        if (!frame.payload) {
            ESP_LOGW(TAG, "ws payload alloc fail len=%u", (unsigned)frame.len);
            return ESP_ERR_NO_MEM;
        }
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ws recv payload fail fd=%d: %s", sock, esp_err_to_name(ret));
            free(frame.payload);
            ws_mark_disconnected();
            return ret;
        }
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WS close fd=%d", sock);
        lvgl_ui_show_hint("Web Disconnected");
        ws_mark_disconnected();
    } else if (frame.type == HTTPD_WS_TYPE_PING) {
        frame.type = HTTPD_WS_TYPE_PONG;
        httpd_ws_send_frame(req, &frame);
    } else {
        ESP_LOGD(TAG, "WS frame fd=%d type=%d len=%u", sock, (int)frame.type, (unsigned)frame.len);
    }

    if (frame.payload) free(frame.payload);
    return ESP_OK;
}

static esp_err_t upload_options_handler(httpd_req_t *req)
{
    if (req) {
        ESP_LOGI(TAG, "OPTIONS %s", req->uri ? req->uri : "");
    }
    return send_no_content(req);
}

static esp_err_t upload_request_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;

    if (!s_upload_mu) {
        return send_json(req, "503 Service Unavailable", "{\"status\":\"failed\",\"reason\":\"not_ready\"}");
    }

    if (req->content_len == 0 || req->content_len > WIFI_UPLOAD_REQ_JSON_MAX) {
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    char peer_ip[64] = {0};
    get_peer_ip(req, peer_ip, sizeof(peer_ip));
    ESP_LOGI(TAG, "HTTP %s %s ip=%s len=%d",
             httpd_method_to_str(req->method),
             req->uri ? req->uri : "",
             peer_ip[0] ? peer_ip : "?",
             (int)req->content_len);

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (upload_is_busy_state(s_upload.state)) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"busy\"}");
    }
    xSemaphoreGive(s_upload_mu);

    char *body = (char *)malloc(req->content_len + 1);
    if (!body) {
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"no_mem\"}");
    }
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, body + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            free(body);
            return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
        }
        got += (size_t)r;
    }
    body[req->content_len] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    const cJSON *j_filename = cJSON_GetObjectItemCaseSensitive(root, "filename");
    const cJSON *j_size     = cJSON_GetObjectItemCaseSensitive(root, "size");
    const cJSON *j_type     = cJSON_GetObjectItemCaseSensitive(root, "type"); /* 兼容：允许省略，通过后缀推断 */
    const cJSON *j_sha256   = cJSON_GetObjectItemCaseSensitive(root, "sha256");

    if (!cJSON_IsString(j_filename) || !j_filename->valuestring ||
        !cJSON_IsNumber(j_size)) {
        cJSON_Delete(root);
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    const char *filename = j_filename->valuestring;
    const char *type = (j_type && cJSON_IsString(j_type)) ? j_type->valuestring : NULL;
    double size_d = j_size->valuedouble;
    if (size_d <= 0 || size_d > (double)WIFI_UPLOAD_MAX_SIZE) {
        cJSON_Delete(root);
        return send_json(req, "413 Payload Too Large", "{\"status\":\"rejected\",\"reason\":\"too_large\"}");
    }
    const uint32_t total_size = (uint32_t)size_d;
    ESP_LOGI(TAG, "upload meta: filename=%s size=%" PRIu32 " type=%s",
             filename ? filename : "",
             total_size,
             (type && type[0] != '\0') ? type : "(auto)");

    bool is_app = false;
    bool is_espdl = false;

    /* 宏观兼容策略：优先按文件后缀判断类型（更符合用户直觉），其次才看 type 字段。 */
    if (ends_with_casei(filename, ".bin")) {
        is_app = true;
        if (type && type[0] != '\0' && !str_eq_casei(type, "app") && !str_eq_casei(type, "ota")) {
            ESP_LOGW(TAG, "upload: type mismatch, infer app by suffix, type=%s filename=%s", type, filename);
        }
    } else if (ends_with_casei(filename, ".espdl")) {
        is_espdl = true;
        if (type && type[0] != '\0' && !str_eq_casei(type, "espdl")) {
            ESP_LOGW(TAG, "upload: type mismatch, infer espdl by suffix, type=%s filename=%s", type, filename);
        }
    } else if (type && type[0] != '\0') {
        if (str_eq_casei(type, "espdl")) {
            is_espdl = true;
        } else if (str_eq_casei(type, "app") || str_eq_casei(type, "ota")) {
            is_app = true;
        } else {
            ESP_LOGW(TAG, "upload reject unsupported_type: filename=%s type=%s",
                     filename ? filename : "", type ? type : "");
            cJSON_Delete(root);
            return send_json(req, "415 Unsupported Media Type", "{\"status\":\"rejected\",\"reason\":\"unsupported_type\"}");
        }
    } else {
        ESP_LOGW(TAG, "upload reject unsupported_type(auto): filename=%s", filename ? filename : "");
        cJSON_Delete(root);
        return send_json(req, "415 Unsupported Media Type", "{\"status\":\"rejected\",\"reason\":\"unsupported_type\"}");
    }

    if (is_app && fw_update_is_busy()) {
        cJSON_Delete(root);
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"fw_update\"}");
    }

    if (is_app) {
        char why_pre[48] = {0};
        if (!ota_partition_precheck(total_size, why_pre, sizeof(why_pre))) {
            cJSON_Delete(root);
            char resp[128];
            snprintf(resp, sizeof(resp), "{\"status\":\"rejected\",\"reason\":\"%s\"}",
                     why_pre[0] ? why_pre : "no_ota_partition");
            if (strcmp(why_pre, "too_large") == 0) {
                return send_json(req, "413 Payload Too Large", resp);
            }
            return send_json(req, "409 Conflict", resp);
        }
    }

    char filename_copy[260] = {0};
    char safe[260] = {0};
    char why[48] = {0};
    if (is_espdl) {
        if (!sanitize_filename_ascii(filename, safe, sizeof(safe), why, sizeof(why))) {
            cJSON_Delete(root);
            char resp[128];
            snprintf(resp, sizeof(resp), "{\"status\":\"rejected\",\"reason\":\"%s\"}", why[0] ? why : "invalid_filename");
            return send_json(req, "400 Bad Request", resp);
        }
    } else {
        /* app 固件升级不落盘：文件名仅用于显示，不能因为“奇怪文件名”阻断升级流程 */
        copy_basename_for_display(filename, filename_copy, sizeof(filename_copy));
        {
            char raw_hex[128] = {0};
            char ui_hex[128] = {0};
            str_hex_preview_32(filename, raw_hex, sizeof(raw_hex));
            str_hex_preview_32(filename_copy, ui_hex, sizeof(ui_hex));
            ESP_LOGI(TAG, "upload(app) filename raw='%s' ui='%s'", filename ? filename : "", filename_copy);
            ESP_LOGI(TAG, "upload(app) filename hex raw=[%s] ui=[%s]", raw_hex, ui_hex);
        }
        safe[0] = '\0';
    }
    if (is_espdl) {
        snprintf(filename_copy, sizeof(filename_copy), "%s", filename);
    }

    bool has_sha = false;
    char sha256_hex[80] = {0};
    if (j_sha256 && cJSON_IsString(j_sha256) && j_sha256->valuestring && j_sha256->valuestring[0] != '\0') {
        if (!is_hex64(j_sha256->valuestring)) {
            cJSON_Delete(root);
            return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_sha256\"}");
        }
        snprintf(sha256_hex, sizeof(sha256_hex), "%s", j_sha256->valuestring);
        has_sha = true;
    }
    cJSON_Delete(root);

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (upload_is_busy_state(s_upload.state)) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"busy\"}");
    }

    upload_session_reset(&s_upload);
    s_upload.created_ts_us = now_us();
    s_upload.total_size = total_size;
    snprintf(s_upload.orig_filename, sizeof(s_upload.orig_filename), "%s", filename_copy);
    snprintf(s_upload.safe_filename, sizeof(s_upload.safe_filename), "%s", safe);
    snprintf(s_upload.type, sizeof(s_upload.type), "%s", is_app ? "app" : "espdl");
    if (peer_ip[0]) {
        snprintf(s_upload.requester_ip, sizeof(s_upload.requester_ip), "%s", peer_ip);
    }
    if (has_sha) {
        s_upload.has_sha256 = true;
        snprintf(s_upload.sha256_hex, sizeof(s_upload.sha256_hex), "%s", sha256_hex);
    }
    random_hex(s_upload.request_id, sizeof(s_upload.request_id), 8);
    s_upload.state = WIFI_UPLOAD_STATE_PENDING_CONFIRM;
    s_upload_abort_requested = false;

    char rid[40];
    snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
    xSemaphoreGive(s_upload_mu);

    lvgl_ui_upload_show_request(rid, filename_copy, total_size, is_app ? "app" : "espdl", has_sha ? sha256_hex : NULL);
    TaskScheduling_PauseForSettings();

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"requestId\":\"%s\",\"status\":\"pending\"}", rid);
    return send_json(req, "202 Accepted", resp);
}

static esp_err_t ota_upload_request_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;

    if (!s_upload_mu) {
        return send_json(req, "503 Service Unavailable", "{\"status\":\"failed\",\"reason\":\"not_ready\"}");
    }
    if (fw_update_is_busy()) {
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"fw_update\"}");
    }

    if (req->content_len == 0 || req->content_len > WIFI_UPLOAD_REQ_JSON_MAX) {
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    char peer_ip[64] = {0};
    get_peer_ip(req, peer_ip, sizeof(peer_ip));
    ESP_LOGI(TAG, "HTTP %s %s ip=%s len=%d",
             httpd_method_to_str(req->method),
             req->uri ? req->uri : "",
             peer_ip[0] ? peer_ip : "?",
             (int)req->content_len);

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (upload_is_busy_state(s_upload.state)) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"busy\"}");
    }
    xSemaphoreGive(s_upload_mu);

    char *body = (char *)malloc(req->content_len + 1);
    if (!body) {
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"no_mem\"}");
    }
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, body + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            free(body);
            return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
        }
        got += (size_t)r;
    }
    body[req->content_len] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    const cJSON *j_filename = cJSON_GetObjectItemCaseSensitive(root, "filename");
    const cJSON *j_size     = cJSON_GetObjectItemCaseSensitive(root, "size");
    const cJSON *j_sha256   = cJSON_GetObjectItemCaseSensitive(root, "sha256");

    if (!cJSON_IsString(j_filename) || !j_filename->valuestring ||
        !cJSON_IsNumber(j_size)) {
        cJSON_Delete(root);
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    const char *filename = j_filename->valuestring;
    double size_d = j_size->valuedouble;
    if (size_d <= 0 || size_d > (double)WIFI_UPLOAD_MAX_SIZE) {
        cJSON_Delete(root);
        return send_json(req, "413 Payload Too Large", "{\"status\":\"rejected\",\"reason\":\"too_large\"}");
    }
    const uint32_t total_size = (uint32_t)size_d;

    if (!ends_with_casei(filename, ".bin")) {
        ESP_LOGW(TAG, "ota reject unsupported_type: filename=%s", filename ? filename : "");
        cJSON_Delete(root);
        return send_json(req, "415 Unsupported Media Type", "{\"status\":\"rejected\",\"reason\":\"unsupported_type\"}");
    }

    char sha256_hex[80] = {0};
    bool has_sha = false;
    if (j_sha256 && cJSON_IsString(j_sha256) && j_sha256->valuestring && j_sha256->valuestring[0] != '\0') {
        if (!is_hex64(j_sha256->valuestring)) {
            cJSON_Delete(root);
            return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_sha256\"}");
        }
        snprintf(sha256_hex, sizeof(sha256_hex), "%s", j_sha256->valuestring);
        has_sha = true;
    }
    cJSON_Delete(root);

    char why[48] = {0};
    if (!ota_partition_precheck(total_size, why, sizeof(why))) {
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"status\":\"rejected\",\"reason\":\"%s\"}", why[0] ? why : "no_ota_partition");
        if (strcmp(why, "too_large") == 0) {
            return send_json(req, "413 Payload Too Large", resp);
        }
        return send_json(req, "409 Conflict", resp);
    }

    /* OTA 不落文件系统：filename 只用于 UI 展示，不能因前端传来的“路径/Unicode/奇怪字符”而拒绝 */
    char filename_copy[260] = {0};
    copy_basename_for_display(filename, filename_copy, sizeof(filename_copy));
    {
        char raw_hex[128] = {0};
        char ui_hex[128] = {0};
        str_hex_preview_32(filename, raw_hex, sizeof(raw_hex));
        str_hex_preview_32(filename_copy, ui_hex, sizeof(ui_hex));
        ESP_LOGI(TAG, "ota filename raw='%s' ui='%s'", filename ? filename : "", filename_copy);
        ESP_LOGI(TAG, "ota filename hex raw=[%s] ui=[%s]", raw_hex, ui_hex);
    }
    char safe[4] = {0}; /* 占位：app 不使用 safe_filename */

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (upload_is_busy_state(s_upload.state)) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"busy\",\"reason\":\"busy\"}");
    }

    upload_session_reset(&s_upload);
    s_upload.created_ts_us = now_us();
    s_upload.total_size = total_size;
    snprintf(s_upload.orig_filename, sizeof(s_upload.orig_filename), "%s", filename_copy);
    snprintf(s_upload.safe_filename, sizeof(s_upload.safe_filename), "%s", safe);
    snprintf(s_upload.type, sizeof(s_upload.type), "%s", "app");
    if (peer_ip[0]) {
        snprintf(s_upload.requester_ip, sizeof(s_upload.requester_ip), "%s", peer_ip);
    }
    if (has_sha) {
        s_upload.has_sha256 = true;
        snprintf(s_upload.sha256_hex, sizeof(s_upload.sha256_hex), "%s", sha256_hex);
    }
    random_hex(s_upload.request_id, sizeof(s_upload.request_id), 8);
    s_upload.state = WIFI_UPLOAD_STATE_PENDING_CONFIRM;
    s_upload_abort_requested = false;

    char rid[40];
    snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
    xSemaphoreGive(s_upload_mu);

    lvgl_ui_upload_show_request(rid, filename_copy, total_size, "app", has_sha ? sha256_hex : NULL);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"requestId\":\"%s\",\"status\":\"pending\"}", rid);
    return send_json(req, "202 Accepted", resp);
}

static esp_err_t upload_status_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;
    char rid[40];
    if (!get_query_param(req, "requestId", rid, sizeof(rid))) {
        return send_json(req, "400 Bad Request", "{\"status\":\"rejected\",\"reason\":\"invalid_metadata\"}");
    }

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (s_upload.request_id[0] == '\0' || strcmp(rid, s_upload.request_id) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "404 Not Found", "{\"status\":\"not_found\"}");
    }

    if (s_upload.state == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD) {
        const int64_t elapsed_ms = (now_us() - s_upload.accepted_ts_us) / 1000;
        if (elapsed_ms > WIFI_UPLOAD_ACCEPT_WAIT_MS) {
            s_upload.state = WIFI_UPLOAD_STATE_EXPIRED;
            snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "upload_timeout");
            char rid_copy[40];
            snprintf(rid_copy, sizeof(rid_copy), "%s", s_upload.request_id);
            xSemaphoreGive(s_upload_mu);
            lvgl_ui_upload_finish(rid_copy, false, "Upload timeout");
            wifi_upload_schedule_resume_delayed(1500);
            char resp_to[128];
            snprintf(resp_to, sizeof(resp_to),
                     "{\"requestId\":\"%s\",\"status\":\"expired\",\"reason\":\"upload_timeout\"}",
                     rid_copy);
            return send_json(req, "200 OK", resp_to);
        }
    }

    wifi_upload_state_t st = s_upload.state;
    char token[40] = {0};
    char reason[48] = {0};
    uint32_t received = s_upload.received;
    uint32_t total = s_upload.total_size;
    if (s_upload.reason[0]) snprintf(reason, sizeof(reason), "%s", s_upload.reason);
    if (s_upload.token[0]) snprintf(token, sizeof(token), "%s", s_upload.token);
    xSemaphoreGive(s_upload_mu);

    const char *st_str = upload_state_to_str(st);
    char resp[256];
    if (st == WIFI_UPLOAD_STATE_UPLOADING) {
        if (reason[0]) {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\",\"token\":\"%s\",\"received\":%lu,\"total\":%lu,\"reason\":\"%s\"}",
                     rid, st_str, token,
                     (unsigned long)received, (unsigned long)total,
                     reason);
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\",\"token\":\"%s\",\"received\":%lu,\"total\":%lu}",
                     rid, st_str, token,
                     (unsigned long)received, (unsigned long)total);
        }
    } else if (st == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD) {
        if (reason[0]) {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\",\"token\":\"%s\",\"reason\":\"%s\"}",
                     rid, st_str, token, reason);
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\",\"token\":\"%s\"}",
                     rid, st_str, token);
        }
    } else {
        if (reason[0]) {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\"}",
                     rid, st_str, reason);
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"requestId\":\"%s\",\"status\":\"%s\"}",
                     rid, st_str);
        }
    }

    return send_json(req, "200 OK", resp);
}

static esp_err_t ota_upload_status_handler(httpd_req_t *req)
{
    return upload_status_handler(req);
}

static esp_err_t upload_data_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;
    char rid[40];
    if (!get_query_param(req, "requestId", rid, sizeof(rid))) {
        return send_json(req, "400 Bad Request", "{\"status\":\"failed\",\"reason\":\"invalid_metadata\"}");
    }

    char token_hdr[48] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-Upload-Token", token_hdr, sizeof(token_hdr)) != ESP_OK) {
        return send_json(req, "401 Unauthorized", "{\"status\":\"failed\",\"reason\":\"token_mismatch\"}");
    }

    char peer_ip[64] = {0};
    get_peer_ip(req, peer_ip, sizeof(peer_ip));
    ESP_LOGI(TAG, "upload data start rid=%s len=%d ip=%s", rid, (int)req->content_len, peer_ip[0] ? peer_ip : "?");

    uint32_t total = 0;
    bool has_sha = false;
    char sha_expect[80] = {0};
    char safe_name[260] = {0};
    char final_path[384] = {0};
    char part_path[384] = {0};

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (s_upload.request_id[0] == '\0' || strcmp(rid, s_upload.request_id) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "404 Not Found", "{\"status\":\"failed\",\"reason\":\"not_found\"}");
    }
    /* 兼容：若通过 /api/model/upload/ 走的是固件升级（type=app），则复用 OTA data handler */
    if (str_eq_casei(s_upload.type, "app")) {
        xSemaphoreGive(s_upload_mu);
        return ota_upload_data_handler(req);
    }
    if (!str_eq_casei(s_upload.type, "espdl")) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"invalid_state\"}");
    }
    if (s_upload.state != WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"invalid_state\"}");
    }
    if (s_upload.token[0] == '\0' || strcmp(token_hdr, s_upload.token) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "401 Unauthorized", "{\"status\":\"failed\",\"reason\":\"token_mismatch\"}");
    }
    if (s_upload.requester_ip[0] && peer_ip[0] && strcmp(peer_ip, s_upload.requester_ip) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "403 Forbidden", "{\"status\":\"failed\",\"reason\":\"ip_mismatch\"}");
    }

    const int64_t elapsed_ms = (now_us() - s_upload.accepted_ts_us) / 1000;
    if (elapsed_ms > WIFI_UPLOAD_ACCEPT_WAIT_MS) {
        s_upload.state = WIFI_UPLOAD_STATE_EXPIRED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "upload_timeout");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Upload timeout");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"upload_timeout\"}");
    }

    total = s_upload.total_size;
    snprintf(safe_name, sizeof(safe_name), "%s", s_upload.safe_filename);
    has_sha = s_upload.has_sha256;
    if (has_sha) snprintf(sha_expect, sizeof(sha_expect), "%s", s_upload.sha256_hex);

    snprintf(final_path, sizeof(final_path), "/sdcard/models/%s", safe_name);
    snprintf(part_path, sizeof(part_path), "/sdcard/models/%s.part", safe_name);
    snprintf(s_upload.final_path, sizeof(s_upload.final_path), "%s", final_path);
    snprintf(s_upload.part_path, sizeof(s_upload.part_path), "%s", part_path);

    struct stat st;
    if (stat(final_path, &st) == 0) {
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "name_conflict");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "File exists");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"name_conflict\"}");
    }

    if (req->content_len > 0 && (uint32_t)req->content_len != total) {
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "size_mismatch");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Size mismatch");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "400 Bad Request", "{\"status\":\"failed\",\"reason\":\"size_mismatch\"}");
    }

    s_upload.state = WIFI_UPLOAD_STATE_UPLOADING;
    s_upload.received = 0;
    s_upload_abort_requested = false;
    s_upload.reason[0] = '\0';
    xSemaphoreGive(s_upload_mu);

    TaskScheduling_PauseForSettings();

    FILE *f = fopen(part_path, "wb");
    if (!f) {
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "io_error");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Write failed");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"io_error\"}");
    }

    mbedtls_sha256_context sha;
    uint8_t sha_out[32];
    bool sha_ok = true;
    if (has_sha) {
        mbedtls_sha256_init(&sha);
        if (mbedtls_sha256_starts(&sha, 0) != 0) {
            sha_ok = false;
        }
    }

    uint8_t *buf = (uint8_t *)malloc(WIFI_UPLOAD_IO_CHUNK);
    if (!buf) {
        if (has_sha) {
            mbedtls_sha256_free(&sha);
        }
        fclose(f);
        unlink(part_path);
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "no_mem");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "No memory");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"no_mem\"}");
    }

    uint32_t received = 0;
    int64_t last_ui_us = 0;
    bool failed = false;
    char fail_reason[48] = {0};

    while (received < total) {
        if (s_upload_abort_requested) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "user_cancel");
            break;
        }
        const uint32_t need = total - received;
        const uint32_t chunk = (need > WIFI_UPLOAD_IO_CHUNK) ? WIFI_UPLOAD_IO_CHUNK : need;
        int r = httpd_req_recv(req, (char *)buf, chunk);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "net_error");
            ESP_LOGW(TAG, "upload recv fail rid=%s r=%d %lu/%lu", rid, r,
                     (unsigned long)received, (unsigned long)total);
            break;
        }

        if (has_sha && sha_ok) {
            if (mbedtls_sha256_update(&sha, buf, (size_t)r) != 0) {
                sha_ok = false;
            }
        }

        size_t w = fwrite(buf, 1, (size_t)r, f);
        if (w != (size_t)r) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "io_error");
            break;
        }

        received += (uint32_t)r;

        const int64_t t = now_us();
        if (t - last_ui_us >= (int64_t)WIFI_UPLOAD_PROGRESS_UI_MS * 1000) {
            last_ui_us = t;
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            s_upload.received = received;
            xSemaphoreGive(s_upload_mu);
            lvgl_ui_upload_update_progress(rid, received, total);
        }
    }

    fflush(f);
    fclose(f);
    free(buf);

    if (!failed && received != total) {
        failed = true;
        snprintf(fail_reason, sizeof(fail_reason), "%s", "size_mismatch");
    }

    if (has_sha) {
        if (!failed) {
            if (!sha_ok || mbedtls_sha256_finish(&sha, sha_out) != 0) {
                failed = true;
                snprintf(fail_reason, sizeof(fail_reason), "%s", "hash_error");
            } else {
                char sha_hex[65];
                for (int i = 0; i < 32; ++i) {
                    snprintf(sha_hex + i * 2, 3, "%02x", sha_out[i]);
                }
                sha_hex[64] = '\0';
                if (!str_eq_casei(sha_hex, sha_expect)) {
                    failed = true;
                    snprintf(fail_reason, sizeof(fail_reason), "%s", "hash_mismatch");
                }
            }
        }
        mbedtls_sha256_free(&sha);
    }

    if (failed) {
        unlink(part_path);
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", fail_reason[0] ? fail_reason : "failed");
        xSemaphoreGive(s_upload_mu);

        if (strcmp(fail_reason, "user_cancel") == 0) {
            lvgl_ui_upload_finish(rid, false, "Canceled");
        } else if (strcmp(fail_reason, "hash_mismatch") == 0) {
            lvgl_ui_upload_finish(rid, false, "SHA256 mismatch");
        } else {
            lvgl_ui_upload_finish(rid, false, "Upload failed");
        }
        wifi_upload_schedule_resume_delayed(1500);
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"status\":\"failed\",\"reason\":\"%s\"}", fail_reason[0] ? fail_reason : "failed");
        return send_json(req, "500 Internal Server Error", resp);
    }

    if (rename(part_path, final_path) != 0) {
        unlink(part_path);
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "io_error");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Save failed");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"io_error\"}");
    }

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    s_upload.state = WIFI_UPLOAD_STATE_SUCCESS;
    s_upload.received = total;
    s_upload.reason[0] = '\0';
    xSemaphoreGive(s_upload_mu);

    lvgl_ui_upload_update_progress(rid, total, total);
    lvgl_ui_upload_finish(rid, true, "Saved");
    lvgl_ui_refresh_model_list();
    wifi_upload_schedule_resume_delayed(1500);

    char resp[192];
    snprintf(resp, sizeof(resp), "{\"status\":\"success\",\"filename\":\"%s\"}", safe_name);
    return send_json(req, "200 OK", resp);
}

static esp_err_t ota_upload_data_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;
    char rid[40];
    if (!get_query_param(req, "requestId", rid, sizeof(rid))) {
        return send_json(req, "400 Bad Request", "{\"status\":\"failed\",\"reason\":\"invalid_metadata\"}");
    }

    char token_hdr[48] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-Upload-Token", token_hdr, sizeof(token_hdr)) != ESP_OK) {
        return send_json(req, "401 Unauthorized", "{\"status\":\"failed\",\"reason\":\"token_mismatch\"}");
    }

    char peer_ip[64] = {0};
    get_peer_ip(req, peer_ip, sizeof(peer_ip));
    ESP_LOGI(TAG, "ota upload data start rid=%s len=%d ip=%s", rid, (int)req->content_len, peer_ip[0] ? peer_ip : "?");

    uint32_t total = 0;
    bool has_sha = false;
    char sha_expect[80] = {0};

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (s_upload.request_id[0] == '\0' || strcmp(rid, s_upload.request_id) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "404 Not Found", "{\"status\":\"failed\",\"reason\":\"not_found\"}");
    }
    if (!str_eq_casei(s_upload.type, "app")) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"invalid_state\"}");
    }
    if (s_upload.state != WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"invalid_state\"}");
    }
    if (s_upload.token[0] == '\0' || strcmp(token_hdr, s_upload.token) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "401 Unauthorized", "{\"status\":\"failed\",\"reason\":\"token_mismatch\"}");
    }
    if (s_upload.requester_ip[0] && peer_ip[0] && strcmp(peer_ip, s_upload.requester_ip) != 0) {
        xSemaphoreGive(s_upload_mu);
        return send_json(req, "403 Forbidden", "{\"status\":\"failed\",\"reason\":\"ip_mismatch\"}");
    }

    const int64_t elapsed_ms = (now_us() - s_upload.accepted_ts_us) / 1000;
    if (elapsed_ms > WIFI_UPLOAD_ACCEPT_WAIT_MS) {
        s_upload.state = WIFI_UPLOAD_STATE_EXPIRED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "upload_timeout");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Upload timeout");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"upload_timeout\"}");
    }

    total = s_upload.total_size;
    has_sha = s_upload.has_sha256;
    if (has_sha) snprintf(sha_expect, sizeof(sha_expect), "%s", s_upload.sha256_hex);

    if (req->content_len > 0 && (uint32_t)req->content_len != total) {
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "size_mismatch");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Size mismatch");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "400 Bad Request", "{\"status\":\"failed\",\"reason\":\"size_mismatch\"}");
    }

    s_upload.state = WIFI_UPLOAD_STATE_UPLOADING;
    s_upload.received = 0;
    s_upload_abort_requested = false;
    s_upload.reason[0] = '\0';
    xSemaphoreGive(s_upload_mu);

    TaskScheduling_PauseForSettings();

    char why[48] = {0};
    if (!ota_partition_precheck(total, why, sizeof(why))) {
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", why[0] ? why : "no_ota_partition");
        xSemaphoreGive(s_upload_mu);
        if (strcmp(why, "too_large") == 0) {
            lvgl_ui_upload_finish(rid, false, "Firmware too large");
        } else {
            lvgl_ui_upload_finish(rid, false, "No OTA partition");
        }
        wifi_upload_schedule_resume_delayed(1500);
        if (strcmp(why, "too_large") == 0) {
            return send_json(req, "413 Payload Too Large", "{\"status\":\"failed\",\"reason\":\"too_large\"}");
        }
        return send_json(req, "409 Conflict", "{\"status\":\"failed\",\"reason\":\"no_ota_partition\"}");
    }

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota = 0;
    esp_err_t r = esp_ota_begin(update, total, &ota);
    if (r != ESP_OK) {
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "ota_begin_failed");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "OTA begin failed");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"ota_begin_failed\"}");
    }

    mbedtls_sha256_context sha;
    uint8_t sha_out[32];
    bool sha_ok = true;
    if (has_sha) {
        mbedtls_sha256_init(&sha);
        if (mbedtls_sha256_starts(&sha, 0) != 0) {
            sha_ok = false;
        }
    }

    uint8_t *buf = (uint8_t *)malloc(WIFI_UPLOAD_IO_CHUNK);
    if (!buf) {
        if (has_sha) {
            mbedtls_sha256_free(&sha);
        }
        esp_ota_abort(ota);
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "no_mem");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "No memory");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"no_mem\"}");
    }

    uint32_t received = 0;
    int64_t last_ui_us = 0;
    bool failed = false;
    char fail_reason[48] = {0};

    while (received < total) {
        if (s_upload_abort_requested) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "user_cancel");
            break;
        }
        const uint32_t need = total - received;
        const uint32_t chunk = (need > WIFI_UPLOAD_IO_CHUNK) ? WIFI_UPLOAD_IO_CHUNK : need;
        int n = httpd_req_recv(req, (char *)buf, chunk);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "net_error");
            ESP_LOGW(TAG, "ota upload recv fail rid=%s r=%d %lu/%lu", rid, n,
                     (unsigned long)received, (unsigned long)total);
            break;
        }

        if (has_sha && sha_ok) {
            if (mbedtls_sha256_update(&sha, buf, (size_t)n) != 0) {
                sha_ok = false;
            }
        }

        r = esp_ota_write(ota, buf, (size_t)n);
        if (r != ESP_OK) {
            failed = true;
            snprintf(fail_reason, sizeof(fail_reason), "%s", "ota_write_failed");
            break;
        }

        received += (uint32_t)n;

        const int64_t t = now_us();
        if (t - last_ui_us >= (int64_t)WIFI_UPLOAD_PROGRESS_UI_MS * 1000) {
            last_ui_us = t;
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            s_upload.received = received;
            xSemaphoreGive(s_upload_mu);
            lvgl_ui_upload_update_progress(rid, received, total);
        }
    }

    free(buf);

    if (!failed && received != total) {
        failed = true;
        snprintf(fail_reason, sizeof(fail_reason), "%s", "size_mismatch");
    }

    if (failed) {
        if (has_sha) {
            mbedtls_sha256_free(&sha);
        }
        esp_ota_abort(ota);
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", fail_reason[0] ? fail_reason : "failed");
        xSemaphoreGive(s_upload_mu);

        if (strcmp(fail_reason, "user_cancel") == 0) {
            lvgl_ui_upload_finish(rid, false, "Canceled");
        } else {
            lvgl_ui_upload_finish(rid, false, "Upload failed");
        }
        wifi_upload_schedule_resume_delayed(1500);
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"status\":\"failed\",\"reason\":\"%s\"}", fail_reason[0] ? fail_reason : "failed");
        return send_json(req, "500 Internal Server Error", resp);
    }

    r = esp_ota_end(ota);
    if (r != ESP_OK) {
        if (has_sha) {
            mbedtls_sha256_free(&sha);
        }
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "ota_end_failed");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "OTA end failed");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"ota_end_failed\"}");
    }

    if (has_sha) {
        if (!sha_ok || mbedtls_sha256_finish(&sha, sha_out) != 0) {
            mbedtls_sha256_free(&sha);
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            s_upload.state = WIFI_UPLOAD_STATE_FAILED;
            snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "hash_error");
            xSemaphoreGive(s_upload_mu);
            lvgl_ui_upload_finish(rid, false, "SHA256 error");
            wifi_upload_schedule_resume_delayed(1500);
            return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"hash_error\"}");
        }
        char sha_hex[65];
        for (int i = 0; i < 32; ++i) {
            snprintf(sha_hex + i * 2, 3, "%02x", sha_out[i]);
        }
        sha_hex[64] = '\0';
        mbedtls_sha256_free(&sha);
        if (!str_eq_casei(sha_hex, sha_expect)) {
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            s_upload.state = WIFI_UPLOAD_STATE_FAILED;
            snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "hash_mismatch");
            xSemaphoreGive(s_upload_mu);
            lvgl_ui_upload_finish(rid, false, "SHA256 mismatch");
            wifi_upload_schedule_resume_delayed(1500);
            return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"hash_mismatch\"}");
        }
    }

    r = esp_ota_set_boot_partition(update);
    if (r != ESP_OK) {
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        s_upload.state = WIFI_UPLOAD_STATE_FAILED;
        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "set_boot_failed");
        xSemaphoreGive(s_upload_mu);
        lvgl_ui_upload_finish(rid, false, "Set boot failed");
        wifi_upload_schedule_resume_delayed(1500);
        return send_json(req, "500 Internal Server Error", "{\"status\":\"failed\",\"reason\":\"set_boot_failed\"}");
    }

    {
        char orig_name[260] = {0};
        xSemaphoreTake(s_upload_mu, portMAX_DELAY);
        snprintf(orig_name, sizeof(orig_name), "%s", s_upload.orig_filename);
        xSemaphoreGive(s_upload_mu);

        if (orig_name[0] != '\0') {
            esp_err_t verr = device_identity_set_fw_version_label_from_filename(orig_name);
            if (verr != ESP_OK) {
                ESP_LOGW(TAG, "save fw label failed: filename=%s err=%s",
                         orig_name, esp_err_to_name(verr));
            }
        }
    }

    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    s_upload.state = WIFI_UPLOAD_STATE_SUCCESS;
    s_upload.received = total;
    s_upload.reason[0] = '\0';
    xSemaphoreGive(s_upload_mu);

    lvgl_ui_upload_update_progress(rid, total, total);
    lvgl_ui_upload_finish(rid, true, "升级完成，重启中...");
    ota_schedule_restart_delayed(1200);

    return send_json(req, "200 OK", "{\"status\":\"success\",\"action\":\"reboot\"}");
}

static bool httpd_start_server(void)
{
    if (s_httpd) {
        return true;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12 * 1024;
    /* 默认 max_uri_handlers=8 不够（本工程注册 WS + 6 个 upload API + 6 个 OPTIONS），
     * 会导致后续 handler 注册失败，从而网页端出现 405 method_not_allowed。 */
    config.max_uri_handlers = 20;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 60;
    config.core_id = 1; /* httpd 跑在 Core1 */

    httpd_handle_t server = NULL;
    esp_err_t r = httpd_start(&server, &config);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s(0x%x)", esp_err_to_name(r), r);
        return false;
    }

    s_httpd = server;
    s_ws_client_fd = -1;
    s_ws_connected = false;
    s_streaming_on = false;
    s_ws_job_pending = false;

    httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .user_ctx     = NULL,
        .is_websocket = true
    };
    r = httpd_register_uri_handler(server, &ws_uri);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "register %s %s failed: %s(0x%x)",
                 httpd_method_to_str(ws_uri.method), ws_uri.uri, esp_err_to_name(r), r);
        httpd_stop(server);
        s_httpd = NULL;
        return false;
    }

    httpd_uri_t opt_req = {
        .uri     = "/api/model/upload/request",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t opt_status = {
        .uri     = "/api/model/upload/status",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t opt_data = {
        .uri     = "/api/model/upload/data",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t *model_opts[] = { &opt_req, &opt_status, &opt_data };
    for (size_t i = 0; i < (sizeof(model_opts) / sizeof(model_opts[0])); ++i) {
        r = httpd_register_uri_handler(server, model_opts[i]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s failed: %s(0x%x)",
                     httpd_method_to_str(model_opts[i]->method), model_opts[i]->uri,
                     esp_err_to_name(r), r);
            httpd_stop(server);
            s_httpd = NULL;
            return false;
        }
    }

    httpd_uri_t ota_opt_req = {
        .uri     = "/api/ota/upload/request",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t ota_opt_status = {
        .uri     = "/api/ota/upload/status",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t ota_opt_data = {
        .uri     = "/api/ota/upload/data",
        .method  = HTTP_OPTIONS,
        .handler = upload_options_handler,
    };
    httpd_uri_t *ota_opts[] = { &ota_opt_req, &ota_opt_status, &ota_opt_data };
    for (size_t i = 0; i < (sizeof(ota_opts) / sizeof(ota_opts[0])); ++i) {
        r = httpd_register_uri_handler(server, ota_opts[i]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s failed: %s(0x%x)",
                     httpd_method_to_str(ota_opts[i]->method), ota_opts[i]->uri,
                     esp_err_to_name(r), r);
            httpd_stop(server);
            s_httpd = NULL;
            return false;
        }
    }

    httpd_uri_t upload_req = {
        .uri     = "/api/model/upload/request",
        .method  = HTTP_POST,
        .handler = upload_request_handler,
    };
    httpd_uri_t upload_status = {
        .uri     = "/api/model/upload/status",
        .method  = HTTP_GET,
        .handler = upload_status_handler,
    };
    httpd_uri_t upload_data = {
        .uri     = "/api/model/upload/data",
        .method  = HTTP_POST,
        .handler = upload_data_handler,
    };
    httpd_uri_t *model_apis[] = { &upload_req, &upload_status, &upload_data };
    for (size_t i = 0; i < (sizeof(model_apis) / sizeof(model_apis[0])); ++i) {
        r = httpd_register_uri_handler(server, model_apis[i]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s failed: %s(0x%x)",
                     httpd_method_to_str(model_apis[i]->method), model_apis[i]->uri,
                     esp_err_to_name(r), r);
            httpd_stop(server);
            s_httpd = NULL;
            return false;
        }
    }

    httpd_uri_t ota_req = {
        .uri     = "/api/ota/upload/request",
        .method  = HTTP_POST,
        .handler = ota_upload_request_handler,
    };
    httpd_uri_t ota_status = {
        .uri     = "/api/ota/upload/status",
        .method  = HTTP_GET,
        .handler = ota_upload_status_handler,
    };
    httpd_uri_t ota_data = {
        .uri     = "/api/ota/upload/data",
        .method  = HTTP_POST,
        .handler = ota_upload_data_handler,
    };
    httpd_uri_t *ota_apis[] = { &ota_req, &ota_status, &ota_data };
    for (size_t i = 0; i < (sizeof(ota_apis) / sizeof(ota_apis[0])); ++i) {
        r = httpd_register_uri_handler(server, ota_apis[i]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s failed: %s(0x%x)",
                     httpd_method_to_str(ota_apis[i]->method), ota_apis[i]->uri,
                     esp_err_to_name(r), r);
            httpd_stop(server);
            s_httpd = NULL;
            return false;
        }
    }

    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_405_METHOD_NOT_ALLOWED, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_408_REQ_TIMEOUT, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_411_LENGTH_REQUIRED, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_413_CONTENT_TOO_LARGE, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_414_URI_TOO_LONG, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_431_REQ_HDR_FIELDS_TOO_LARGE, httpd_cors_err_handler);
    httpd_register_err_handler(server, HTTPD_500_INTERNAL_SERVER_ERROR, httpd_cors_err_handler);

    ESP_LOGI(TAG, "HTTPD started");
    return true;
}

static void httpd_stop_server(void)
{
    wifi_upload_abort_and_cleanup("server_stop");
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    ws_mark_disconnected();
}

// ===================== Wi-Fi 基础 =====================
static esp_err_t wifi_core_init(void)
{
    static bool nvs_ready = false;
    if (!nvs_ready) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "nvs flash needs erase");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) return ret;
        nvs_ready = true;
    }

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init fail: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event_loop_create_default fail: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_netif_ap == NULL) {
        s_netif_ap = esp_netif_create_default_wifi_ap();
    }
    if (s_netif_sta == NULL) {
        s_netif_sta = esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_INIT_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init fail: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!s_event_handler_registered) {
        esp_event_handler_instance_t any_id;
        esp_event_handler_instance_t ip_id;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &ip_id));
        s_event_handler_registered = true;
    }

    if (!s_upload_mu) {
        s_upload_mu = xSemaphoreCreateMutex();
    }
    if (!s_upload_evt_q) {
        s_upload_evt_q = xQueueCreate(4, sizeof(wifi_upload_evt_t));
    }
    if (!s_upload_task && s_upload_mu && s_upload_evt_q) {
        xTaskCreatePinnedToCore(wifi_upload_task, "wifi_upload", 4096, NULL, 3, &s_upload_task, 1);
    }
    if (!s_upload_resume_timer) {
        esp_timer_create_args_t t = {
            .callback = wifi_upload_resume_timer_cb,
            .name = "upload_resume",
        };
        esp_timer_create(&t, &s_upload_resume_timer);
    }
    if (!s_ota_restart_timer) {
        esp_timer_create_args_t t = {
            .callback = ota_restart_timer_cb,
            .name = "ota_restart",
        };
        esp_timer_create(&t, &s_ota_restart_timer);
    }

    return ESP_OK;
}

static void wifi_apply_power_settings(void)
{
    /* esp_wifi_set_max_tx_power() 的单位是 0.25dBm：
     * - 78=19.5dBm（更强信号，但峰值电流更大）
     * - 72=18.0dBm（折中）
     *
     * 这里按模式做区分：AP 更强，STA 略低，提升连接成功率的同时尽量控制风险。
     */
    wifi_mode_t mode = WIFI_MODE_NULL;
    (void)esp_wifi_get_mode(&mode);

    int8_t max_tx_power = 72; // 默认 18dBm
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        max_tx_power = 78; // 19.5dBm
    } else if (mode == WIFI_MODE_STA) {
        max_tx_power = 72; // 18dBm
    }

    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    (void)esp_wifi_set_max_tx_power(max_tx_power);
    ESP_LOGI(TAG, "wifi power: mode=%d max_tx=%d(0.25dBm)", (int)mode, (int)max_tx_power);
}

bool wifi_start_ap_mode(void)
{
    wifi_api_lock();
    bool ok = false;

    if (wifi_core_init() != ESP_OK) {
        goto out;
    }

    s_run_mode = WIFI_RUN_MODE_NONE;
    s_sta_connecting = false;
    s_sta_connected = false;
    httpd_stop_server();
    esp_wifi_stop();

    wifi_config_t w = {0};
    memcpy(w.ap.ssid, EXAMPLE_WIFI_AP_SSID, strlen(EXAMPLE_WIFI_AP_SSID));
    w.ap.ssid_len       = strlen(EXAMPLE_WIFI_AP_SSID);
    memcpy(w.ap.password, EXAMPLE_WIFI_AP_PASS, strlen(EXAMPLE_WIFI_AP_PASS));
    w.ap.channel        = EXAMPLE_WIFI_AP_CHANNEL;
    w.ap.max_connection = EXAMPLE_WIFI_AP_MAX_CONN;
    w.ap.ssid_hidden    = 0;
    w.ap.beacon_interval= 100;
    w.ap.authmode       = strlen(EXAMPLE_WIFI_AP_PASS) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_mode AP fail: %s", esp_err_to_name(ret));
        goto out;
    }
    ret = esp_wifi_set_config(WIFI_IF_AP, &w);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_config AP fail: %s", esp_err_to_name(ret));
        goto out;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi start AP fail: %s", esp_err_to_name(ret));
        goto out;
    }

    wifi_apply_power_settings();
    s_run_mode = WIFI_RUN_MODE_AP;
    s_wifi_started = true;
    ESP_LOGI(TAG, "AP started: SSID=%s PASS=%s CH=%d MAX_CONN=%d",
             EXAMPLE_WIFI_AP_SSID, EXAMPLE_WIFI_AP_PASS, EXAMPLE_WIFI_AP_CHANNEL, EXAMPLE_WIFI_AP_MAX_CONN);
    ok = true;

out:
    wifi_api_unlock();
    return ok;
}

bool wifi_connect_sta(const char *ssid, const char *password)
{
    wifi_api_lock();
    bool ok = false;

    if (!ssid || ssid[0] == '\0') {
        ESP_LOGW(TAG, "wifi_connect_sta: empty ssid");
        goto out;
    }
    if (wifi_core_init() != ESP_OK) {
        goto out;
    }

    s_run_mode = WIFI_RUN_MODE_NONE;
    s_sta_connecting = false;
    s_sta_connected = false;
    httpd_stop_server();
    esp_wifi_stop();

    wifi_config_t cfg = {0};
    size_t ssid_len = strlen(ssid);
    if (ssid_len >= sizeof(cfg.sta.ssid)) ssid_len = sizeof(cfg.sta.ssid) - 1;
    memcpy(cfg.sta.ssid, ssid, ssid_len);

    size_t pass_len = password ? strlen(password) : 0;
    if (pass_len >= sizeof(cfg.sta.password)) pass_len = sizeof(cfg.sta.password) - 1;
    if (password && pass_len > 0) {
        memcpy(cfg.sta.password, password, pass_len);
    } else {
        cfg.sta.password[0] = '\0';
    }
    cfg.sta.threshold.authmode = (pass_len == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_mode STA fail: %s", esp_err_to_name(ret));
        goto out;
    }
    ret = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_config STA fail: %s", esp_err_to_name(ret));
        goto out;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "wifi start STA fail: %s", esp_err_to_name(ret));
        goto out;
    }
    wifi_apply_power_settings();

    s_run_mode = WIFI_RUN_MODE_STA;
    s_wifi_started = true;
    s_sta_connecting = true;
    s_sta_connected = false;

    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect fail: %s", esp_err_to_name(ret));
        s_sta_connecting = false;
        goto out;
    }
    ESP_LOGI(TAG, "STA connecting to SSID=%s", ssid);
    lvgl_ui_show_hint("Connecting WiFi...");
    ok = true;

out:
    wifi_api_unlock();
    return ok;
}

bool wifi_scan_networks(wifi_ap_record_t *records, uint16_t max_records, uint16_t *out_count)
{
    wifi_api_lock();
    bool ok = false;

    if (wifi_core_init() != ESP_OK) {
        goto out;
    }

    s_run_mode = WIFI_RUN_MODE_NONE;
    s_sta_connecting = false;
    s_sta_connected = false;
    httpd_stop_server();
    esp_wifi_stop();

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_mode STA for scan fail: %s", esp_err_to_name(ret));
        goto out;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "wifi start for scan fail: %s", esp_err_to_name(ret));
        goto out;
    }

    wifi_scan_config_t cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 30, .max = 120 },
    };

    ret = esp_wifi_scan_start(&cfg, true); // 阻塞等待
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "scan start fail: %s", esp_err_to_name(ret));
        goto out;
    }

    uint16_t number = max_records;
    ret = esp_wifi_scan_get_ap_records(&number, records);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "scan get records fail: %s", esp_err_to_name(ret));
        goto out;
    }
    if (out_count) {
        *out_count = number;
    }
    s_run_mode = WIFI_RUN_MODE_STA;
    ESP_LOGI(TAG, "scan done, found %u", (unsigned)number);
    ok = true;

out:
    wifi_api_unlock();
    return ok;
}

wifi_run_mode_t wifi_get_run_mode(void)
{
    return s_run_mode;
}

bool wifi_upload_is_busy(void)
{
    if (!s_upload_mu) return false;
    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    bool busy = upload_is_busy_state(s_upload.state);
    xSemaphoreGive(s_upload_mu);
    return busy;
}

bool wifi_ui_get_info(wifi_ui_info_t *out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    wifi_config_t cfg = {0};
    if (s_sta_connected && esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK) {
        size_t ssid_len = strnlen((const char *)cfg.sta.ssid, sizeof(cfg.sta.ssid));
        if (ssid_len >= sizeof(out->sta_ssid)) ssid_len = sizeof(out->sta_ssid) - 1;
        memcpy(out->sta_ssid, cfg.sta.ssid, ssid_len);
        out->sta_ssid[ssid_len] = '\0';
    }

    if (s_sta_connected && s_netif_sta) {
        esp_netif_ip_info_t ip_info = {0};
        if (esp_netif_get_ip_info(s_netif_sta, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            esp_ip4addr_ntoa(&ip_info.ip, out->sta_ip, sizeof(out->sta_ip));
        }
    }

    memset(&cfg, 0, sizeof(cfg));
    if (esp_wifi_get_config(WIFI_IF_AP, &cfg) == ESP_OK) {
        size_t ap_ssid_len = strnlen((const char *)cfg.ap.ssid, sizeof(cfg.ap.ssid));
        if (ap_ssid_len >= sizeof(out->ap_ssid)) ap_ssid_len = sizeof(out->ap_ssid) - 1;
        memcpy(out->ap_ssid, cfg.ap.ssid, ap_ssid_len);
        out->ap_ssid[ap_ssid_len] = '\0';

        size_t ap_pass_len = strnlen((const char *)cfg.ap.password, sizeof(cfg.ap.password));
        if (ap_pass_len >= sizeof(out->ap_password)) ap_pass_len = sizeof(out->ap_password) - 1;
        memcpy(out->ap_password, cfg.ap.password, ap_pass_len);
        out->ap_password[ap_pass_len] = '\0';
    } else {
        snprintf(out->ap_ssid, sizeof(out->ap_ssid), "%s", EXAMPLE_WIFI_AP_SSID);
        snprintf(out->ap_password, sizeof(out->ap_password), "%s", EXAMPLE_WIFI_AP_PASS);
    }

    if (s_netif_ap) {
        esp_netif_ip_info_t ip_info = {0};
        if (esp_netif_get_ip_info(s_netif_ap, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            esp_ip4addr_ntoa(&ip_info.ip, out->ap_ip, sizeof(out->ap_ip));
        }
    }

    return true;
}

// ===================== 事件处理 =====================
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    ESP_LOGD(TAG, "event: base=%s id=%" PRId32, base, id);
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_AP_START:
            s_run_mode = WIFI_RUN_MODE_AP;
            s_sta_connecting = false;
            s_sta_connected = false;
            httpd_start_server();
            lvgl_ui_show_hint("AP Ready");
            break;
        case WIFI_EVENT_AP_STOP:
            httpd_stop_server();
            break;
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
            ESP_LOGI(TAG, "STA connected: aid=%d, %02X:%02X:%02X:%02X:%02X:%02X",
                     e->aid, e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5]);
            lvgl_ui_show_hint("WiFi Connected");
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGI(TAG, "STA disconnected: aid=%d, %02X:%02X:%02X:%02X:%02X:%02X",
                     e->aid, e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5]);
            lvgl_ui_show_hint("WiFi Disconnected");
            break;
        }
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA start");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            s_sta_connecting = true;
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
            s_sta_connected = false;
            bool was_connecting = s_sta_connecting;
            s_sta_connecting = false;
            httpd_stop_server();

            char reason[48];
            snprintf(reason, sizeof(reason), "STA disconnected (%d)", e ? e->reason : -1);
            lvgl_ui_wifi_on_sta_disconnected(reason);
            if (s_run_mode == WIFI_RUN_MODE_STA && e && e->reason != WIFI_REASON_AUTH_FAIL) {
                /* 事件回调线程里不要长时间阻塞：尝试拿锁，失败则跳过本次自动重连 */
                wifi_api_lock_init();
                if (s_wifi_api_mu && xSemaphoreTake(s_wifi_api_mu, 0) == pdTRUE) {
                    esp_wifi_connect();
                    xSemaphoreGive(s_wifi_api_mu);
                    s_sta_connecting = true;
                }
            } else if (was_connecting) {
                lvgl_ui_show_hint("STA Disconnect");
            }
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        char ip[20] = {0};
        if (event) {
            esp_ip4addr_ntoa(&event->ip_info.ip, ip, sizeof(ip));
        }
        s_sta_connecting = false;
        s_sta_connected = true;
        wifi_saved_password_store_current_sta();
        httpd_start_server();
        lvgl_ui_wifi_on_sta_connected(ip);
        lvgl_ui_show_hint_ms("STA Connected", 1000);
        ESP_LOGI(TAG, "STA got IP: %s", ip);
    }
}

// ===================== 图传控制 =====================
static bool wifi_send_frame_internal(const camera_fb_t *fb)
{
    if (!ws_ready() || !fb) {
        return false;
    }
    if (s_ws_job_pending) {
        return false;
    }
    const size_t MAX_WS_FRAME = 200 * 1024;

    uint8_t *payload = NULL;
    size_t payload_len = 0;

    if (fb->format == PIXFORMAT_JPEG) {
        if (fb->len > MAX_WS_FRAME) {
            ESP_LOGW(TAG, "ws: jpeg frame too big len=%u drop", (unsigned)fb->len);
            return false;
        }
        payload = (uint8_t *)malloc(fb->len);
        if (!payload) {
            ESP_LOGW(TAG, "ws: jpeg alloc fail len=%u", (unsigned)fb->len);
            return false;
        }
        memcpy(payload, fb->buf, fb->len);
        payload_len = fb->len;
    } else {
        uint8_t *out = NULL; size_t out_len = 0;
        /* STA 模式下适当降低 JPEG 质量以提升吞吐；AP 模式保持清晰度 */
        const int quality = (s_run_mode == WIFI_RUN_MODE_STA) ? 30 : 16;
        if (!frame2jpg((camera_fb_t *)fb, quality, &out, &out_len) || !out) {
            ESP_LOGW(TAG, "ws: frame2jpg fail fmt=%d", fb->format);
            return false;
        }
        if (out_len > MAX_WS_FRAME) {
            free(out);
            ESP_LOGW(TAG, "ws: frame too big len=%u drop", (unsigned)out_len);
            return false;
        }
        payload = out;
        payload_len = out_len;
    }

    return ws_send_binary_take(payload, payload_len);
}

bool wifi_stream_has_client(void)
{
    return ws_ready();
}

void wifi_stream_update_frame_queue(QueueHandle_t frame_q)
{
    xQueueFrameIn = frame_q;
}

bool wifi_stream_start_continuous(void)
{
    s_streaming_on = true;
    return true;
}

void wifi_stream_stop_continuous(void)
{
    s_streaming_on = false;
}

bool wifi_stream_is_running(void)
{
    return s_streaming_on;
}

void wifi_stream_request_single_shot(void)
{
    if (!ws_ready()) {
        s_single_shot_pending = false;
        return;
    }
    s_single_shot_pending = true;
}

bool wifi_stream_consume_single_shot(void)
{
    bool pending = s_single_shot_pending && ws_ready();
    s_single_shot_pending = false;
    return pending;
}

bool wifi_stream_send_single(const camera_fb_t *fb)
{
    return wifi_stream_send_single_jpeg(fb);
}

bool wifi_stream_push_if_running(const camera_fb_t *fb)
{
    if (!s_streaming_on) {
        return false;
    }
    return wifi_send_frame_internal(fb);
}

void register_wifi(QueueHandle_t frame_q)
{
    xQueueFrameIn = frame_q;
    if (!s_wifi_started) {
        if (!wifi_start_ap_mode()) {
            ESP_LOGE(TAG, "start AP failed");
            return;
        }
    } else if (!s_httpd) {
        httpd_start_server();
    }
}

static bool wifi_send_frame_rgb565(const camera_fb_t *fb)
{
    if (!ws_ready() || !fb) {
        return false;
    }
    if (s_ws_job_pending) {
        return false;
    }
    const uint16_t w = fb->width;
    const uint16_t h = fb->height;
    if (fb->format != PIXFORMAT_RGB565 || !fb->buf || w == 0 || h == 0) {
        ESP_LOGW(TAG, "ws: rgb565 single requires RGB565, fmt=%d", fb->format);
        return false;
    }
    const size_t payload_len = (size_t)w * h * 2;
    const size_t MAX_WS_FRAME = 200 * 1024;
    if (payload_len + 5 > MAX_WS_FRAME) {
        ESP_LOGW(TAG, "ws: rgb565 frame too big total=%u drop", (unsigned)(payload_len + 5));
        return false;
    }

    uint8_t *buf = (uint8_t *)heap_caps_malloc(payload_len + 5, MALLOC_CAP_8BIT);
    if (!buf) {
        return false;
    }
    buf[0] = 'R';
    buf[1] = (uint8_t)(w >> 8);
    buf[2] = (uint8_t)(w & 0xFF);
    buf[3] = (uint8_t)(h >> 8);
    buf[4] = (uint8_t)(h & 0xFF);
    memcpy(buf + 5, fb->buf, payload_len);

    return ws_send_binary_take(buf, payload_len + 5);
}

bool wifi_stream_send_single_rgb565(const camera_fb_t *fb)
{
    return wifi_send_frame_rgb565(fb);
}

bool wifi_stream_send_single_jpeg(const camera_fb_t *fb)
{
    return wifi_send_frame_internal(fb);
}

static void wifi_upload_resume_timer_cb(void *arg)
{
    (void)arg;
    TaskScheduling_ResumeAfterSettings();
}

static void wifi_upload_schedule_resume_delayed(uint32_t delay_ms)
{
    if (!s_upload_resume_timer) return;
    esp_timer_stop(s_upload_resume_timer);
    esp_timer_start_once(s_upload_resume_timer, (uint64_t)delay_ms * 1000ULL);
}

static void wifi_upload_abort_and_cleanup(const char *reason)
{
    if (!s_upload_mu) return;
    char rid[40] = {0};
    bool need_finish = false;
    xSemaphoreTake(s_upload_mu, portMAX_DELAY);
    if (upload_is_busy_state(s_upload.state)) {
        snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
        if (reason && reason[0] != '\0') {
            snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", reason);
        }
        s_upload_abort_requested = true;
        if (s_upload.state == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD ||
            s_upload.state == WIFI_UPLOAD_STATE_PENDING_CONFIRM) {
            s_upload.state = WIFI_UPLOAD_STATE_FAILED;
            need_finish = true;
        } else if (s_upload.state == WIFI_UPLOAD_STATE_UPLOADING) {
            need_finish = true;
        }
    }
    xSemaphoreGive(s_upload_mu);

    if (need_finish && rid[0] != '\0') {
                    lvgl_ui_upload_finish(rid, false, "Disconnected");
        wifi_upload_schedule_resume_delayed(1500);
    }
}

bool wifi_upload_user_decide(const char *request_id, wifi_upload_decision_t decision)
{
    if (!request_id || request_id[0] == '\0' || !s_upload_evt_q) return false;
    wifi_upload_evt_t evt = {0};
    evt.decision = decision;
    snprintf(evt.request_id, sizeof(evt.request_id), "%s", request_id);
    return xQueueSend(s_upload_evt_q, &evt, 0) == pdPASS;
}

static void wifi_upload_task(void *arg)
{
    (void)arg;

    for (;;) {
        wifi_upload_evt_t evt = {0};
        if (!xQueueReceive(s_upload_evt_q, &evt, pdMS_TO_TICKS(200))) {
            if (s_upload_mu) {
                char rid[40] = {0};
                bool expire = false;
                xSemaphoreTake(s_upload_mu, portMAX_DELAY);
                if (s_upload.request_id[0] != '\0' && s_upload.state == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD) {
                    const int64_t elapsed_ms = (now_us() - s_upload.accepted_ts_us) / 1000;
                    if (elapsed_ms > WIFI_UPLOAD_ACCEPT_WAIT_MS) {
                        s_upload.state = WIFI_UPLOAD_STATE_EXPIRED;
                        snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "upload_timeout");
                        snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
                        expire = true;
                    }
                }
                xSemaphoreGive(s_upload_mu);
                if (expire && rid[0]) {
                    lvgl_ui_upload_finish(rid, false, "Upload timeout");
                    wifi_upload_schedule_resume_delayed(1500);
                }
            }
            continue;
        }

        if (!evt.request_id[0]) {
            continue;
        }

        if (!s_upload_mu) {
            continue;
        }

        if (evt.decision == WIFI_UPLOAD_DECISION_CANCEL) {
            char rid[40] = {0};
            bool need_finish = false;
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            if (s_upload.request_id[0] != '\0' && strcmp(evt.request_id, s_upload.request_id) == 0) {
                snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
                if (s_upload.state == WIFI_UPLOAD_STATE_UPLOADING) {
                    s_upload_abort_requested = true;
                    snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "user_cancel");
                } else if (s_upload.state == WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD ||
                           s_upload.state == WIFI_UPLOAD_STATE_PENDING_CONFIRM) {
                    s_upload.state = WIFI_UPLOAD_STATE_FAILED;
                    snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", "user_cancel");
                    need_finish = true;
                }
            }
            xSemaphoreGive(s_upload_mu);
            if (need_finish) {
                lvgl_ui_upload_finish(rid, false, "Canceled");
                wifi_upload_schedule_resume_delayed(1500);
            }
            continue;
        }

        if (evt.decision == WIFI_UPLOAD_DECISION_REJECT || evt.decision == WIFI_UPLOAD_DECISION_TIMEOUT) {
            char rid[40] = {0};
            bool need_resume = false;
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            if (s_upload.request_id[0] != '\0' && strcmp(evt.request_id, s_upload.request_id) == 0 &&
                s_upload.state == WIFI_UPLOAD_STATE_PENDING_CONFIRM) {
                snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
                s_upload.state = (evt.decision == WIFI_UPLOAD_DECISION_TIMEOUT) ?
                                 WIFI_UPLOAD_STATE_EXPIRED : WIFI_UPLOAD_STATE_REJECTED;
                snprintf(s_upload.reason, sizeof(s_upload.reason), "%s",
                         (evt.decision == WIFI_UPLOAD_DECISION_TIMEOUT) ? "timeout" : "user_reject");
                need_resume = true;
            }
            xSemaphoreGive(s_upload_mu);
            if (need_resume) {
                wifi_upload_schedule_resume_delayed(200);
            }
            continue;
        }

        if (evt.decision == WIFI_UPLOAD_DECISION_ACCEPT) {
            char rid[40] = {0};
            char type[16] = {0};
            uint32_t need = 0;
            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            if (s_upload.request_id[0] == '\0' ||
                strcmp(evt.request_id, s_upload.request_id) != 0 ||
                s_upload.state != WIFI_UPLOAD_STATE_PENDING_CONFIRM) {
                xSemaphoreGive(s_upload_mu);
                continue;
            }
            snprintf(rid, sizeof(rid), "%s", s_upload.request_id);
            snprintf(type, sizeof(type), "%s", s_upload.type);
            need = s_upload.total_size;
            xSemaphoreGive(s_upload_mu);

            char why[48] = {0};
            bool ok = false;
            if (str_eq_casei(type, "espdl")) {
                ok = sd_models_precheck(need, why, sizeof(why));
            } else if (str_eq_casei(type, "app")) {
                ok = ota_partition_precheck(need, why, sizeof(why));
            } else {
                snprintf(why, sizeof(why), "%s", "unsupported_type");
                ok = false;
            }

            xSemaphoreTake(s_upload_mu, portMAX_DELAY);
            if (s_upload.request_id[0] == '\0' ||
                strcmp(rid, s_upload.request_id) != 0 ||
                s_upload.state != WIFI_UPLOAD_STATE_PENDING_CONFIRM) {
                xSemaphoreGive(s_upload_mu);
                continue;
            }

            if (!ok) {
                s_upload.state = WIFI_UPLOAD_STATE_REJECTED;
                snprintf(s_upload.reason, sizeof(s_upload.reason), "%s", why[0] ? why : "precheck_fail");
                xSemaphoreGive(s_upload_mu);

                if (str_eq_casei(type, "espdl")) {
                    if (strcmp(why, "no_sd") == 0) {
                        lvgl_ui_upload_finish(rid, false, "No SD card");
                    } else if (strcmp(why, "read_only") == 0) {
                        lvgl_ui_upload_finish(rid, false, "SD read-only");
                    } else if (strcmp(why, "no_space") == 0) {
                        lvgl_ui_upload_finish(rid, false, "SD no space");
                    } else {
                        lvgl_ui_upload_finish(rid, false, "SD check failed");
                    }
                } else if (str_eq_casei(type, "app")) {
                    if (strcmp(why, "no_ota_partition") == 0) {
                        lvgl_ui_upload_finish(rid, false, "No OTA partition");
                    } else if (strcmp(why, "too_large") == 0) {
                        lvgl_ui_upload_finish(rid, false, "Firmware too large");
                    } else {
                        lvgl_ui_upload_finish(rid, false, "OTA check failed");
                    }
                } else {
                    lvgl_ui_upload_finish(rid, false, "Unsupported type");
                }
                wifi_upload_schedule_resume_delayed(1500);
                continue;
            }

            random_hex(s_upload.token, sizeof(s_upload.token), 8);
            s_upload.accepted_ts_us = now_us();
            s_upload.state = WIFI_UPLOAD_STATE_ACCEPTED_WAIT_UPLOAD;
            s_upload.reason[0] = '\0';
            s_upload_abort_requested = false;
            xSemaphoreGive(s_upload_mu);

            lvgl_ui_upload_set_waiting(rid);
        }
    }
}
