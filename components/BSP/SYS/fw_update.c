#include "fw_update.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"

#include "TaskScheduling.h"
#include "cJSON.h"
#include "device_identity.h"
#include "lvgl_ui.h"
#include "wifi.h"

#define FW_UPDATE_SERVER_BASE_URL "http://192.168.5.29"
#define FW_UPDATE_MANIFEST_PATH   "/ota/manifest.json"

#define FW_UPDATE_MANIFEST_MAX_BYTES (8 * 1024)
#define FW_UPDATE_VER_MAX_LEN        31
#define FW_UPDATE_URL_MAX_LEN        255
#define FW_UPDATE_SHA256_MAX_LEN     64

static const char *TAG = "fw_update";

typedef enum {
    FW_UPDATE_STATE_IDLE = 0,
    FW_UPDATE_STATE_CHECKING,
    FW_UPDATE_STATE_AVAILABLE,
    FW_UPDATE_STATE_DOWNLOADING,
} fw_update_state_t;

typedef struct {
    int year;
    int month;
    int major;
    int minor;
} fw_version_t;

typedef struct {
    char latest_version[FW_UPDATE_VER_MAX_LEN + 1];
    char url[FW_UPDATE_URL_MAX_LEN + 1];
    char sha256_hex[FW_UPDATE_SHA256_MAX_LEN + 1];
    uint32_t size_bytes;
} fw_update_latest_t;

typedef struct {
    char current_ver[FW_UPDATE_VER_MAX_LEN + 1];
    fw_update_latest_t latest;
} fw_update_prompt_t;

typedef struct {
    fw_update_latest_t latest;
} fw_update_job_t;

static SemaphoreHandle_t s_mu = NULL;
static bool s_auto_checked_once = false;
static bool s_prompt_shown = false;
static fw_update_state_t s_state = FW_UPDATE_STATE_IDLE;
static fw_update_latest_t s_latest = {0};

static void fw_update_lock_init(void)
{
    if (!s_mu) {
        s_mu = xSemaphoreCreateMutex();
    }
}

static void fw_update_lock(void)
{
    fw_update_lock_init();
    if (s_mu) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
}

static void fw_update_unlock(void)
{
    if (s_mu) {
        xSemaphoreGive(s_mu);
    }
}

static bool is_hex64(const char *s)
{
    if (!s) return false;
    size_t n = strlen(s);
    if (n != 64) return false;
    for (size_t i = 0; i < n; ++i) {
        char c = s[i];
        bool ok = (c >= '0' && c <= '9') ||
                  (c >= 'a' && c <= 'f') ||
                  (c >= 'A' && c <= 'F');
        if (!ok) return false;
    }
    return true;
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

static bool wifi_sta_ready(void)
{
    if (wifi_get_run_mode() != WIFI_RUN_MODE_STA) {
        return false;
    }
    wifi_ui_info_t info;
    if (!wifi_ui_get_info(&info)) {
        return false;
    }
    return info.sta_ip[0] != '\0';
}

static bool fw_version_parse(const char *s, fw_version_t *out)
{
    if (!s || !out) return false;
    size_t len = strlen(s);
    if (len < 10) return false;

    /* YYYY-MM-Vm.n */
    if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1]) ||
        !isdigit((unsigned char)s[2]) || !isdigit((unsigned char)s[3]) ||
        s[4] != '-' ||
        !isdigit((unsigned char)s[5]) || !isdigit((unsigned char)s[6]) ||
        s[7] != '-' ||
        !(s[8] == 'V' || s[8] == 'v')) {
        return false;
    }

    int year = (s[0] - '0') * 1000 + (s[1] - '0') * 100 + (s[2] - '0') * 10 + (s[3] - '0');
    int month = (s[5] - '0') * 10 + (s[6] - '0');
    if (month < 1 || month > 12) return false;

    const char *p = s + 9;
    if (!isdigit((unsigned char)*p)) return false;

    int major = 0;
    while (isdigit((unsigned char)*p)) {
        major = major * 10 + (*p - '0');
        p++;
        if (major > 999) return false;
    }

    int minor = 0;
    if (*p == '.') {
        p++;
        if (!isdigit((unsigned char)*p)) return false;
        while (isdigit((unsigned char)*p)) {
            minor = minor * 10 + (*p - '0');
            p++;
            if (minor > 999) return false;
        }
    }

    if (*p != '\0') {
        return false;
    }

    out->year = year;
    out->month = month;
    out->major = major;
    out->minor = minor;
    return true;
}

static int fw_version_cmp(const fw_version_t *a, const fw_version_t *b)
{
    if (a->year != b->year) return (a->year < b->year) ? -1 : 1;
    if (a->month != b->month) return (a->month < b->month) ? -1 : 1;
    if (a->major != b->major) return (a->major < b->major) ? -1 : 1;
    if (a->minor != b->minor) return (a->minor < b->minor) ? -1 : 1;
    return 0;
}

static bool url_is_absolute_http(const char *url)
{
    return url && (strncmp(url, "http://", 7) == 0 || strncmp(url, "https://", 8) == 0);
}

static bool join_url(char *out, size_t out_len, const char *base, const char *path_or_url)
{
    if (!out || out_len == 0) return false;
    out[0] = '\0';
    if (!base || !path_or_url) return false;

    if (url_is_absolute_http(path_or_url)) {
        size_t n = strlen(path_or_url);
        if (n >= out_len) return false;
        memcpy(out, path_or_url, n + 1);
        return true;
    }

    /* relative: base + '/' + path */
    size_t bl = strlen(base);
    size_t pl = strlen(path_or_url);
    if (bl == 0 || pl == 0) return false;

    bool base_has_slash = base[bl - 1] == '/';
    bool path_has_slash = path_or_url[0] == '/';

    size_t needed = bl + pl + 2;
    if (needed > out_len) return false;

    if (base_has_slash && path_has_slash) {
        snprintf(out, out_len, "%.*s%s", (int)(bl - 1), base, path_or_url);
    } else if (!base_has_slash && !path_has_slash) {
        snprintf(out, out_len, "%s/%s", base, path_or_url);
    } else {
        snprintf(out, out_len, "%s%s", base, path_or_url);
    }
    return true;
}

static esp_err_t http_get_to_buf(const char *url, char **out_body, size_t *out_len)
{
    if (!url || !out_body) return ESP_ERR_INVALID_ARG;
    *out_body = NULL;
    if (out_len) *out_len = 0;

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 5000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Accept", "application/json");

    char sn[32] = {0};
    if (device_identity_get_sn(sn, sizeof(sn)) == ESP_OK && sn[0]) {
        esp_http_client_set_header(client, "X-Device-SN", sn);
    }
    char mac[32] = {0};
    if (device_identity_get_base_mac_str(mac, sizeof(mac)) == ESP_OK && mac[0]) {
        esp_http_client_set_header(client, "X-Device-MAC", mac);
    }
    const char *ver = device_identity_get_fw_version();
    if (ver && ver[0]) {
        esp_http_client_set_header(client, "X-Device-Ver", ver);
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return err;
    }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status < 200 || status >= 300) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int content_len = esp_http_client_get_content_length(client);
    if (content_len > (int)FW_UPDATE_MANIFEST_MAX_BYTES) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    size_t cap = (content_len > 0) ? (size_t)content_len : (size_t)FW_UPDATE_MANIFEST_MAX_BYTES;
    char *buf = (char *)malloc(cap + 1);
    if (!buf) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    size_t pos = 0;
    while (pos < cap) {
        int r = esp_http_client_read(client, buf + pos, (int)(cap - pos));
        if (r < 0) {
            free(buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
        if (r == 0) {
            break;
        }
        pos += (size_t)r;
    }
    buf[pos] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    *out_body = buf;
    if (out_len) *out_len = pos;
    return ESP_OK;
}

static bool parse_manifest_latest(const char *json, fw_update_latest_t *out_latest)
{
    if (!json || !out_latest) return false;
    memset(out_latest, 0, sizeof(*out_latest));

    cJSON *root = cJSON_Parse(json);
    if (!root) return false;

    const cJSON *latest = cJSON_GetObjectItemCaseSensitive(root, "latest");
    const cJSON *obj = (latest && cJSON_IsObject(latest)) ? latest : root;

    const cJSON *j_ver = cJSON_GetObjectItemCaseSensitive(obj, "version");
    const cJSON *j_url = cJSON_GetObjectItemCaseSensitive(obj, "url");
    const cJSON *j_sha = cJSON_GetObjectItemCaseSensitive(obj, "sha256");
    const cJSON *j_size = cJSON_GetObjectItemCaseSensitive(obj, "size");

    if (!cJSON_IsString(j_ver) || !j_ver->valuestring ||
        !cJSON_IsString(j_url) || !j_url->valuestring ||
        !cJSON_IsString(j_sha) || !j_sha->valuestring) {
        cJSON_Delete(root);
        return false;
    }

    if (!is_hex64(j_sha->valuestring)) {
        cJSON_Delete(root);
        return false;
    }

    fw_version_t tmp;
    if (!fw_version_parse(j_ver->valuestring, &tmp)) {
        cJSON_Delete(root);
        return false;
    }

    snprintf(out_latest->latest_version, sizeof(out_latest->latest_version), "%s", j_ver->valuestring);
    snprintf(out_latest->sha256_hex, sizeof(out_latest->sha256_hex), "%s", j_sha->valuestring);

    char full_url[FW_UPDATE_URL_MAX_LEN + 1];
    if (!join_url(full_url, sizeof(full_url), FW_UPDATE_SERVER_BASE_URL, j_url->valuestring)) {
        cJSON_Delete(root);
        return false;
    }
    snprintf(out_latest->url, sizeof(out_latest->url), "%s", full_url);

    if (j_size && cJSON_IsNumber(j_size) && j_size->valuedouble > 0) {
        out_latest->size_bytes = (uint32_t)j_size->valuedouble;
    }

    cJSON_Delete(root);
    return true;
}

static bool ota_precheck(uint32_t total_size, char *why, size_t why_len)
{
    if (why && why_len) why[0] = '\0';

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        if (why && why_len) snprintf(why, why_len, "no_ota_partition");
        return false;
    }
    if (total_size > 0 && total_size > update->size) {
        if (why && why_len) snprintf(why, why_len, "too_large");
        return false;
    }
    return true;
}

static void fw_update_try_show_prompt_unlocked(void)
{
    if (s_state != FW_UPDATE_STATE_AVAILABLE) return;
    if (s_prompt_shown) return;
    if (!lvgl_ui_is_main_screen_active()) return;
    if (wifi_upload_is_busy()) return;

    fw_update_prompt_t p = {0};
    const char *cur = device_identity_get_fw_version();
    snprintf(p.current_ver, sizeof(p.current_ver), "%s", (cur && cur[0]) ? cur : "-");
    p.latest = s_latest;

    s_prompt_shown = true;
    lvgl_ui_fw_update_show_prompt(p.current_ver, p.latest.latest_version);
}

static void fw_update_check_task(void *arg)
{
    (void)arg;

    if (!wifi_sta_ready()) {
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    const char *cur = device_identity_get_fw_version();
    fw_version_t curv;
    if (!fw_version_parse(cur, &curv)) {
        ESP_LOGW(TAG, "current version invalid: %s", cur ? cur : "");
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    char manifest_url[FW_UPDATE_URL_MAX_LEN + 1];
    if (!join_url(manifest_url, sizeof(manifest_url), FW_UPDATE_SERVER_BASE_URL, FW_UPDATE_MANIFEST_PATH)) {
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    char *body = NULL;
    esp_err_t err = http_get_to_buf(manifest_url, &body, NULL);
    if (err != ESP_OK || !body) {
        ESP_LOGW(TAG, "manifest fetch failed: %s", esp_err_to_name(err));
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    fw_update_latest_t latest;
    bool ok = parse_manifest_latest(body, &latest);
    free(body);
    if (!ok) {
        ESP_LOGW(TAG, "manifest parse failed");
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    fw_version_t newv;
    if (!fw_version_parse(latest.latest_version, &newv)) {
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    int cmp = fw_version_cmp(&curv, &newv);
    if (cmp >= 0) {
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    fw_update_lock();
    s_latest = latest;
    s_state = FW_UPDATE_STATE_AVAILABLE;
    fw_update_try_show_prompt_unlocked();
    fw_update_unlock();

    vTaskDelete(NULL);
}

static void sha256_to_hex(const uint8_t in[32], char out_hex[65])
{
    static const char *hex = "0123456789abcdef";
    for (int i = 0; i < 32; ++i) {
        out_hex[i * 2] = hex[(in[i] >> 4) & 0xF];
        out_hex[i * 2 + 1] = hex[in[i] & 0xF];
    }
    out_hex[64] = '\0';
}

static void fw_update_download_task(void *arg)
{
    fw_update_job_t *job = (fw_update_job_t *)arg;
    if (!job) {
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        vTaskDelete(NULL);
        return;
    }

    if (!wifi_sta_ready()) {
        lvgl_ui_fw_update_finish(false, "No STA");
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }
    if (wifi_upload_is_busy()) {
        lvgl_ui_fw_update_finish(false, "Busy");
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    TaskScheduling_PauseForSettings();
    wifi_stream_stop_continuous();

    lvgl_ui_fw_update_set_progress(0, "检查分区...");

    esp_http_client_config_t cfg = {
        .url = job->latest.url,
        .timeout_ms = 8000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        lvgl_ui_fw_update_finish(false, "No mem");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);

    esp_err_t r = esp_http_client_open(client, 0);
    if (r != ESP_OK) {
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, "HTTP open fail");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status < 200 || status >= 300) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, "HTTP status");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    int content_len = esp_http_client_get_content_length(client);
    uint32_t total = (content_len > 0) ? (uint32_t)content_len : job->latest.size_bytes;
    char why[48] = {0};
    if (!ota_precheck(total, why, sizeof(why))) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, str_eq_casei(why, "too_large") ? "Firmware too large" : "No OTA partition");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, "No OTA partition");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    esp_ota_handle_t ota = 0;
    r = esp_ota_begin(update, (total > 0) ? total : OTA_SIZE_UNKNOWN, &ota);
    if (r != ESP_OK) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, "OTA begin fail");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    bool sha_ok = mbedtls_sha256_starts(&sha, 0) == 0;

    uint8_t *buf = (uint8_t *)malloc(4096);
    if (!buf) {
        if (sha_ok) mbedtls_sha256_free(&sha);
        esp_ota_abort(ota);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        lvgl_ui_fw_update_finish(false, "No mem");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    uint32_t received = 0;
    int last_percent = -1;

    for (;;) {
        int n = esp_http_client_read(client, (char *)buf, 4096);
        if (n < 0) {
            r = ESP_FAIL;
            break;
        }
        if (n == 0) {
            r = ESP_OK;
            break;
        }

        if (sha_ok) {
            if (mbedtls_sha256_update(&sha, buf, (size_t)n) != 0) {
                sha_ok = false;
            }
        }

        r = esp_ota_write(ota, buf, (size_t)n);
        if (r != ESP_OK) {
            break;
        }

        received += (uint32_t)n;
        if (total > 0) {
            int percent = (int)((received * 100u) / total);
            if (percent > 100) percent = 100;
            if (percent != last_percent) {
                last_percent = percent;
                char msg[64];
                snprintf(msg, sizeof(msg), "Downloading... %d%%", percent);
                lvgl_ui_fw_update_set_progress(percent, msg);
            }
        }
    }

    free(buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (r != ESP_OK) {
        if (sha_ok) mbedtls_sha256_free(&sha);
        esp_ota_abort(ota);
        lvgl_ui_fw_update_finish(false, "Download failed");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    uint8_t sha_out[32] = {0};
    if (!sha_ok || mbedtls_sha256_finish(&sha, sha_out) != 0) {
        mbedtls_sha256_free(&sha);
        esp_ota_abort(ota);
        lvgl_ui_fw_update_finish(false, "SHA256 error");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }
    mbedtls_sha256_free(&sha);

    char sha_hex[65];
    sha256_to_hex(sha_out, sha_hex);
    if (!str_eq_casei(sha_hex, job->latest.sha256_hex)) {
        esp_ota_abort(ota);
        lvgl_ui_fw_update_finish(false, "SHA256 mismatch");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    r = esp_ota_end(ota);
    if (r != ESP_OK) {
        lvgl_ui_fw_update_finish(false, "OTA end fail");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    r = esp_ota_set_boot_partition(update);
    if (r != ESP_OK) {
        lvgl_ui_fw_update_finish(false, "Set boot fail");
        TaskScheduling_ResumeAfterSettings();
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        free(job);
        vTaskDelete(NULL);
        return;
    }

    /* 写入显示用版本标签（与“关于”页一致） */
    (void)device_identity_set_fw_version_label(job->latest.latest_version);

    lvgl_ui_fw_update_set_progress(100, "升级完成，重启中...");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

bool fw_update_is_busy(void)
{
    fw_update_lock();
    bool busy = (s_state == FW_UPDATE_STATE_DOWNLOADING);
    fw_update_unlock();
    return busy;
}

void fw_update_notify_main_screen_entered(void)
{
    fw_update_lock();

    if (s_state == FW_UPDATE_STATE_AVAILABLE) {
        fw_update_try_show_prompt_unlocked();
        fw_update_unlock();
        return;
    }

    if (s_auto_checked_once || s_state != FW_UPDATE_STATE_IDLE) {
        fw_update_unlock();
        return;
    }

    if (!wifi_sta_ready()) {
        fw_update_unlock();
        return;
    }
    if (wifi_upload_is_busy()) {
        fw_update_unlock();
        return;
    }

    s_state = FW_UPDATE_STATE_CHECKING;
    s_auto_checked_once = true;
    fw_update_unlock();

    (void)xTaskCreate(fw_update_check_task, "fw_update_chk", 6144, NULL, 3, NULL);
}

void fw_update_user_decide(bool accept_upgrade)
{
    fw_update_job_t *job = NULL;

    fw_update_lock();
    if (!accept_upgrade) {
        s_state = FW_UPDATE_STATE_IDLE;
        s_prompt_shown = false;
        memset(&s_latest, 0, sizeof(s_latest));
        fw_update_unlock();
        return;
    }

    if (s_state != FW_UPDATE_STATE_AVAILABLE) {
        fw_update_unlock();
        return;
    }

    job = (fw_update_job_t *)malloc(sizeof(fw_update_job_t));
    if (job) {
        memset(job, 0, sizeof(*job));
        job->latest = s_latest;
    }

    s_state = FW_UPDATE_STATE_DOWNLOADING;
    fw_update_unlock();

    if (!job) {
        lvgl_ui_fw_update_finish(false, "No mem");
        fw_update_lock();
        s_state = FW_UPDATE_STATE_IDLE;
        fw_update_unlock();
        return;
    }

    (void)xTaskCreate(fw_update_download_task, "fw_update_dl", 8192, job, 4, NULL);
}
