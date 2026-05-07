#include "camera.h"
#include <stdbool.h>

static const char *TAG = "CAM";

static pixformat_t   s_fmt;
static framesize_t   s_size;
static uint8_t       s_fb_count;
static QueueHandle_t xQueueFrameOut = NULL;
static TaskHandle_t  s_camera_task_handle = NULL;
static bool          s_camera_started = false;


static int           s_xclk_freq = XCLK_FREQ;
static const int     s_xclk_fallback_hz = 16000000;
static const uint8_t s_fb_count_fallback = 2;

void camera_set_xclk(int xclk_hz)
{
    if (xclk_hz <= 0) {
        s_xclk_freq = XCLK_FREQ;
    } else {
        s_xclk_freq = xclk_hz;
    }
}

void camera_init(const pixformat_t pixel_fromat,
    const framesize_t frame_size,
    const uint8_t fb_count)
{
    s_camera_started = false;
    camera_config_t config = {0};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;

    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_sscb_sda = CAM_PIN_SIOD;
    config.pin_sscb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = s_xclk_freq;

    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;

    // 稍降质量以减小体积、降低延时（数值越小质量越高、体积越大）
    config.jpeg_quality = 18;
    config.fb_count = fb_count;
    // 避免阻塞积压，优先取最新帧，减轻下游抖动
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    s_camera_started = true;

    sensor_t *s = esp_camera_sensor_get();
    
    s->set_whitebal(s, 1);         // 开启自动白平衡 AWB
    s->set_awb_gain(s, 1);    
    s->set_vflip(s, 1); // flip it back
    s->set_hmirror(s,1); //镜像翻转
}

void camera_reinit(camera_fb_t* frame)
{
    if(frame)
    {
        esp_camera_fb_return(frame);
    }
    esp_camera_deinit();
    s_camera_started = false;
    camera_init(s_fmt, s_size, s_fb_count);
}

static void camera_process_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            printf("进入camera_process_handler出了问题\n");
        }
        if (frame){
            if (xQueueSend(xQueueFrameOut, &frame, 0) != pdTRUE) {
                /* Queue full: drop the oldest frame to keep camera running */
                esp_camera_fb_return(frame);
                frame = NULL;
            }
        }
        else{
            ESP_LOGI(TAG, "Camera capture failed !!! Start reinit...");
            camera_reinit(frame);
        }
    }
}


void register_camera(const pixformat_t pixel_fromat,
    const framesize_t frame_size,
    const uint8_t fb_count,
    const QueueHandle_t frame_out)
{
    if (s_camera_task_handle || s_camera_started) {
        camera_stop();
    }
    s_fmt = pixel_fromat;
    s_size = frame_size;
    s_fb_count = fb_count;
    xQueueFrameOut = frame_out;

    /* 首次上电/快速切换时摄像头可能尚未稳定：
     * - 现象：开机很快进入主界面点“开启”容易失败；点一次“扫描”相当于人为等待一会儿后再开启就能成功
     * - 目标：无需依赖 UI 的“扫描”制造延时，保证直接进入也能稳定启动
     */
    const int64_t up_us = esp_timer_get_time();
    const int64_t min_ready_us = 3500LL * 1000LL;
    if (up_us >= 0 && up_us < min_ready_us) {
        uint32_t wait_ms = (uint32_t)((min_ready_us - up_us + 999) / 1000);
        if (wait_ms > 0) {
            ESP_LOGI(TAG, "camera warmup delay %ums", (unsigned)wait_ms);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }

    const int max_retry = 20;
    bool used_xclk_fallback = false;
    bool used_fb_fallback = false;
    for (int i = 0; i < max_retry; ++i) {
        camera_init(s_fmt, s_size, s_fb_count);
        if (s_camera_started) {
            break;
        }
        ESP_LOGW(TAG, "camera init failed, retry %d/%d", i + 1, max_retry);
        (void)esp_camera_deinit();

        if (!used_xclk_fallback && s_xclk_freq > s_xclk_fallback_hz) {
            s_xclk_freq = s_xclk_fallback_hz;
            used_xclk_fallback = true;
            ESP_LOGW(TAG, "fallback camera xclk=%dHz to reduce DMA pressure", s_xclk_freq);
            continue;
        }

        if (!used_fb_fallback && s_fb_count > s_fb_count_fallback) {
            s_fb_count = s_fb_count_fallback;
            used_fb_fallback = true;
            ESP_LOGW(TAG, "fallback camera fb_count=%u", (unsigned)s_fb_count);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!s_camera_started) {
        ESP_LOGE(TAG, "camera init failed after retries");
        return;
    }

    if (used_xclk_fallback || used_fb_fallback) {
        ESP_LOGW(TAG, "camera fallback applied: xclk=%d fb_count=%u",
                 s_xclk_freq, (unsigned)s_fb_count);
    }
    /* 降低优先级，避免摄像头采集长期占用 CPU，影响 UI 按键 */
    s_camera_task_handle = NULL;
    const int max_task_retry = 3;
    for (int i = 0; i < max_task_retry; ++i) {
        BaseType_t ok = xTaskCreatePinnedToCore(camera_process_handler, TAG, 4 * 1024, NULL, 3, &s_camera_task_handle, 1);
        if (ok == pdPASS && s_camera_task_handle) {
            break;
        }
        s_camera_task_handle = NULL;
        ESP_LOGW(TAG, "create camera task failed, retry %d/%d", i + 1, max_task_retry);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_camera_task_handle) {
        ESP_LOGE(TAG, "create camera task failed after retries");
    }
}

void camera_stop(void)
{
    if (s_camera_task_handle) {
        vTaskDelete(s_camera_task_handle);
        s_camera_task_handle = NULL;
    }
    if (s_camera_started) {
        esp_camera_deinit();
        s_camera_started = false;
    }
    xQueueFrameOut = NULL;
}

bool camera_task_is_created(void)
{
    return s_camera_task_handle != NULL;
}
