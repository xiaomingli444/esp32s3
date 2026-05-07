#include "TaskScheduling.h"
#include "find_color.h"
#include "color_detection.h"
#include "apriltag_recognition.h"
#include "line_detection.h"
#include "AIdetection.h"
#include "sd.h"
#include "wifi.h"
#include "camera.h"
#include "lcd.h"
#include <esp_err.h>
#include <sys/stat.h>
#include <cstdio>
#include <atomic>
#include "freertos/semphr.h"

static const char *TAG = "TaskScheduling";

static QueueHandle_t s_cam_frame_q  = nullptr;
static QueueHandle_t s_lcd_frame_q  = nullptr;
static lvgl_ui_task_t s_running_task = LVGL_UI_TASK_NONE;
static bool          s_pipeline_started = false;
static bool          s_paused_for_settings = false;
static std::atomic<bool> s_restart_requested{false}; // set from UI when dropdown confirmed
static std::atomic<lvgl_ui_task_t> s_restart_task{LVGL_UI_TASK_NONE};
static std::atomic<bool> s_stop_for_dropdown_requested{false};
static std::atomic<bool> s_pause_for_settings_requested{false};
static std::atomic<bool> s_resume_after_settings_requested{false};
static std::atomic<bool> s_uart_stop_requested{false};
static bool          s_stopped_for_dropdown = false; // true only when we stopped pipeline due to dropdown open
static SemaphoreHandle_t s_pipeline_mu = nullptr;

typedef struct {
    lvgl_ui_task_t task;
    uint8_t        color_id;
    tagformat_t    tag_dict;
    bool           trigger_color_learn;
    uint8_t        ai_model_id;
    uint8_t        ai_score_id;
} uart_task_cmd_t;

static QueueHandle_t s_uart_cmd_q = nullptr;
static bool          s_uart_started = false;

class PipelineLockGuard {
public:
    PipelineLockGuard()
        : mu_(s_pipeline_mu)
    {
        if (mu_) {
            xSemaphoreTakeRecursive(mu_, portMAX_DELAY);
        }
    }

    ~PipelineLockGuard()
    {
        if (mu_) {
            xSemaphoreGiveRecursive(mu_);
        }
    }

    PipelineLockGuard(const PipelineLockGuard&) = delete;
    PipelineLockGuard& operator=(const PipelineLockGuard&) = delete;

private:
    SemaphoreHandle_t mu_;
};

static void lvgl_refresh_cb(void *arg)
{
    (void)arg;
    lv_obj_invalidate(lv_scr_act());
    lv_disp_t *disp = lv_disp_get_default();
    if (disp) {
        lv_refr_now(disp);
    }
}

static uartp_task_t to_uartp_task(lvgl_ui_task_t ui_task)
{
    switch (ui_task) 
    {
    case LVGL_UI_TASK_FIND_COLOR:   return UARTP_TASK_COLOR_LEARN;
    case LVGL_UI_TASK_COLOR_DETECT: return UARTP_TASK_COLOR_DETECT;
    case LVGL_UI_TASK_APRILTAG:     return UARTP_TASK_APRILTAG;
    case LVGL_UI_TASK_LINE_DETECT:  return UARTP_TASK_LINE_FOLLOW;
    case LVGL_UI_TASK_AI_DETECT:    return UARTP_TASK_AI_DETECT;
    default:                        return UARTP_TASK_NONE;
    }
}

static tagformat_t to_tag_dict(lvgl_ui_tag_dict_t tag_sel)
{
    switch (tag_sel) {
    case LVGL_UI_TAG_16H5:  return tag16h5;
    case LVGL_UI_TAG_36H11: return tag36h11;
    default:                return none;
    }
}

static lvgl_ui_tag_dict_t from_tag_dict(tagformat_t dict)
{
    switch (dict) {
    case tag16h5:  return LVGL_UI_TAG_16H5;
    case tag36h11: return LVGL_UI_TAG_36H11;
    default:       return LVGL_UI_TAG_NONE;
    }
}

static inline bool uart_cmd_accept_now(void)
{
    return lvgl_ui_is_main_screen_active();
}

static void enqueue_uart_cmd(const uart_task_cmd_t *cmd)
{
    if (!cmd || !s_uart_cmd_q) return;
    (void)xQueueSend(s_uart_cmd_q, cmd, 0);
}

static void uart_on_color_learn(uint8_t color_id)
{
    if (!uart_cmd_accept_now()) return;
    if (color_id < UARTP_COLOR_ID_MIN || color_id > UARTP_COLOR_ID_MAX) {
        ESP_LOGW(TAG, "UART D1H invalid color_id=%u", (unsigned)color_id);
        return;
    }

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_FIND_COLOR,
        .color_id = color_id,
        .tag_dict = none,
        .trigger_color_learn = true,
        .ai_model_id = 0,
        .ai_score_id = 0,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_color_detect(uint8_t color_id)
{
    if (!uart_cmd_accept_now()) return;
    if (color_id < UARTP_COLOR_ID_MIN || color_id > UARTP_COLOR_ID_MAX) {
        ESP_LOGW(TAG, "UART D2H invalid color_id=%u", (unsigned)color_id);
        return;
    }

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_COLOR_DETECT,
        .color_id = color_id,
        .tag_dict = none,
        .trigger_color_learn = false,
        .ai_model_id = 0,
        .ai_score_id = 0,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_apriltag(void)
{
    if (!uart_cmd_accept_now()) return;

    uartp_task_state_t st = {};
    uartp_get_current_task(&st);

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_APRILTAG,
        .color_id = 0,
        .tag_dict = st.tag_dict,
        .trigger_color_learn = false,
        .ai_model_id = 0,
        .ai_score_id = 0,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_line_follow(uint8_t color_id)
{
    if (!uart_cmd_accept_now()) return;
    if (color_id < UARTP_COLOR_ID_MIN || color_id > UARTP_COLOR_ID_MAX) {
        ESP_LOGW(TAG, "UART D4H invalid color_id=%u", (unsigned)color_id);
        return;
    }

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_LINE_DETECT,
        .color_id = color_id,
        .tag_dict = none,
        .trigger_color_learn = false,
        .ai_model_id = 0,
        .ai_score_id = 0,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_ai_detect(uint8_t model_id, uint8_t score_id)
{
    if (!uart_cmd_accept_now()) return;
    if (model_id == 0 || score_id == 0) {
        ESP_LOGW(TAG, "UART D5H invalid model_id=%u score_id=%u", (unsigned)model_id, (unsigned)score_id);
        return;
    }
    if (score_id > 7) {
        ESP_LOGW(TAG, "UART D5H invalid score_id=%u", (unsigned)score_id);
        return;
    }

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_AI_DETECT,
        .color_id = 0,
        .tag_dict = none,
        .trigger_color_learn = false,
        .ai_model_id = model_id,
        .ai_score_id = score_id,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_frame_overlay(bool enable)
{
    lvgl_ui_set_selected_frame_mode(enable ? LVGL_UI_FRAME_ON : LVGL_UI_FRAME_OFF);
}

static void uart_on_fill_light(bool enable)
{
    lvgl_ui_set_selected_led_mode(enable ? LVGL_UI_LED_ON : LVGL_UI_LED_OFF);
}

static void uart_on_msroi_ctrl(bool msroi_enable,
                               bool module1_enable,
                               bool module2_enable,
                               bool module3_enable)
{
    lvgl_ui_set_selected_msroi_modules(module1_enable, module2_enable, module3_enable);
    lvgl_ui_set_selected_msroi_enabled(msroi_enable);
    ESP_LOGI(TAG, "UART DAH msroi=%u m1=%u m2=%u m3=%u",
             msroi_enable ? 1u : 0u,
             module1_enable ? 1u : 0u,
             module2_enable ? 1u : 0u,
             module3_enable ? 1u : 0u);
}

static void uart_on_empty_task(bool enable)
{
    if (!uart_cmd_accept_now() || !enable) {
        return;
    }

    uart_task_cmd_t cmd = {
        .task = LVGL_UI_TASK_NONE,
        .color_id = 0,
        .tag_dict = none,
        .trigger_color_learn = false,
        .ai_model_id = 0,
        .ai_score_id = 0,
    };
    enqueue_uart_cmd(&cmd);
}

static void uart_on_stop(uint8_t reason)
{
    (void)reason;
    s_uart_stop_requested.store(true, std::memory_order_release);
}

static void stop_current_pipeline(void)
{
    PipelineLockGuard lock;
    wifi_stream_stop_continuous();
    wifi_stream_consume_single_shot();

    if (!s_pipeline_started && s_running_task == LVGL_UI_TASK_NONE &&
        !s_cam_frame_q && !s_lcd_frame_q &&
        !camera_task_is_created() && !lcd_task_is_created()) {
        return;
    }

    switch (s_running_task) {
    case LVGL_UI_TASK_FIND_COLOR:   unregister_find_color(); break;
    case LVGL_UI_TASK_COLOR_DETECT: unregister_color_detection(); break;
    case LVGL_UI_TASK_APRILTAG:     unregister_apriltag_detection(); break;
    case LVGL_UI_TASK_LINE_DETECT:  unregister_line_detection(); break;
    case LVGL_UI_TASK_AI_DETECT:    unregister_AIdetection(); break;
    default: break;
    }

    lcd_task_delete();
    camera_stop();

    QueueHandle_t q_cam = s_cam_frame_q;
    QueueHandle_t q_lcd = s_lcd_frame_q;
    if (q_cam && q_cam == q_lcd) {
        vQueueDelete(q_cam);
    } else {
        if (q_cam) { vQueueDelete(q_cam); }
        if (q_lcd) { vQueueDelete(q_lcd); }
    }
    s_cam_frame_q = nullptr;
    s_lcd_frame_q = nullptr;

    uartp_set_task_state(UARTP_TASK_NONE, 0, none);
    lvgl_ui_async_call(lvgl_refresh_cb, nullptr);
    s_running_task = LVGL_UI_TASK_NONE;
    s_pipeline_started = false;
    ESP_LOGI(TAG, "pipeline stopped");
}

static bool start_pipeline(lvgl_ui_task_t ui_task)
{
    PipelineLockGuard lock;
    if (ui_task != LVGL_UI_TASK_FIND_COLOR) {
        find_color_request_learn(0);
    }
    uint8_t color_id = lvgl_ui_get_selected_color_profile();
    const bool color_task = (ui_task == LVGL_UI_TASK_FIND_COLOR ||
                             ui_task == LVGL_UI_TASK_COLOR_DETECT ||
                             ui_task == LVGL_UI_TASK_LINE_DETECT);
    if (color_task && color_id == 0) {
        lvgl_ui_show_hint("请选择颜色ID");
        ESP_LOGW(TAG, "start pipeline aborted: ColorID not selected");
        return false;
    }

    tagformat_t tag_dict = to_tag_dict(lvgl_ui_get_selected_tag_dict());
    if (ui_task == LVGL_UI_TASK_APRILTAG && tag_dict == none) {
        lvgl_ui_show_hint("请选择AprilTag类别");
        ESP_LOGW(TAG, "start pipeline aborted: AprilTag dict not selected");
        return false;
    }

    stop_current_pipeline();

    int xclk = XCLK_FREQ;
    if (ui_task == LVGL_UI_TASK_AI_DETECT) {
        xclk = 16000000; // force psram_mode to avoid DMA malloc failure during AI model preload
    }
    camera_set_xclk(xclk);

    uartp_task_t uart_task = to_uartp_task(ui_task);
    uartp_set_task_state(uart_task, color_id, tag_dict);

    bool ok = false;
    switch (ui_task) {
    case LVGL_UI_TASK_NONE:
        s_cam_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        if (s_cam_frame_q) {
            s_lcd_frame_q = s_cam_frame_q;
            /* TaskNone: 纯预览，保持分辨率与色彩，提升刷新节奏与缓冲深度 */
            register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 3, s_cam_frame_q);
            register_lcd(s_cam_frame_q, s_cam_frame_q, UARTP_TASK_NONE);
            lcd_set_frame_interval_ms(10); // 更快的 LCD 刷新节奏，目标 ~20fps 预览
            ok = camera_task_is_created() && lcd_task_is_created();
        }
        break;

    case LVGL_UI_TASK_FIND_COLOR:
        s_cam_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        s_lcd_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        if (s_cam_frame_q && s_lcd_frame_q) {
            register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 3, s_cam_frame_q);
            register_find_color(s_cam_frame_q, s_lcd_frame_q, color_id);
            register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_COLOR_LEARN);
            lcd_set_frame_interval_ms(10); // 提高 LCD 刷新节奏，目标 20fps 预览
            ok = camera_task_is_created() && lcd_task_is_created();
        }
        break;

    case LVGL_UI_TASK_COLOR_DETECT:
        s_cam_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        s_lcd_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        if (s_cam_frame_q && s_lcd_frame_q) {
            register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 3, s_cam_frame_q);
            register_color_detection(s_cam_frame_q, s_lcd_frame_q, color_id);
            register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_COLOR_DETECT);
            lcd_set_frame_interval_ms(18); // 提升显示节奏，目标 ~20fps
            ok = camera_task_is_created() && lcd_task_is_created();
        }
        break;

    case LVGL_UI_TASK_APRILTAG:
        s_cam_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        s_lcd_frame_q = xQueueCreate(3, sizeof(camera_fb_t *));
        if (s_cam_frame_q && s_lcd_frame_q) {
            register_camera(PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA, 3, s_cam_frame_q);
            register_apriltag_detection(s_cam_frame_q, s_lcd_frame_q, tag_dict);
            register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_APRILTAG);
            lcd_set_frame_interval_ms(18); // 提升显示节奏，目标接近 20fps
            ok = camera_task_is_created() && lcd_task_is_created();
        }
        break;

    case LVGL_UI_TASK_LINE_DETECT:
        s_cam_frame_q = xQueueCreate(2, sizeof(camera_fb_t *));
        s_lcd_frame_q = xQueueCreate(2, sizeof(camera_fb_t *));
        if (s_cam_frame_q && s_lcd_frame_q) {
            register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, s_cam_frame_q);
            register_line_detection(s_cam_frame_q, s_lcd_frame_q, color_id);
            register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_LINE_FOLLOW);
            ok = camera_task_is_created() && lcd_task_is_created();
        }
        break;

    case LVGL_UI_TASK_AI_DETECT:
        {
            // QueueHandle_t cam_q = xQueueCreate(2, sizeof(camera_fb_t *));
            // QueueHandle_t lcd_q = xQueueCreate(2, sizeof(camera_fb_t *));

            char model_name[256];
            if (!lvgl_ui_get_selected_model(model_name, sizeof(model_name))) {
                lvgl_ui_show_hint("请选择AI模型");
                ESP_LOGW(TAG, "AI detect: no model selected");
                ok = false;
                break;
            }

            float score_thr = 0.0f;
            if (!lvgl_ui_get_selected_ai_score(&score_thr)) {
                lvgl_ui_show_hint("请选择AI检测阈值");
                ESP_LOGW(TAG, "AI detect: no score selected");
                ok = false;
                break;
            }

            char model_path[320];
            snprintf(model_path, sizeof(model_path), "/sdcard/models/%s", model_name);
            sd_init();
            struct stat st;
            if (stat(model_path, &st) != 0) {
                lvgl_ui_show_hint("模型文件不存在");
                ESP_LOGW(TAG, "AI detect: model file not found: %s", model_path);
                ok = false;
                break;
            }

            // Preload model before camera starts to avoid SDMMC DMA memory pressure.
            esp_err_t model_err = (esp_err_t)y11_init_from_sd(model_path, score_thr, 0.70f);
            if (model_err != ESP_OK) {
                lvgl_ui_show_hint("AI模型加载失败");
                ESP_LOGW(TAG, "AI detect: model init failed (%d)", (int)model_err);
                ok = false;
                break;
            }

            QueueHandle_t cam_q = xQueueCreate(2, sizeof(camera_fb_t *));
            QueueHandle_t lcd_q = xQueueCreate(2, sizeof(camera_fb_t *));
            if (cam_q && lcd_q) {
                s_cam_frame_q = cam_q;
                s_lcd_frame_q = lcd_q;
                register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, s_cam_frame_q);
                register_AIdetection(s_cam_frame_q, s_lcd_frame_q, model_path, score_thr);
                //register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_NONE);
                register_lcd(s_lcd_frame_q, NULL, UARTP_TASK_AI_DETECT);
                ok = camera_task_is_created() && lcd_task_is_created();
            } else {
                if (cam_q) vQueueDelete(cam_q);
                if (lcd_q) vQueueDelete(lcd_q);
                y11_deinit();
                ok = false;
            }
        }
        break;

    default:
        break;
    }

    if (!ok) {
        ESP_LOGE(TAG, "start pipeline failed, task=%d", (int)ui_task);
        lvgl_ui_show_hint("Start failed");
        s_running_task = ui_task;
        stop_current_pipeline();
        return false;
    }

    s_running_task = ui_task;
    s_pipeline_started = true;
    s_stopped_for_dropdown = false;
    if (s_lcd_frame_q) {
        wifi_stream_update_frame_queue(s_lcd_frame_q);
        register_wifi(s_lcd_frame_q); // safe to call once; subsequent calls only update queue
    }
    ESP_LOGI(TAG, "pipeline started for task=%d", (int)ui_task);
    return true;
}

static void ui_watch_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (lvgl_ui_consume_close_clicked()) {
            s_restart_requested.store(false, std::memory_order_release); // user explicitly stopped, cancel pending restarts
            s_stop_for_dropdown_requested.store(false, std::memory_order_release);
            stop_current_pipeline();
            s_stopped_for_dropdown = false;
        }

        if (s_uart_stop_requested.exchange(false, std::memory_order_acq_rel)) {
            s_restart_requested.store(false, std::memory_order_release);
            s_stop_for_dropdown_requested.store(false, std::memory_order_release);
            stop_current_pipeline();
            s_stopped_for_dropdown = false;
        }

        if (s_pause_for_settings_requested.exchange(false, std::memory_order_acq_rel)) {
            bool need_stop = false;
            {
                PipelineLockGuard lock;
                if (s_pipeline_started) {
                    s_paused_for_settings = true;
                    need_stop = true;
                }
            }
            if (need_stop) {
                stop_current_pipeline();
                s_stopped_for_dropdown = false;
            }
        }

        if (s_resume_after_settings_requested.exchange(false, std::memory_order_acq_rel)) {
            bool need_resume = false;
            {
                PipelineLockGuard lock;
                if (s_paused_for_settings) {
                    s_paused_for_settings = false;
                    need_resume = true;
                }
            }
            if (need_resume) {
                lvgl_ui_task_t sel = lvgl_ui_get_selected_task();
                if (!start_pipeline(sel)) {
                    ESP_LOGW(TAG, "resume after settings failed, task=%d", (int)sel);
                }
            }
        }

        if (s_stop_for_dropdown_requested.exchange(false, std::memory_order_acq_rel)) {
            PipelineLockGuard lock;
            if (s_pipeline_started) {
                s_stopped_for_dropdown = true;
                stop_current_pipeline();
            }
        }

        if (s_uart_cmd_q) {
            uart_task_cmd_t cmd = {};
            while (xQueueReceive(s_uart_cmd_q, &cmd, 0) == pdTRUE) {
                s_restart_requested.store(false, std::memory_order_release);
                s_stop_for_dropdown_requested.store(false, std::memory_order_release);
                {
                    PipelineLockGuard lock;
                    s_stopped_for_dropdown = false;
                }

                lvgl_ui_set_selected_task(cmd.task);
                if (cmd.color_id) {
                    lvgl_ui_set_selected_color_profile(cmd.color_id);
                }
                if (cmd.task == LVGL_UI_TASK_APRILTAG) {
                    lvgl_ui_set_selected_tag_dict(from_tag_dict(cmd.tag_dict));
                }
                if (cmd.task == LVGL_UI_TASK_AI_DETECT) {
                    if (cmd.ai_model_id) {
                        lvgl_ui_set_selected_ai_model(cmd.ai_model_id);
                    }
                    if (cmd.ai_score_id) {
                        lvgl_ui_set_selected_ai_score(cmd.ai_score_id);
                    }
                }

                bool started = start_pipeline(cmd.task);
                if (started && cmd.task == LVGL_UI_TASK_FIND_COLOR &&
                    cmd.trigger_color_learn && cmd.color_id) {
                    find_color_request_learn(cmd.color_id);
                }
            }
        }

        lvgl_ui_task_t pending_task = LVGL_UI_TASK_NONE;
        if (s_restart_requested.exchange(false, std::memory_order_acq_rel)) {
            s_stop_for_dropdown_requested.store(false, std::memory_order_release);
            pending_task = s_restart_task.load(std::memory_order_acquire);
            bool can_auto_start = false;
            {
                PipelineLockGuard lock;
                can_auto_start = (s_pipeline_started || s_stopped_for_dropdown);
            }
            if (can_auto_start) {
                start_pipeline(pending_task);
            }
        }

        if (lvgl_ui_consume_open_clicked()) {
            lvgl_ui_task_t sel = lvgl_ui_get_selected_task();
            bool need_start = false;
            {
                PipelineLockGuard lock;
                need_start = (!s_pipeline_started || sel != s_running_task);
            }
            if (need_start) {
                start_pipeline(sel);
            }
        }

        lvgl_ui_task_t running = TaskScheduling_GetRunningTask();
        if (running != LVGL_UI_TASK_FIND_COLOR && lvgl_ui_consume_web_clicked()) {
            lvgl_ui_stream_mode_t stream_mode = lvgl_ui_get_selected_stream_mode();
            if (stream_mode == LVGL_UI_STREAM_NONE) {
                lvgl_ui_show_hint("请选择图传模式");
            } else if (!wifi_stream_has_client() && stream_mode == LVGL_UI_STREAM_SINGLE) {
                lvgl_ui_show_hint("未连接图传客户端");
                wifi_stream_stop_continuous();
            } else if (stream_mode == LVGL_UI_STREAM_SINGLE) {
                wifi_stream_stop_continuous();
                wifi_stream_request_single_shot();
            } else {
                if (wifi_stream_is_running()) {
                    wifi_stream_stop_continuous();
                } else if (!wifi_stream_start_continuous()) {
                    lvgl_ui_show_hint("图传未就绪");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void TaskScheduling_Init(void)
{
    if (!s_pipeline_mu) {
        s_pipeline_mu = xSemaphoreCreateRecursiveMutex();
        if (!s_pipeline_mu) {
            ESP_LOGE(TAG, "create pipeline mutex failed");
        }
    }
    if (!s_uart_cmd_q) {
        s_uart_cmd_q = xQueueCreate(8, sizeof(uart_task_cmd_t));
        if (!s_uart_cmd_q) {
            ESP_LOGE(TAG, "create uart cmd queue failed");
        }
    }

    uartp_set_handlers(uart_on_color_learn,
                       uart_on_color_detect,
                       uart_on_apriltag,
                       uart_on_line_follow,
                       uart_on_ai_detect,
                       uart_on_frame_overlay,
                       uart_on_fill_light,
                       uart_on_stop,
                       uart_on_empty_task,
                       uart_on_msroi_ctrl);

    if (!s_uart_started) {
        uartp_config_t cfg = {
            .port = UART_NUM_1,
            .tx_gpio = 20,
            .rx_gpio = 19,
            .baud = 115200,
            .rx_buf = 2048,
            .tx_buf = 2048,
            .rx_task_core = -1,
            .tx_task_core = -1,
            .rx_task_prio = 8,
            .tx_task_prio = 7,
        };
        esp_err_t err = uartp_start(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uartp_start failed: %s(0x%x)", esp_err_to_name(err), (unsigned)err);
        } else {
            s_uart_started = true;
        }
    }

    xTaskCreatePinnedToCore(ui_watch_task, "ui_watch", 4096, nullptr, 3, nullptr, 0);
}

void TaskScheduling_PauseForSettings(void)
{
    PipelineLockGuard lock;
    if (s_pipeline_started) {
        s_paused_for_settings = true;
        stop_current_pipeline();
    }
}

void TaskScheduling_ResumeAfterSettings(void)
{
    PipelineLockGuard lock;
    if (!s_paused_for_settings) return;
    s_paused_for_settings = false;
    lvgl_ui_task_t sel = lvgl_ui_get_selected_task();
    if (!start_pipeline(sel)) {
        ESP_LOGW(TAG, "resume after settings failed, task=%d", (int)sel);
    }
}

void TaskScheduling_RequestPauseForSettings(void)
{
    s_pause_for_settings_requested.store(true, std::memory_order_release);
}

void TaskScheduling_RequestResumeAfterSettings(void)
{
    s_pause_for_settings_requested.store(false, std::memory_order_release);
    s_resume_after_settings_requested.store(true, std::memory_order_release);
}

void TaskScheduling_RequestRestartSelected(lvgl_ui_task_t task)
{
    /* 仅在之前有流水线运行或确实因下拉而暂停时才允许自动重启；
     * 这样“初始无任务”时选择下拉不会立刻开跑，需按 Open */
    s_restart_task.store(task, std::memory_order_release);
    s_restart_requested.store(true, std::memory_order_release);
}

void TaskScheduling_StopForDropdown(void)
{
    s_stop_for_dropdown_requested.store(true, std::memory_order_release);
}

lvgl_ui_task_t TaskScheduling_GetRunningTask(void)
{
    PipelineLockGuard lock;
    return s_pipeline_started ? s_running_task : LVGL_UI_TASK_NONE;
}
