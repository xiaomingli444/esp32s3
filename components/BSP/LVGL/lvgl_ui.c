#include "lvgl_ui.h"
#include "key.h"
#include "sys.h"
#include "device_identity.h"
#include "vision_msroi.h"
#include "sd.h"
#include "TaskScheduling.h"
#include "lcd.h"
#include "led.h"
#include "wifi.h"
#include "fw_update.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG_UI = "LVGL_UI";
#define WIFI_MAX_LIST_ITEMS 12

LV_FONT_DECLARE(lv_font_source_han_sans_sc_14_system)

typedef void (*lvgl_ui_async_cb_t)(void *user_data);
typedef struct {
    lvgl_ui_async_cb_t cb;
    void *user_data;
} lvgl_ui_async_msg_t;

static QueueHandle_t s_ui_async_q = NULL;
static lv_timer_t   *s_ui_async_timer = NULL;

static void ui_async_ensure(void)
{
    if (s_ui_async_q) return;
    /* 仅用于跨线程投递 UI 更新，消息体很小 */
    s_ui_async_q = xQueueCreate(32, sizeof(lvgl_ui_async_msg_t));
    if (!s_ui_async_q) {
        ESP_LOGE(TAG_UI, "ui async queue create failed");
    }
}

static void ui_async_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (!s_ui_async_q) return;

    lvgl_ui_async_msg_t msg;
    while (xQueueReceive(s_ui_async_q, &msg, 0) == pdTRUE) {
        if (msg.cb) {
            msg.cb(msg.user_data);
        }
    }
}

/* 主界面控件 */
static lv_obj_t *s_main_screen        = NULL;
static bool     s_programmatic_dropdown = false;
static volatile bool s_is_main_screen_active = false;
static lv_obj_t *s_dropdown_task      = NULL;
static lv_obj_t *s_btn_control        = NULL;
static lv_obj_t *s_btn_open           = NULL;
static lv_obj_t *s_btn_close          = NULL;
static lv_obj_t *s_btn_web            = NULL;
static lv_obj_t *s_btn_about          = NULL;
static volatile bool s_open_clicked       = false;
static volatile bool s_close_clicked      = false;
static volatile bool s_web_clicked        = false;
static bool s_dropdown_paused_lcd     = false;
static uint16_t s_sel_task            = 0;

/* 控制页控件 */
static lv_obj_t *s_control_screen     = NULL;
static lv_obj_t *s_btn_back           = NULL;
static lv_obj_t *s_dd_color_profile   = NULL;
static lv_obj_t *s_dd_tag_dict        = NULL;
static lv_obj_t *s_dd_ai_model        = NULL;
static lv_obj_t *s_dd_ai_score        = NULL;
static lv_obj_t *s_dd_stream_mode     = NULL;
static lv_obj_t *s_dd_fill_light      = NULL;
static lv_obj_t *s_dd_frame_overlay   = NULL;
static lv_obj_t *s_dd_msroi           = NULL;
static lv_obj_t *s_dd_m1_coarse_mode  = NULL;
static lv_obj_t *s_dd_m2_fallback     = NULL;
static lv_obj_t *s_dd_m3_roi_refine   = NULL;
static uint16_t s_sel_color_profile   = 0;
static uint16_t s_sel_tag_dict        = 0;
static uint16_t s_sel_ai_model        = 0;
static uint16_t s_sel_ai_score        = 0;
static uint16_t s_sel_stream_mode     = 0;
static uint16_t s_sel_fill_light      = 0;
static uint16_t s_sel_frame_overlay   = 0;
static uint16_t s_sel_msroi           = 0;
static uint16_t s_sel_m1_coarse_mode  = 0;
static uint16_t s_sel_m2_fallback     = 0;
static uint16_t s_sel_m3_roi_refine   = 0;
static bool     s_main_screen_ready   = false;

/* 关于页控件 */
static lv_obj_t *s_about_screen       = NULL;
static lv_obj_t *s_about_info_label   = NULL;
static lv_obj_t *s_about_btn_back     = NULL;

/* Wi-Fi 启动页控件 */
static lv_obj_t *s_wifi_screen        = NULL;
static lv_obj_t *s_wifi_list_panel    = NULL;
static lv_obj_t *s_wifi_status_label  = NULL;
static lv_obj_t *s_wifi_ssid_value    = NULL;
static lv_obj_t *s_wifi_pass_ta       = NULL;
static lv_obj_t *s_wifi_btn_scan      = NULL;
static lv_obj_t *s_wifi_btn_connect   = NULL;
static lv_obj_t *s_wifi_btn_enter     = NULL;
static lv_obj_t *s_wifi_keyboard      = NULL;
static TaskHandle_t s_wifi_ap_autostart_task = NULL;
typedef struct {
    lv_obj_t *btn;
    char ssid[33];
} wifi_list_item_t;
static wifi_list_item_t s_wifi_list_items[WIFI_MAX_LIST_ITEMS];
static uint16_t  s_wifi_list_count    = 0;
static lv_obj_t *s_wifi_selected_item = NULL;
static bool      s_wifi_sta_ok        = false;
static volatile bool s_wifi_scanning  = false;
static char      s_wifi_status_buf[96] = {0};
static lv_timer_t *s_wifi_sta_auto_enter_timer = NULL;

/* HTTP 模型上传弹窗 */
static lv_obj_t   *s_upload_overlay        = NULL;
static lv_obj_t   *s_upload_panel          = NULL;
static lv_obj_t   *s_upload_title_label    = NULL;
static lv_obj_t   *s_upload_meta_label     = NULL;
static lv_obj_t   *s_upload_progress_label = NULL;
static lv_obj_t   *s_upload_progress_bar   = NULL;
static lv_obj_t   *s_upload_btn_accept     = NULL;
static lv_obj_t   *s_upload_btn_reject     = NULL;
static lv_obj_t   *s_upload_btn_cancel     = NULL;
static lv_timer_t *s_upload_timeout_timer  = NULL;
static lv_timer_t *s_upload_close_timer    = NULL;
static char        s_upload_request_id[40] = {0};
static uint32_t    s_upload_total_bytes    = 0;
static char        s_upload_type[16]       = {0};
static lv_group_t *s_upload_group          = NULL;
static lv_obj_t   *s_upload_prev_focus     = NULL;

/* STA 自动固件更新弹窗（仅在版本落后时弹出） */
static lv_obj_t   *s_fwupd_overlay        = NULL;
static lv_obj_t   *s_fwupd_panel          = NULL;
static lv_obj_t   *s_fwupd_title_label    = NULL;
static lv_obj_t   *s_fwupd_meta_label     = NULL;
static lv_obj_t   *s_fwupd_progress_label = NULL;
static lv_obj_t   *s_fwupd_progress_bar   = NULL;
static lv_obj_t   *s_fwupd_btn_accept     = NULL;
static lv_obj_t   *s_fwupd_btn_reject     = NULL;
static lv_timer_t *s_fwupd_close_timer    = NULL;
static lv_group_t *s_fwupd_group          = NULL;
static lv_obj_t   *s_fwupd_prev_focus     = NULL;

static void apply_led_setting(void);
static void apply_frame_overlay_setting(void);
static void apply_msroi_setting(void);
static void apply_msroi_module_settings(void);
static void apply_runtime_settings(void);
static void resume_lcd_if_needed(lv_obj_t *dd);
static lvgl_ui_task_t task_from_index(uint16_t idx);
static void fps_timer_cb(lv_timer_t *t);
static void ui_create_wifi_screen(void);
static void ui_load_main_screen(void);
static void wifi_ap_autostart_task(void *arg);
static void wifi_ap_autostart_cancel_request(void);
static void wifi_sta_auto_enter_cancel(void);
static void wifi_sta_auto_enter_timer_cb(lv_timer_t *t);
static void wifi_sta_auto_enter_schedule_async(void *user_data);
static void wifi_sta_auto_enter_cancel_async(void *user_data);

/* 焦点组与按键输入设备 */
static lv_group_t *s_group            = NULL;
static lv_indev_t *s_keypad           = NULL;

/* 底部提示标签 */
static lv_obj_t *s_hint_label         = NULL;
static lv_timer_t *s_hint_timer       = NULL;
/* FPS 标签与定时器 */
static lv_obj_t *s_fps_label          = NULL;
static lv_timer_t *s_fps_timer        = NULL;
static bool       s_fps_hidden_for_lcd = false;

extern uint32_t lvgl_get_ui_fps(void);

static void resume_lcd_if_needed(lv_obj_t *dd)
{
    if (s_dropdown_paused_lcd && dd == s_dropdown_task) {
        lcd_task_resume();
        s_dropdown_paused_lcd = false;
        lv_obj_invalidate(lv_scr_act());
        lv_refr_now(NULL);
    }
}

static void resume_lcd_async(void *user_data)
{
    resume_lcd_if_needed((lv_obj_t *)user_data);
}

static void fps_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (s_fps_label) {
        lv_obj_add_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN);
        s_fps_hidden_for_lcd = true;
    }
}

static lvgl_ui_task_t task_from_index(uint16_t idx)
{
    switch (idx) {
    case 1: return LVGL_UI_TASK_FIND_COLOR;
    case 2: return LVGL_UI_TASK_COLOR_DETECT;
    case 3: return LVGL_UI_TASK_APRILTAG;
    case 4: return LVGL_UI_TASK_LINE_DETECT;
    case 5: return LVGL_UI_TASK_AI_DETECT;
    default: return LVGL_UI_TASK_NONE;
    }
}

/**********************
 *  本地工具函数
 **********************/

/* 在屏幕底部显示短暂提示文本 */
static void hide_hint_cb(lv_timer_t *t)
{
    lv_obj_t *hint = (lv_obj_t *)t->user_data;
    if (hint != NULL) {
        lv_obj_add_flag(hint, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_hint_timer == t) {
        s_hint_timer = NULL;
    }
    lv_timer_del(t);
}

static void ui_show_hint_ms(const char *text, uint32_t duration_ms)
{
    lv_obj_t *act_scr = lv_scr_act();

    if (!text || text[0] == '\0') {
        if (s_hint_timer) {
            lv_timer_del(s_hint_timer);
            s_hint_timer = NULL;
        }
        if (s_hint_label && lv_obj_get_screen(s_hint_label) == act_scr) {
            lv_obj_add_flag(s_hint_label, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    if (s_hint_label == NULL || lv_obj_get_screen(s_hint_label) != act_scr) {
        s_hint_label = lv_label_create(act_scr);
        // 按钮风格的提示胶囊（与主界面按钮同色：浅灰底+白字）
        lv_obj_set_style_bg_color(s_hint_label, lv_color_hex(0xEFEFEF), 0);
        lv_obj_set_style_bg_opa(s_hint_label, LV_OPA_90, 0);
        lv_obj_set_style_border_color(s_hint_label, lv_color_hex(0xC8CDD5), 0);
        lv_obj_set_style_border_width(s_hint_label, 2, 0);
        lv_obj_set_style_border_opa(s_hint_label, LV_OPA_70, 0);
        lv_obj_set_style_radius(s_hint_label, 14, 0);
        lv_obj_set_style_shadow_color(s_hint_label, lv_color_hex(0x9FA6B5), 0);
        lv_obj_set_style_shadow_opa(s_hint_label, LV_OPA_40, 0);
        lv_obj_set_style_shadow_width(s_hint_label, 6, 0);
        lv_obj_set_style_text_color(s_hint_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(s_hint_label, &lv_font_source_han_sans_sc_14_system, 0);
        lv_obj_set_style_text_letter_space(s_hint_label, 1, 0);
        lv_obj_set_style_pad_hor(s_hint_label, 14, 0);
        lv_obj_set_style_pad_ver(s_hint_label, 8, 0);
    }

    lv_label_set_text(s_hint_label, (text != NULL) ? text : "");
    lv_obj_align(s_hint_label, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_clear_flag(s_hint_label, LV_OBJ_FLAG_HIDDEN);

    if (s_hint_timer) {
        lv_timer_del(s_hint_timer);
        s_hint_timer = NULL;
    }
    if (duration_ms == 0) {
        duration_ms = 1;
    }
    s_hint_timer = lv_timer_create(hide_hint_cb, duration_ms, s_hint_label);
}

static void apply_focus_border_style(lv_obj_t *obj)
{
    if (!obj) return;

    const lv_color_t c = lv_color_hex(0x2A7FFF);

    lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN | LV_STATE_EDITED);

    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_color(obj, c, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_FOCUS_KEY);

    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(obj, c, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_FOCUSED);

    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_EDITED);
    lv_obj_set_style_border_color(obj, c, LV_PART_MAIN | LV_STATE_EDITED);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_EDITED);
}

static void wifi_status_apply_async(void *user_data)
{
    LV_UNUSED(user_data);
    if (s_wifi_status_label) {
        lv_label_set_text(s_wifi_status_label, s_wifi_status_buf);
        if (s_wifi_status_buf[0] == '\0') {
            lv_obj_add_flag(s_wifi_status_label, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(s_wifi_status_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void lvgl_ui_wifi_set_status(const char *text)
{
    size_t len = text ? strlen(text) : 0;
    if (len >= sizeof(s_wifi_status_buf)) {
        len = sizeof(s_wifi_status_buf) - 1;
    }
    if (len > 0 && text) {
        memcpy(s_wifi_status_buf, text, len);
        s_wifi_status_buf[len] = '\0';
    } else {
        s_wifi_status_buf[0] = '\0';
    }
    (void)lvgl_ui_async_call(wifi_status_apply_async, NULL);
}

static void wifi_sta_auto_enter_cancel(void)
{
    if (s_wifi_sta_auto_enter_timer) {
        lv_timer_del(s_wifi_sta_auto_enter_timer);
        s_wifi_sta_auto_enter_timer = NULL;
    }
}

static void wifi_sta_auto_enter_timer_cb(lv_timer_t *t)
{
    if (s_wifi_sta_auto_enter_timer == t) {
        s_wifi_sta_auto_enter_timer = NULL;
    }
    lv_timer_del(t);

    if (!s_wifi_sta_ok) {
        return;
    }
    if (s_wifi_screen && lv_scr_act() == s_wifi_screen) {
        ui_load_main_screen();
    }
}

static void wifi_sta_auto_enter_schedule_async(void *user_data)
{
    LV_UNUSED(user_data);

    wifi_sta_auto_enter_cancel();
    if (!s_wifi_sta_ok) {
        return;
    }
    if (!s_wifi_screen || lv_scr_act() != s_wifi_screen) {
        return;
    }

    s_wifi_sta_auto_enter_timer = lv_timer_create(wifi_sta_auto_enter_timer_cb, 3000, NULL);
    if (s_wifi_sta_auto_enter_timer) {
        lv_timer_set_repeat_count(s_wifi_sta_auto_enter_timer, 1);
    }
}

static void wifi_sta_auto_enter_cancel_async(void *user_data)
{
    LV_UNUSED(user_data);
    wifi_sta_auto_enter_cancel();
}

static void wifi_ap_autostart_cancel_request(void)
{
    if (s_wifi_ap_autostart_task) {
        xTaskNotifyGive(s_wifi_ap_autostart_task);
    }
}

void lvgl_ui_wifi_on_sta_connected(const char *ip)
{
    s_wifi_sta_ok = true;
    char buf[96];
    if (ip && ip[0] != '\0') {
        snprintf(buf, sizeof(buf), "STA connected: %s", ip);
    } else {
        snprintf(buf, sizeof(buf), "STA connected");
    }
    lvgl_ui_wifi_set_status(buf);
    (void)lvgl_ui_async_call(wifi_sta_auto_enter_schedule_async, NULL);
}

void lvgl_ui_wifi_on_sta_disconnected(const char *reason)
{
    s_wifi_sta_ok = false;
    char buf[96];
    if (reason && reason[0] != '\0') {
        snprintf(buf, sizeof(buf), "%s", reason);
    } else {
        snprintf(buf, sizeof(buf), "STA disconnected");
    }
    lvgl_ui_wifi_set_status(buf);
    (void)lvgl_ui_async_call(wifi_sta_auto_enter_cancel_async, NULL);
}


static void ensure_nvs_ready(void)
{
    static bool nvs_ready = false;
    if (!nvs_ready) {
        NVS_init();
        nvs_ready = true;
    }
}

static void build_color_profile_options(char *buf, size_t len)
{
    if (!buf || len == 0) return;
    buf[0] = '\0';
    ensure_nvs_ready();

    size_t pos = snprintf(buf, len, "未设置颜色ID\n");
    if (pos >= len) { buf[len - 1] = '\0'; return; }

    for (uint8_t id = 1; id <= 7 && pos < len; ++id) {
        uint8_t r = 0, g = 0, b = 0;
        bool ok = nvs_read_rgb(id, &r, &g, &b);
        const char *fmt_ok = "%u: R%u G%u B%u\n";
        const char *fmt_na = "%u: N/A\n";
        int written = snprintf(buf + pos, len - pos, ok ? fmt_ok : fmt_na,
                               (unsigned)id, (unsigned)r, (unsigned)g, (unsigned)b);
        if (written < 0) break;
        pos += (size_t)written;
        if (pos >= len) { pos = len - 1; break; }
    }

    if (pos > 0 && pos < len && buf[pos - 1] == '\n') {
        buf[pos - 1] = '\0';
    } else {
        buf[len - 1] = '\0';
    }
}

static void wifi_clear_list(void)
{
    for (uint16_t i = 0; i < s_wifi_list_count; ++i) {
        if (s_wifi_list_items[i].btn != NULL) {
            if (s_group) {
                lv_group_remove_obj(s_wifi_list_items[i].btn);
            }
            lv_obj_del(s_wifi_list_items[i].btn);
        }
        s_wifi_list_items[i].btn = NULL;
        s_wifi_list_items[i].ssid[0] = '\0';
    }
    s_wifi_list_count = 0;
    s_wifi_selected_item = NULL;
}

static const char *wifi_ssid_from_btn(lv_obj_t *btn)
{
    if (!btn) return NULL;
    for (uint16_t i = 0; i < s_wifi_list_count; ++i) {
        if (s_wifi_list_items[i].btn == btn) {
            return s_wifi_list_items[i].ssid;
        }
    }
    return NULL;
}

static void wifi_set_selected(lv_obj_t *btn, const char *ssid)
{
    if (s_wifi_selected_item && s_wifi_selected_item != btn) {
        lv_obj_clear_state(s_wifi_selected_item, LV_STATE_CHECKED);
    }
    s_wifi_selected_item = btn;
    if (btn) {
        lv_obj_add_state(btn, LV_STATE_CHECKED);
    }
    if (s_wifi_ssid_value) {
        lv_label_set_text(s_wifi_ssid_value, (ssid && ssid[0] != '\0') ? ssid : "****");
    }
    if (s_wifi_pass_ta) {
        char saved[65] = {0};
        if (ssid && ssid[0] != '\0' && wifi_saved_password_get(ssid, saved, sizeof(saved))) {
            lv_textarea_set_text(s_wifi_pass_ta, saved);
        } else {
            lv_textarea_set_text(s_wifi_pass_ta, "");
        }
    }
    s_wifi_sta_ok = false;
}

static void wifi_refresh_group(void)
{
    if (!s_group) return;
    lv_group_remove_all_objs(s_group);

    if (s_wifi_list_count > 0) {
        for (uint16_t i = 0; i < s_wifi_list_count; ++i) {
            if (s_wifi_list_items[i].btn) {
                lv_group_add_obj(s_group, s_wifi_list_items[i].btn);
            }
        }
    }
    if (s_wifi_pass_ta)    lv_group_add_obj(s_group, s_wifi_pass_ta);
    if (s_wifi_btn_scan)   lv_group_add_obj(s_group, s_wifi_btn_scan);
    if (s_wifi_btn_connect)lv_group_add_obj(s_group, s_wifi_btn_connect);
    if (s_wifi_btn_enter)  lv_group_add_obj(s_group, s_wifi_btn_enter);
    if (s_wifi_keyboard && !lv_obj_has_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN)) {
        lv_group_add_obj(s_group, s_wifi_keyboard);
    }

    lv_obj_t *focus = NULL;
    if (s_wifi_list_count > 0) {
        focus = s_wifi_list_items[0].btn;
    } else if (s_wifi_pass_ta) {
        focus = s_wifi_pass_ta;
    } else if (s_wifi_btn_scan) {
        focus = s_wifi_btn_scan;
    }
    if (focus) {
        lv_group_focus_obj(focus);
    }
}

static void wifi_list_item_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);

    if (code == LV_EVENT_FOCUSED && s_wifi_list_panel) {
        lv_obj_scroll_to_view(target, LV_ANIM_OFF);
    }
    if (code == LV_EVENT_CLICKED || (code == LV_EVENT_KEY && lv_event_get_key(e) == LV_KEY_ENTER)) {
        const char *ssid = wifi_ssid_from_btn(target);
        wifi_set_selected(target, ssid);
    }
}

typedef struct {
    uint16_t count;
    wifi_ap_record_t recs[WIFI_MAX_LIST_ITEMS];
} wifi_scan_result_t;

static void wifi_scan_cleanup_async(void *user_data)
{
    LV_UNUSED(user_data);
    if (s_wifi_btn_scan) {
        lv_obj_clear_state(s_wifi_btn_scan, LV_STATE_DISABLED);
    }
    if (s_wifi_btn_connect) {
        lv_obj_clear_state(s_wifi_btn_connect, LV_STATE_DISABLED);
    }
    if (s_wifi_btn_enter) {
        lv_obj_clear_state(s_wifi_btn_enter, LV_STATE_DISABLED);
    }
    s_wifi_scanning = false;
}

static void wifi_apply_scan_result(void *user_data)
{
    wifi_scan_result_t *res = (wifi_scan_result_t *)user_data;
    if (res) {
        wifi_clear_list();
        uint16_t added = (res->count > WIFI_MAX_LIST_ITEMS) ? WIFI_MAX_LIST_ITEMS : res->count;
        for (uint16_t i = 0; i < added; ++i) {
            wifi_ap_record_t *r = &res->recs[i];
            lv_obj_t *btn = lv_btn_create(s_wifi_list_panel);
            lv_obj_set_width(btn, LV_PCT(100));
            lv_obj_set_height(btn, 24);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0xF5F5F5), 0);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(btn, 1, 0);
            lv_obj_set_style_border_color(btn, lv_color_hex(0xC8CDD5), 0);
            lv_obj_set_style_pad_all(btn, 6, 0);
            lv_obj_add_event_cb(btn, wifi_list_item_event_cb, LV_EVENT_ALL, NULL);
            apply_focus_border_style(btn);

            lv_obj_t *lbl = lv_label_create(btn);
            lv_label_set_text(lbl, (const char *)r->ssid);
            lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
            lv_obj_set_width(lbl, LV_PCT(100));
            lv_obj_center(lbl);

            s_wifi_list_items[i].btn = btn;
            strncpy(s_wifi_list_items[i].ssid, (const char *)r->ssid, sizeof(s_wifi_list_items[i].ssid) - 1);
            s_wifi_list_items[i].ssid[sizeof(s_wifi_list_items[i].ssid) - 1] = '\0';
        }
        s_wifi_list_count = added;
        if (added == 0) {
            wifi_set_selected(NULL, NULL);
        }
        if (added > 0) {
            wifi_set_selected(s_wifi_list_items[0].btn, s_wifi_list_items[0].ssid);
            if (s_group && s_wifi_list_items[0].btn) {
                lv_group_focus_obj(s_wifi_list_items[0].btn);
            }
        }
        char status[64];
        snprintf(status, sizeof(status), "Scan done: %u found", (unsigned)added);
        lvgl_ui_wifi_set_status(status);
        wifi_refresh_group();
    } else {
        lvgl_ui_wifi_set_status("Scan failed");
    }
    wifi_scan_cleanup_async(NULL);
    free(res);
}

static void wifi_scan_task(void *arg)
{
    (void)arg;
    wifi_scan_result_t *res = (wifi_scan_result_t *)calloc(1, sizeof(wifi_scan_result_t));
    if (!res) {
        lvgl_ui_wifi_set_status("Scan OOM");
        (void)lvgl_ui_async_call(wifi_scan_cleanup_async, NULL);
        vTaskDelete(NULL);
        return;
    }
    bool ok = wifi_scan_networks(res->recs, WIFI_MAX_LIST_ITEMS, &res->count);
    if (!ok) {
        free(res);
        (void)lvgl_ui_async_call(wifi_apply_scan_result, NULL);
        vTaskDelete(NULL);
        return;
    }
    if (!lvgl_ui_async_call(wifi_apply_scan_result, res)) {
        free(res);
        lvgl_ui_wifi_set_status("Scan failed");
        (void)lvgl_ui_async_call(wifi_scan_cleanup_async, NULL);
    }
    vTaskDelete(NULL);
}

static void wifi_pass_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED || (code == LV_EVENT_KEY && lv_event_get_key(e) == LV_KEY_ENTER)) {
        if (s_wifi_keyboard) {
            if (!lv_obj_has_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN)) {
                lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
                wifi_refresh_group();
                if (s_group && s_wifi_pass_ta) {
                    lv_group_focus_obj(s_wifi_pass_ta);
                }
            } else {
                lv_obj_clear_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
                lv_keyboard_set_textarea(s_wifi_keyboard, s_wifi_pass_ta);
                wifi_refresh_group();
                if (s_group) {
                    lv_group_focus_obj(s_wifi_keyboard);
                }
            }
        }
    }
}

static void wifi_keyboard_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (s_wifi_keyboard) {
            lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
            lv_keyboard_set_textarea(s_wifi_keyboard, NULL);
        }
        wifi_refresh_group();
        if (s_group && s_wifi_pass_ta) {
            lv_group_focus_obj(s_wifi_pass_ta);
        }
    }
}

static lv_obj_t *create_wifi_button(lv_obj_t *parent, const char *text, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 28);
    lv_obj_set_width(btn, 48);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_pad_hor(btn, 10, 0);
    lv_obj_set_style_pad_ver(btn, 5, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(btn);
    return btn;
}

static void wifi_connect_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    /* 若开机后台正在自动启 AP，用户选择 STA 连接时应取消该任务，避免并发切换导致连接不稳定 */
    wifi_ap_autostart_cancel_request();

    const char *ssid = wifi_ssid_from_btn(s_wifi_selected_item);
    if (!ssid || ssid[0] == '\0') {
        lvgl_ui_show_hint("Select WiFi");
        return;
    }
    const char *pwd = s_wifi_pass_ta ? lv_textarea_get_text(s_wifi_pass_ta) : "";
    s_wifi_sta_ok = false;
    if (!wifi_connect_sta(ssid, pwd)) {
        lvgl_ui_wifi_set_status("Connect failed");
        return;
    }
    lvgl_ui_wifi_set_status("Connecting...");
}

static void wifi_scan_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (s_wifi_scanning) {
        return;
    }

    /* 扫描需要切到 STA 模式：取消后台自动启 AP，避免并发切换 */
    wifi_ap_autostart_cancel_request();
    s_wifi_scanning = true;
    if (s_wifi_btn_scan) {
        lv_obj_add_state(s_wifi_btn_scan, LV_STATE_DISABLED);
    }
    if (s_wifi_btn_connect) {
        lv_obj_add_state(s_wifi_btn_connect, LV_STATE_DISABLED);
    }
    if (s_wifi_btn_enter) {
        lv_obj_add_state(s_wifi_btn_enter, LV_STATE_DISABLED);
    }
    lvgl_ui_wifi_set_status("Scanning...");
    xTaskCreate(wifi_scan_task, "wifi_scan", 4096, NULL, 4, NULL);
}

static void wifi_enter_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    if (s_wifi_scanning) {
        lvgl_ui_show_hint("Scanning...");
        return;
    }

    /* 用户已主动操作：取消后台自动启 AP，进入流程由当前线程显式保证 Wi-Fi 状态 */
    wifi_ap_autostart_cancel_request();

    bool ok = true;
    if (s_wifi_sta_ok) {
        lvgl_ui_wifi_set_status("STA ready");
    } else {
        /* 关键：即便当前已在 AP，也做一次“停-启”以消除开机后台任务/并发切换带来的不稳定状态
         * （现象：不点“扫描”直接“进入”，主界面任务偶发无法启动；点一次“扫描”后恢复正常）
         */
        ok = wifi_start_ap_mode();
        lvgl_ui_wifi_set_status(ok ? "AP mode ready" : "AP start failed");
    }
    if (s_wifi_keyboard) {
        lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if (ok) {
        wifi_sta_auto_enter_cancel();
        ui_load_main_screen();
    }
}

static void ui_create_wifi_screen(void)
{
    if (s_wifi_screen != NULL) {
        lv_scr_load(s_wifi_screen);
        s_is_main_screen_active = false;
        wifi_refresh_group();
        return;
    }

    const lv_coord_t disp_w = lv_disp_get_hor_res(NULL);
    const lv_coord_t disp_h = lv_disp_get_ver_res(NULL);
    const lv_coord_t margin_x = 8;
    const lv_coord_t title_y = 6;
    const lv_coord_t list_y = 36;
    const lv_coord_t list_h = 118;
    const lv_coord_t row_gap = 8;
    const lv_coord_t row_h = 22;
    const lv_coord_t pin_h = 30;
    const lv_coord_t btn_y = 238;
    const lv_coord_t status_y = 272;
    const lv_coord_t key_w = 36;
    const lv_coord_t colon_w = 8;
    lv_coord_t value_w = disp_w - margin_x * 2 - key_w - colon_w;
    if (value_w < 0) value_w = 0;

    s_wifi_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_wifi_screen, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_wifi_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_wifi_screen, 0, 0);
    lv_obj_set_scrollbar_mode(s_wifi_screen, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *top = lv_obj_create(s_wifi_screen);
    lv_obj_remove_style_all(top);
    lv_obj_set_size(top, disp_w, disp_h);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(top, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *title = lv_label_create(top);
    lv_label_set_text(title, "WiFi-Connect");
    lv_obj_set_style_text_color(title, lv_color_hex(0x2A7FFF), 0);
    lv_obj_set_width(title, disp_w);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, title_y);

    s_wifi_list_panel = lv_obj_create(top);
    lv_obj_remove_style_all(s_wifi_list_panel);
    lv_obj_set_size(s_wifi_list_panel, disp_w - margin_x * 2, list_h);
    lv_obj_set_pos(s_wifi_list_panel, margin_x, list_y);
    lv_obj_set_style_bg_color(s_wifi_list_panel, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_wifi_list_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_wifi_list_panel, 2, 0);
    lv_obj_set_style_border_color(s_wifi_list_panel, lv_color_hex(0xC8CDD5), 0);
    lv_obj_set_style_radius(s_wifi_list_panel, 6, 0);
    lv_obj_set_style_pad_all(s_wifi_list_panel, 0, 0);
    lv_obj_set_scrollbar_mode(s_wifi_list_panel, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_flex_flow(s_wifi_list_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_wifi_list_panel,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    const lv_coord_t ssid_y = list_y + list_h + row_gap;
    lv_obj_t *ssid_row = lv_obj_create(top);
    lv_obj_remove_style_all(ssid_row);
    lv_obj_set_size(ssid_row, disp_w - margin_x * 2, row_h);
    lv_obj_set_pos(ssid_row, margin_x, ssid_y);
    lv_obj_clear_flag(ssid_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(ssid_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ssid_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(ssid_row, 0, 0);
    lv_obj_t *ssid_key = lv_label_create(ssid_row);
    lv_label_set_text(ssid_key, "SSID");
    lv_obj_set_width(ssid_key, key_w);
    lv_obj_set_style_text_align(ssid_key, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ssid_colon = lv_label_create(ssid_row);
    lv_label_set_text(ssid_colon, ":");
    lv_obj_set_width(ssid_colon, colon_w);
    lv_obj_set_style_text_align(ssid_colon, LV_TEXT_ALIGN_CENTER, 0);

    s_wifi_ssid_value = lv_label_create(ssid_row);
    lv_label_set_text(s_wifi_ssid_value, "******");
    lv_obj_set_width(s_wifi_ssid_value, value_w);
    lv_obj_set_style_text_align(s_wifi_ssid_value, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(s_wifi_ssid_value, LV_LABEL_LONG_DOT);

    const lv_coord_t pin_y = ssid_y + row_h + row_gap;
    lv_obj_t *pin_row = lv_obj_create(top);
    lv_obj_remove_style_all(pin_row);
    lv_obj_set_size(pin_row, disp_w - margin_x * 2, pin_h);
    lv_obj_set_pos(pin_row, margin_x, pin_y);
    lv_obj_clear_flag(pin_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(pin_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pin_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(pin_row, 0, 0);

    lv_obj_t *pin_key = lv_label_create(pin_row);
    lv_label_set_text(pin_key, "PIN");
    lv_obj_set_width(pin_key, key_w);
    lv_obj_set_style_text_align(pin_key, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *pin_colon = lv_label_create(pin_row);
    lv_label_set_text(pin_colon, ":");
    lv_obj_set_width(pin_colon, colon_w);
    lv_obj_set_style_text_align(pin_colon, LV_TEXT_ALIGN_CENTER, 0);

    s_wifi_pass_ta = lv_textarea_create(pin_row);
    lv_textarea_set_password_mode(s_wifi_pass_ta, true);
    lv_textarea_set_one_line(s_wifi_pass_ta, true);
    lv_obj_set_width(s_wifi_pass_ta, value_w);
    lv_obj_set_height(s_wifi_pass_ta, pin_h);
    lv_obj_set_scrollbar_mode(s_wifi_pass_ta, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(s_wifi_pass_ta, LV_DIR_NONE);
    lv_obj_set_style_text_align(s_wifi_pass_ta, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lv_textarea_get_label(s_wifi_pass_ta), LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_event_cb(s_wifi_pass_ta, wifi_pass_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(s_wifi_pass_ta);

    lv_obj_t *btn_row = lv_obj_create(top);
    lv_obj_remove_style_all(btn_row);
    lv_obj_set_size(btn_row, disp_w, 30);
    lv_obj_set_pos(btn_row, 0, btn_y);
    lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(btn_row, 8, 0);

    s_wifi_btn_scan    = create_wifi_button(btn_row, "扫描", wifi_scan_btn_event_cb);
    s_wifi_btn_connect = create_wifi_button(btn_row, "连接", wifi_connect_btn_event_cb);
    s_wifi_btn_enter   = create_wifi_button(btn_row, "进入", wifi_enter_btn_event_cb);

    s_wifi_status_label = lv_label_create(top);
    lv_label_set_text(s_wifi_status_label, "");
    lv_obj_set_style_text_color(s_wifi_status_label, lv_color_hex(0x888888), 0);
    lv_obj_set_width(s_wifi_status_label, disp_w);
    lv_obj_set_style_text_align(s_wifi_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(s_wifi_status_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_wifi_status_label, 0, status_y);

    s_wifi_keyboard = lv_keyboard_create(s_wifi_screen);
    lv_obj_set_size(s_wifi_keyboard, LV_PCT(100), 96);
    lv_obj_align(s_wifi_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_wifi_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_keyboard_set_textarea(s_wifi_keyboard, s_wifi_pass_ta);
    lv_obj_add_event_cb(s_wifi_keyboard, wifi_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    apply_focus_border_style(s_wifi_keyboard);

    lv_scr_load(s_wifi_screen);
    s_is_main_screen_active = false;
    wifi_refresh_group();
}


static lv_obj_t *find_open_dropdown_for_keypad(lv_obj_t *focused_obj)
{
    lv_obj_t *const dropdowns[] = {
        s_dropdown_task,
        s_dd_color_profile,
        s_dd_tag_dict,
        s_dd_ai_model,
        s_dd_ai_score,
        s_dd_stream_mode,
        s_dd_fill_light,
        s_dd_frame_overlay,
        s_dd_msroi,
        s_dd_m1_coarse_mode,
        s_dd_m2_fallback,
        s_dd_m3_roi_refine,
    };

    for (size_t i = 0; i < sizeof(dropdowns) / sizeof(dropdowns[0]); ++i) {
        lv_obj_t *dd = dropdowns[i];
        if (!dd || !lv_dropdown_is_open(dd)) continue;
        lv_obj_t *list = lv_dropdown_get_list(dd);
        if (focused_obj == dd || (list && focused_obj == list)) {
            return dd;
        }
    }

    /* 兜底：下拉已展开但焦点落在其它对象（例如 list/遮罩） */
    for (size_t i = 0; i < sizeof(dropdowns) / sizeof(dropdowns[0]); ++i) {
        lv_obj_t *dd = dropdowns[i];
        if (dd && lv_dropdown_is_open(dd)) {
            return dd;
        }
    }

    return NULL;
}

/* 3 个硬件按键映射到 LVGL keypad：
 * KEY1 -> 上/左（上一控件）
 * KEY2 -> 下/右（下一控件）
 * KEY3 -> 确认/点击
 *
 * 这样虽然物理上只有两个“方向键”，但依靠 LVGL 的焦点顺序，
 * 可以在所有控件间前后移动，相当于统一了上下左右的导航。 */
static void keypad_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    LV_UNUSED(drv);

    static uint32_t last_key     = 0;
    static bool     last_pressed = false;

    uint8_t k = key_scan(0);
    if (k == 0) {
        /* 没有新按键：如果上一次是按下状态，产生一次“释放” */
        data->state = LV_INDEV_STATE_RELEASED;
        data->key   = last_pressed ? last_key : 0;
        last_pressed = false;
        return;
    }

    lv_group_t *active_group = NULL;
    if (s_keypad) active_group = s_keypad->group;
    if (!active_group) active_group = s_group;

    bool dropdown_open = false;
    lv_obj_t *focused_obj = NULL;
    if (active_group != NULL) {
        focused_obj = lv_group_get_focused(active_group);
    }

    if (s_upload_overlay == NULL && active_group == s_group) {
        lv_obj_t *open_dd = find_open_dropdown_for_keypad(focused_obj);
        if (open_dd) {
            dropdown_open = true;
        }
    }

    bool focus_keyboard = (focused_obj == s_wifi_keyboard) &&
                          s_wifi_keyboard &&
                          !lv_obj_has_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);

    uint32_t lv_key = 0;
    if (k == KEY1_PRES) {
        lv_key = focus_keyboard ? LV_KEY_LEFT : (dropdown_open ? LV_KEY_UP : LV_KEY_PREV);
    } else if (k == KEY2_PRES) {
        lv_key = focus_keyboard ? LV_KEY_RIGHT : (dropdown_open ? LV_KEY_DOWN : LV_KEY_NEXT);
    } else if (k == KEY3_PRES) {
        lv_key = LV_KEY_ENTER;
    }

    /* 注意：不要在这里调用 lv_dropdown_set_selected 做“首尾循环”。
     * lv_dropdown_set_selected 会同时修改 sel_opt_id 与 sel_opt_id_orig（等同于“直接确认”），
     * 会导致用户按确认关闭时不再触发 LV_EVENT_VALUE_CHANGED，进而出现 UI 显示与实际状态不同步。 */

    if (lv_key != 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key   = lv_key;
        last_key    = lv_key;
        last_pressed = true;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->key   = 0;
        last_pressed = false;
    }
}

/**********************
 *  事件回调
 **********************/

/* 下拉框：任务选择 */
static void dropdown_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *dd = lv_event_get_current_target(e);

    if (s_programmatic_dropdown && code == LV_EVENT_VALUE_CHANGED) {
        return;
    }

    /* 主界面任务下拉展开时暂停 LCD 刷新，避免被视频覆盖 */
    if (dd == s_dropdown_task &&
        (code == LV_EVENT_PRESSED || code == LV_EVENT_CLICKED)) {
        TaskScheduling_StopForDropdown();
        if (!s_dropdown_paused_lcd) {
            lcd_task_suspend();
            s_dropdown_paused_lcd = true;
            /* 触发一次 LVGL 重绘，覆盖暂停前的残留视频 */
            lv_obj_invalidate(lv_scr_act());
            lv_refr_now(NULL);
        }
    }

    /* 取消/失焦时恢复 */
    if (code == LV_EVENT_CANCEL || code == LV_EVENT_DEFOCUSED) {
        resume_lcd_if_needed(dd);
        return;
    }

    if (code == LV_EVENT_KEY) {
        uint32_t key = lv_event_get_key(e);

        if (dd == s_dropdown_task) {
            bool open = lv_dropdown_is_open(dd);
            if (!open) {
                /* 通过键盘打开时也先暂停 LCD，避免列表被预览遮挡 */
                if (key == LV_KEY_ENTER || key == LV_KEY_DOWN || key == LV_KEY_UP ||
                    key == LV_KEY_LEFT  || key == LV_KEY_RIGHT) {
                    TaskScheduling_StopForDropdown();
                    if (!s_dropdown_paused_lcd) {
                        lcd_task_suspend();
                        s_dropdown_paused_lcd = true;
                        lv_obj_invalidate(lv_scr_act());
                        lv_refr_now(NULL);
                    }
                }
            } else if (key == LV_KEY_ENTER) {
                uint16_t sel = lv_dropdown_get_selected(dd);
                bool unchanged = (sel == s_sel_task);
                s_sel_task = sel;
                if (unchanged) {
                    TaskScheduling_RequestRestartSelected(task_from_index(sel));
                }
                /* 用户按确认关闭下拉框，即使未更改选项也要重启当前任务 */
                lv_async_call(resume_lcd_async, dd);
            }
        }
    }

    if (code != LV_EVENT_VALUE_CHANGED) return;

    char buf[48] = {0};

    lv_dropdown_get_selected_str(dd, buf, sizeof(buf));
    const char *name = "Dropdown";
    if (dd == s_dropdown_task)        name = "Task";
    else if (dd == s_dd_color_profile) name = "ColorProfile";
    else if (dd == s_dd_tag_dict)      name = "TagDict";
    else if (dd == s_dd_ai_model)      name = "AIModel";
    else if (dd == s_dd_ai_score)      name = "AI-Score";
    else if (dd == s_dd_stream_mode)   name = "StreamMode";
    else if (dd == s_dd_fill_light)    name = "FillLight";
    else if (dd == s_dd_frame_overlay) name = "FrameOverlay";
    else if (dd == s_dd_msroi)         name = "MSROI";
    else if (dd == s_dd_m1_coarse_mode) name = "MSROI-M1";
    else if (dd == s_dd_m2_fallback)    name = "MSROI-M2";
    else if (dd == s_dd_m3_roi_refine)  name = "MSROI-M3";

    ESP_LOGI(TAG_UI, "%s selected: %s", name, buf);

    /* 记录设置页的选择，便于返回后保持记忆 */
    if (dd == s_dropdown_task) {
        s_sel_task = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_color_profile) {
        s_sel_color_profile = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_tag_dict) {
        s_sel_tag_dict = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_ai_model) {
        s_sel_ai_model = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_ai_score) {
        s_sel_ai_score = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_stream_mode) {
        s_sel_stream_mode = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_fill_light) {
        s_sel_fill_light = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_frame_overlay) {
        s_sel_frame_overlay = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_msroi) {
        s_sel_msroi = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_m1_coarse_mode) {
        s_sel_m1_coarse_mode = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_m2_fallback) {
        s_sel_m2_fallback = lv_dropdown_get_selected(dd);
    } else if (dd == s_dd_m3_roi_refine) {
        s_sel_m3_roi_refine = lv_dropdown_get_selected(dd);
    }

    /* 解耦：仅对“即时生效”的设置执行硬件动作 */
    if (dd == s_dd_fill_light) {
        apply_led_setting();
    } else if (dd == s_dd_frame_overlay) {
        apply_frame_overlay_setting();
    } else if (dd == s_dd_msroi ||
               dd == s_dd_m1_coarse_mode ||
               dd == s_dd_m2_fallback ||
               dd == s_dd_m3_roi_refine) {
        apply_msroi_setting();
        apply_msroi_module_settings();
    }

    /* 选定后关闭下拉并恢复 LCD */
    if (dd == s_dropdown_task) {
        lv_dropdown_close(dd);
        TaskScheduling_RequestRestartSelected(task_from_index(s_sel_task));
    }
    resume_lcd_if_needed(dd);
}

/* 顶部“控制”按钮：进入新页面 */
static void back_btn_event_cb(lv_event_t *e);

static lv_obj_t *create_setting_dropdown(lv_obj_t *parent,
                                         const char *options)
{
    /* 直接使用父容器的列布局，减少额外空白 */
    lv_obj_t *dd = lv_dropdown_create(parent);
    lv_dropdown_set_dir(dd, LV_DIR_BOTTOM);
    lv_dropdown_set_options(dd, options);
    lv_obj_set_style_text_font(dd, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_t *list = lv_dropdown_get_list(dd);
    if (list) {
        lv_obj_set_style_text_font(list, &lv_font_source_han_sans_sc_14_system, 0);
    }
    lv_obj_set_width(dd, LV_PCT(100));
    lv_obj_add_event_cb(dd, dropdown_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(dd);

    return dd;
}

static void refresh_settings_dropdowns(void)
{
    const char *tag_opts    = "未设置AprilTag类别\nTag16h5\nTag36h11";
    const char *score_opts  = "未设置AI检测阈值\nAI-Score-0.25\nAI-Score-0.30\nAI-Score-0.40\nAI-Score-0.50\nAI-Score-0.60\nAI-Score-0.70\nAI-Score-0.80";
    const char *stream_opts = "未设置图传模式\n单帧图传\n视频流图传";
    const char *fill_opts   = "未设置补光灯状态\n补光灯开启\n补光灯关闭";
    const char *frame_opts  = "未设置检测框状态\n检测框开启\n检测框关闭";
    const char *msroi_opts  = "未设置分层检测\n开启\n关闭";
    const char *module_opts = "未设置模块\n开启\n关闭";

    vision_msroi_config_t msroi_cfg = {0};
    vision_msroi_get_config(&msroi_cfg);

    char color_opts[192];
    build_color_profile_options(color_opts, sizeof(color_opts));
    char *model_opts = sd_list_models_alloc();

    if (s_dd_color_profile) {
        lv_dropdown_set_options(s_dd_color_profile, color_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_color_profile);
        if (cnt > 0 && s_sel_color_profile >= cnt) {
            s_sel_color_profile = cnt - 1;
        }
        lv_dropdown_set_selected(s_dd_color_profile, s_sel_color_profile);
    }
    if (s_dd_tag_dict) {
        lv_dropdown_set_options(s_dd_tag_dict, tag_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_tag_dict);
        if (cnt > 0 && s_sel_tag_dict >= cnt) {
            s_sel_tag_dict = cnt - 1;
        }
        lv_dropdown_set_selected(s_dd_tag_dict, s_sel_tag_dict);
    }
    if (s_dd_ai_model) {
        lv_dropdown_set_options(s_dd_ai_model, model_opts ? model_opts : "未设置AI模型");
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_ai_model);
        if (cnt > 0 && s_sel_ai_model >= cnt) {
            s_sel_ai_model = (cnt > 0) ? cnt - 1 : 0;
        }
        lv_dropdown_set_selected(s_dd_ai_model, s_sel_ai_model);
    }
    if (model_opts) {
        free(model_opts);
    }
    if (s_dd_ai_score) {
        lv_dropdown_set_options(s_dd_ai_score, score_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_ai_score);
        if (cnt > 0 && s_sel_ai_score >= cnt) {
            s_sel_ai_score = 0; // 默认 None
        }
        lv_dropdown_set_selected(s_dd_ai_score, s_sel_ai_score);
    }
    if (s_dd_stream_mode) {
        lv_dropdown_set_options(s_dd_stream_mode, stream_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_stream_mode);
        if (cnt > 0 && s_sel_stream_mode >= cnt) {
            s_sel_stream_mode = cnt - 1;
        }
        lv_dropdown_set_selected(s_dd_stream_mode, s_sel_stream_mode);
    }
    if (s_dd_fill_light) {
        lv_dropdown_set_options(s_dd_fill_light, fill_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_fill_light);
        if (cnt > 0 && s_sel_fill_light >= cnt) {
            s_sel_fill_light = 0; // 默认 None
        }
        lv_dropdown_set_selected(s_dd_fill_light, s_sel_fill_light);
    }
    if (s_dd_frame_overlay) {
        lv_dropdown_set_options(s_dd_frame_overlay, frame_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_frame_overlay);
        if (cnt > 0 && s_sel_frame_overlay >= cnt) {
            s_sel_frame_overlay = 0; // 默认 None
        }
        lv_dropdown_set_selected(s_dd_frame_overlay, s_sel_frame_overlay);
    }
    if (s_dd_msroi) {
        lv_dropdown_set_options(s_dd_msroi, msroi_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_msroi);
        if (cnt > 0 && s_sel_msroi >= cnt) {
            s_sel_msroi = vision_msroi_is_enabled() ? 1 : 2;
        }
        if (s_sel_msroi == 0) {
            s_sel_msroi = vision_msroi_is_enabled() ? 1 : 2;
        }
        if (cnt > 0 && s_sel_msroi >= cnt) {
            s_sel_msroi = 0;
        }
        lv_dropdown_set_selected(s_dd_msroi, s_sel_msroi);
    }
    if (s_dd_m1_coarse_mode) {
        lv_dropdown_set_options(s_dd_m1_coarse_mode, module_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_m1_coarse_mode);
        if (cnt > 0 && s_sel_m1_coarse_mode >= cnt) {
            s_sel_m1_coarse_mode = msroi_cfg.module_coarse_mode ? 1 : 2;
        }
        if (s_sel_m1_coarse_mode == 0) {
            s_sel_m1_coarse_mode = msroi_cfg.module_coarse_mode ? 1 : 2;
        }
        lv_dropdown_set_selected(s_dd_m1_coarse_mode, s_sel_m1_coarse_mode);
    }
    if (s_dd_m2_fallback) {
        lv_dropdown_set_options(s_dd_m2_fallback, module_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_m2_fallback);
        if (cnt > 0 && s_sel_m2_fallback >= cnt) {
            s_sel_m2_fallback = msroi_cfg.module_valid_fallback ? 1 : 2;
        }
        if (s_sel_m2_fallback == 0) {
            s_sel_m2_fallback = msroi_cfg.module_valid_fallback ? 1 : 2;
        }
        lv_dropdown_set_selected(s_dd_m2_fallback, s_sel_m2_fallback);
    }
    if (s_dd_m3_roi_refine) {
        lv_dropdown_set_options(s_dd_m3_roi_refine, module_opts);
        uint16_t cnt = lv_dropdown_get_option_cnt(s_dd_m3_roi_refine);
        if (cnt > 0 && s_sel_m3_roi_refine >= cnt) {
            s_sel_m3_roi_refine = msroi_cfg.module_roi_refine ? 1 : 2;
        }
        if (s_sel_m3_roi_refine == 0) {
            s_sel_m3_roi_refine = msroi_cfg.module_roi_refine ? 1 : 2;
        }
        lv_dropdown_set_selected(s_dd_m3_roi_refine, s_sel_m3_roi_refine);
    }

    apply_runtime_settings();
}

static void apply_led_setting(void)
{
    lvgl_ui_led_mode_t mode = (lvgl_ui_led_mode_t)s_sel_fill_light;
    switch (mode) {
    case LVGL_UI_LED_ON:
        bled_on();
        break;
    case LVGL_UI_LED_OFF:
    case LVGL_UI_LED_NONE:
    default:
        bled_off();
        break;
    }
}

static void apply_frame_overlay_setting(void)
{
    lvgl_ui_frame_mode_t mode = (lvgl_ui_frame_mode_t)s_sel_frame_overlay;
    switch (mode) {
    case LVGL_UI_FRAME_ON:
        lcd_set_overlay_enabled(true);
        break;
    case LVGL_UI_FRAME_OFF:
    case LVGL_UI_FRAME_NONE:
    default:
        lcd_set_overlay_enabled(false);
        break;
    }
}

static void apply_msroi_setting(void)
{
    bool enable = false;
    if (s_sel_msroi == 1) {
        enable = true;
    } else if (s_sel_msroi == 2) {
        enable = false;
    } else {
        enable = vision_msroi_is_enabled();
    }
    vision_msroi_set_enabled(enable);
}

static void apply_msroi_module_settings(void)
{
    bool m1 = true;
    bool m2 = true;
    bool m3 = true;

    if (s_sel_m1_coarse_mode == 1) m1 = true;
    else if (s_sel_m1_coarse_mode == 2) m1 = false;

    if (s_sel_m2_fallback == 1) m2 = true;
    else if (s_sel_m2_fallback == 2) m2 = false;

    if (s_sel_m3_roi_refine == 1) m3 = true;
    else if (s_sel_m3_roi_refine == 2) m3 = false;

    vision_msroi_set_modules(m1, m2, m3);
}

static void apply_runtime_settings(void)
{
    apply_led_setting();
    apply_frame_overlay_setting();
    apply_msroi_setting();
    apply_msroi_module_settings();
}

static void refresh_about_info(void)
{
    if (!s_about_info_label) {
        return;
    }

    ensure_nvs_ready();
    (void)device_identity_init();

    char sn[64] = {0};
    esp_err_t sn_err = device_identity_get_sn(sn, sizeof(sn));
    if (sn_err != ESP_OK) {
        device_identity_status_t st = device_identity_get_status();
        if (st == DEVICE_IDENTITY_STATUS_MISMATCH) {
            snprintf(sn, sizeof(sn), "INVALID(MISMATCH)");
        } else if (st == DEVICE_IDENTITY_STATUS_CORRUPT) {
            snprintf(sn, sizeof(sn), "INVALID(CORRUPT)");
        } else {
            snprintf(sn, sizeof(sn), "-");
        }
    }

    char mac[32] = {0};
    if (device_identity_get_base_mac_str(mac, sizeof(mac)) != ESP_OK) {
        snprintf(mac, sizeof(mac), "-");
    }

    const char *oem = device_identity_get_oem();
    if (!oem || oem[0] == '\0') {
        oem = "-";
    }

    const char *fw_ver = device_identity_get_fw_version();
    if (!fw_ver || fw_ver[0] == '\0') {
        fw_ver = "-";
    }

    wifi_ui_info_t info;
    bool wifi_ok = wifi_ui_get_info(&info);

    const char *sta_ssid = (wifi_ok && info.sta_ssid[0]) ? info.sta_ssid : "-";
    const char *sta_ip   = (wifi_ok && info.sta_ip[0]) ? info.sta_ip : "-";
    const char *ap_ssid  = (wifi_ok && info.ap_ssid[0]) ? info.ap_ssid : "-";
    const char *ap_ip    = (wifi_ok && info.ap_ip[0]) ? info.ap_ip : "-";
    const char *ap_pass  = (wifi_ok && info.ap_password[0]) ? info.ap_password : "-";

    char buf[384];
    snprintf(buf, sizeof(buf),
             "热点名称: %s\n"
             "STA IP: %s\n"
             "AP SSID: %s\n"
             "AP IP: %s\n"
             "AP 密码: %s\n"
             "OEM: %s\n"
             "SN: %s\n"
             "MAC: %s\n"
             "固件版本: %s",
             sta_ssid, sta_ip, ap_ssid, ap_ip, ap_pass, oem, sn, mac, fw_ver);
    lv_label_set_text(s_about_info_label, buf);
}

static void ensure_about_screen_created(void)
{
    if (s_about_screen != NULL) {
        return;
    }

    s_about_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_about_screen, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_about_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_about_screen, 6, 0);
    lv_obj_set_flex_flow(s_about_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_about_screen,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_about_screen, 4, 0);
    lv_obj_set_scrollbar_mode(s_about_screen, LV_SCROLLBAR_MODE_ACTIVE);

    lv_obj_t *title = lv_label_create(s_about_screen);
    lv_label_set_text(title, "关于");
    lv_obj_set_style_text_font(title, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x2A7FFF), 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_letter_space(title, 1, 0);
    lv_obj_set_style_bg_color(title, lv_color_hex(0xEAF2FF), 0);
    lv_obj_set_style_bg_opa(title, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(title, 12, 0);
    lv_obj_set_style_pad_hor(title, 12, 0);
    lv_obj_set_style_pad_ver(title, 6, 0);

    /* 额外的间隔，避免标题与正文贴太近 */
    lv_obj_t *title_spacer = lv_obj_create(s_about_screen);
    lv_obj_remove_style_all(title_spacer);
    lv_obj_set_size(title_spacer, LV_PCT(100), 4);

    s_about_info_label = lv_label_create(s_about_screen);
    lv_obj_set_style_text_font(s_about_info_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_about_info_label, lv_color_hex(0x333333), 0);
    lv_obj_set_width(s_about_info_label, LV_PCT(100));
    lv_label_set_long_mode(s_about_info_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_about_info_label, "");

    s_about_btn_back = lv_btn_create(s_about_screen);
    lv_obj_set_style_radius(s_about_btn_back, 10, 0);
    lv_obj_set_style_bg_color(s_about_btn_back, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(s_about_btn_back, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_about_btn_back, 0, 0);
    lv_obj_set_style_pad_hor(s_about_btn_back, 14, 0);
    lv_obj_set_style_pad_ver(s_about_btn_back, 6, 0);
    lv_obj_add_event_cb(s_about_btn_back, back_btn_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(s_about_btn_back);
    lv_obj_t *lbl_back = lv_label_create(s_about_btn_back);
    lv_label_set_text(lbl_back, "返回");
    lv_obj_set_style_text_font(lbl_back, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_center(lbl_back);
}


static void ensure_control_screen_created(void)
{
    if (s_control_screen != NULL) {
        return;
    }

    s_control_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_control_screen, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_control_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_control_screen, 6, 0);
    lv_obj_set_flex_flow(s_control_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_control_screen,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_control_screen, 4, 0);
    lv_obj_set_scrollbar_mode(s_control_screen, LV_SCROLLBAR_MODE_ACTIVE);

    lv_obj_t *title = lv_label_create(s_control_screen);
    lv_label_set_text(title, "TaskSetting");
    lv_obj_set_style_text_color(title, lv_color_hex(0x2A7FFF), 0);
#if LV_FONT_MONTSERRAT_18
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
#else
    lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
#endif
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_letter_space(title, 1, 0);
    lv_obj_set_style_bg_color(title, lv_color_hex(0xEAF2FF), 0);
    lv_obj_set_style_bg_opa(title, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(title, 12, 0);
    lv_obj_set_style_pad_hor(title, 12, 0);
    lv_obj_set_style_pad_ver(title, 6, 0);

    /* 额外的间隔，让标题和下拉区保持距离，替代 margin 底部效果 */
    lv_obj_t *title_spacer = lv_obj_create(s_control_screen);
    lv_obj_remove_style_all(title_spacer);
    lv_obj_set_size(title_spacer, LV_PCT(100), 4);

    s_dd_color_profile = create_setting_dropdown(s_control_screen, "未设置颜色ID");
    s_dd_tag_dict      = create_setting_dropdown(s_control_screen, "未设置AprilTag类别");
    s_dd_ai_model      = create_setting_dropdown(s_control_screen, "未设置AI模型");
    s_dd_ai_score      = create_setting_dropdown(s_control_screen, "未设置AI检测阈值");
    s_dd_stream_mode   = create_setting_dropdown(s_control_screen, "未设置图传模式");
    s_dd_fill_light    = create_setting_dropdown(s_control_screen, "未设置补光灯状态");
    s_dd_frame_overlay = create_setting_dropdown(s_control_screen, "未设置检测框状态");
    s_dd_msroi         = create_setting_dropdown(s_control_screen, "未设置分层检测");
    s_dd_m1_coarse_mode = create_setting_dropdown(s_control_screen, "未设置模块1-粗检判别");
    s_dd_m2_fallback    = create_setting_dropdown(s_control_screen, "未设置模块2-回退机制");
    s_dd_m3_roi_refine  = create_setting_dropdown(s_control_screen, "未设置模块3-ROI精检");

    s_btn_back = lv_btn_create(s_control_screen);
    lv_obj_set_style_radius(s_btn_back, 10, 0);
    lv_obj_set_style_bg_color(s_btn_back, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(s_btn_back, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_btn_back, 0, 0);
    lv_obj_set_style_pad_hor(s_btn_back, 14, 0);
    lv_obj_set_style_pad_ver(s_btn_back, 6, 0);
    lv_obj_add_event_cb(s_btn_back, back_btn_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(s_btn_back);
    lv_obj_t *lbl_back = lv_label_create(s_btn_back);
    lv_label_set_text(lbl_back, "返回");
    lv_obj_set_style_text_font(lbl_back, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_center(lbl_back);

    refresh_settings_dropdowns();
}
static void control_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    ESP_LOGI(TAG_UI, "Control button pressed, enter new page");

    TaskScheduling_RequestPauseForSettings();
    ensure_control_screen_created();
    refresh_settings_dropdowns();
    lv_scr_load(s_control_screen);
    s_is_main_screen_active = false;
    if (s_group != NULL) {
        lv_group_remove_all_objs(s_group);
        if (s_dd_color_profile) lv_group_add_obj(s_group, s_dd_color_profile);
        if (s_dd_tag_dict)      lv_group_add_obj(s_group, s_dd_tag_dict);
        if (s_dd_ai_model)      lv_group_add_obj(s_group, s_dd_ai_model);
        if (s_dd_ai_score)      lv_group_add_obj(s_group, s_dd_ai_score);
        if (s_dd_stream_mode)   lv_group_add_obj(s_group, s_dd_stream_mode);
        if (s_dd_fill_light)    lv_group_add_obj(s_group, s_dd_fill_light);
        if (s_dd_frame_overlay) lv_group_add_obj(s_group, s_dd_frame_overlay);
        if (s_dd_msroi)         lv_group_add_obj(s_group, s_dd_msroi);
        if (s_dd_m1_coarse_mode) lv_group_add_obj(s_group, s_dd_m1_coarse_mode);
        if (s_dd_m2_fallback)    lv_group_add_obj(s_group, s_dd_m2_fallback);
        if (s_dd_m3_roi_refine)  lv_group_add_obj(s_group, s_dd_m3_roi_refine);
        if (s_btn_back)         lv_group_add_obj(s_group, s_btn_back);

        if (s_dd_color_profile) {
            lv_group_focus_obj(s_dd_color_profile);
        }
    }
}

static void back_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    ESP_LOGI(TAG_UI, "Back to main UI");
    if (s_main_screen != NULL) {
        lv_scr_load(s_main_screen);
        s_is_main_screen_active = true;
    }
    fw_update_notify_main_screen_entered();
    TaskScheduling_RequestResumeAfterSettings();

    if (s_group != NULL) {
        lv_group_remove_all_objs(s_group);
        if (s_dropdown_task) lv_group_add_obj(s_group, s_dropdown_task);
        if (s_btn_control)   lv_group_add_obj(s_group, s_btn_control);
        if (s_btn_open)      lv_group_add_obj(s_group, s_btn_open);
        if (s_btn_close)     lv_group_add_obj(s_group, s_btn_close);
        if (s_btn_web)       lv_group_add_obj(s_group, s_btn_web);
        if (s_btn_about)     lv_group_add_obj(s_group, s_btn_about);

        if (s_dropdown_task) {
            lv_group_focus_obj(s_dropdown_task);
        }
    }
}

/* 底部功能按钮：任务控制/页面入口 */
static void action_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);

    if (code == LV_EVENT_FOCUSED) {
        lv_obj_scroll_to_view(target, LV_ANIM_ON);
        return;
    }
    if (code != LV_EVENT_CLICKED) {
        return;
    }

    const char *name = "Btn";
    if (target == s_btn_open) {
        name = "Open";
        s_open_clicked = true;
    } else if (target == s_btn_close) {
        name = "Close";
        s_close_clicked = true;
    } else if (target == s_btn_web) {
        name = "Web";
        s_web_clicked = true;
    } else if (target == s_btn_about) {
        name = "About";
        TaskScheduling_RequestPauseForSettings();
        ensure_about_screen_created();
        refresh_about_info();
        lv_scr_load(s_about_screen);
        s_is_main_screen_active = false;
        if (s_group != NULL) {
            lv_group_remove_all_objs(s_group);
            if (s_about_btn_back) lv_group_add_obj(s_group, s_about_btn_back);
            if (s_about_btn_back) lv_group_focus_obj(s_about_btn_back);
        }
        ESP_LOGI(TAG_UI, "Enter about page");
        return;
    }

    ESP_LOGI(TAG_UI, "Button clicked: %s", name);
}

/**********************
 *  UI 构建
 **********************/

static lv_obj_t *create_action_button(lv_obj_t *parent, const char *text)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 28);
    lv_obj_set_width(btn, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_pad_hor(btn, 6, 0);
    lv_obj_set_style_pad_ver(btn, 6, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, action_btn_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(btn);

    return btn;
}

static void ui_create_main_screen(void)
{
    if (s_main_screen_ready) {
        return;
    }
    s_main_screen = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(s_main_screen, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_main_screen, LV_OPA_COVER, 0);

    /* 计算屏幕尺寸以及底部摄像头画面的起始行，方便把上方 UI 与画面区域分离开 */
    const lv_coord_t disp_w = lv_disp_get_hor_res(NULL);
    const lv_coord_t disp_h = lv_disp_get_ver_res(NULL);
    const lv_coord_t cam_req_h = 230;
    lv_coord_t cam_h = (cam_req_h > disp_h) ? disp_h : cam_req_h;
    lv_coord_t cam_top = disp_h - cam_h;

    /* 顶部区域：圆角矩形背景 + 下拉框 + 设置按钮 */
    lv_obj_t *header = lv_obj_create(s_main_screen);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, LV_PCT(100), 40);
    lv_obj_set_style_bg_color(header, lv_color_hex(0xF0F0F0), 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(header, 16, 0);
    lv_obj_set_style_pad_hor(header, 8, 0);
    lv_obj_set_style_pad_ver(header, 6, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header,
                          LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 4);

    s_dropdown_task = lv_dropdown_create(header);
    lv_dropdown_set_dir(s_dropdown_task, LV_DIR_BOTTOM);
    lv_dropdown_set_options_static(
        s_dropdown_task,
        "空任务\n"
        "RGB取色\n"
        "颜色识别\n"
        "AprilTag识别\n"
        "智能巡线\n"
        "AI模型检测");
    lv_obj_set_style_text_font(s_dropdown_task, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_t *task_list = lv_dropdown_get_list(s_dropdown_task);
    if (task_list) {
        lv_obj_set_style_text_font(task_list, &lv_font_source_han_sans_sc_14_system, 0);
    }
    lv_obj_set_width(s_dropdown_task, 120);
    lv_dropdown_set_selected(s_dropdown_task, 0);
    s_sel_task = lv_dropdown_get_selected(s_dropdown_task);
    lv_obj_add_event_cb(s_dropdown_task, dropdown_event_cb, LV_EVENT_ALL, NULL);
    apply_focus_border_style(s_dropdown_task);

    /* 右侧控制按钮：齿轮图标 */
    s_btn_control = lv_btn_create(header);
    lv_obj_set_size(s_btn_control, 30, 30);
    lv_obj_set_style_radius(s_btn_control, 15, 0);
    lv_obj_set_style_bg_color(s_btn_control, lv_color_hex(0xF0F0F0), 0);
    lv_obj_set_style_bg_opa(s_btn_control, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_btn_control, 0, 0);
    lv_obj_add_event_cb(s_btn_control, control_btn_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *icon = lv_label_create(s_btn_control);
    lv_label_set_text(icon, LV_SYMBOL_SETTINGS);
    lv_obj_center(icon);
    apply_focus_border_style(s_btn_control);

    /* 下方按钮排成一行：Open / Close / Web / About（可横向滑动） */
    lv_obj_t *btn_row = lv_obj_create(s_main_screen);
    lv_obj_remove_style_all(btn_row);
    lv_obj_set_size(btn_row, LV_PCT(100), LV_SIZE_CONTENT);
    const lv_coord_t btn_row_pad_h = 6;
    const lv_coord_t btn_row_gap   = 6;
    lv_obj_set_style_pad_hor(btn_row, btn_row_pad_h, 0);
    lv_obj_set_style_pad_ver(btn_row, 6, 0);
    lv_obj_set_style_pad_column(btn_row, btn_row_gap, 0);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(btn_row, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(btn_row, LV_SCROLL_SNAP_START);
    lv_obj_set_scrollbar_mode(btn_row, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(btn_row, LV_OBJ_FLAG_SCROLL_ONE);

    s_btn_open  = create_action_button(btn_row, "开启");
    s_btn_close = create_action_button(btn_row, "关闭");
    s_btn_web   = create_action_button(btn_row, "图传");
    s_btn_about = create_action_button(btn_row, "关于");

    lv_coord_t btn_w = disp_w - btn_row_pad_h * 2 - btn_row_gap * 2;
    btn_w = (btn_w > 0) ? (btn_w / 3) : 0;
    if (btn_w > 0) {
        lv_obj_set_width(s_btn_open, btn_w);
        lv_obj_set_width(s_btn_close, btn_w);
        lv_obj_set_width(s_btn_web, btn_w);
        lv_obj_set_width(s_btn_about, btn_w);
    }
    lv_obj_update_layout(btn_row);
    lv_obj_scroll_to_x(btn_row, 0, LV_ANIM_OFF);

    /* 将按钮行固定在摄像头画面之上，避免被实时画面覆盖，焦点移动才看得见 */
    const lv_coord_t btn_row_h     = lv_obj_get_height(btn_row);
    // const lv_coord_t header_bottom = lv_obj_get_y(header) + lv_obj_get_height(header);
    lv_coord_t row_y = cam_top - btn_row_h - 6;           
    // if (row_y < header_bottom + 2) {
    //     row_y = header_bottom + 2;                        
    // }
    lv_obj_set_pos(btn_row, 0, row_y);

    lv_obj_t *divider = lv_obj_create(s_main_screen);
    lv_obj_remove_style_all(divider);
    lv_coord_t divider_w = (disp_w > 12) ? disp_w - 12 : disp_w;
    lv_obj_set_size(divider, divider_w, 2);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0xD0D5DE), 0);
    lv_obj_set_style_bg_opa(divider, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(divider, 1, 0);
    lv_coord_t divider_x = (disp_w - divider_w) / 2;
    lv_coord_t divider_y = cam_top - 4;
    lv_obj_set_pos(divider, divider_x, divider_y);

    /* 右下角 FPS 标签 */
    s_fps_label = lv_label_create(lv_layer_top()); /* 顶层确保不被覆盖 */
    lv_obj_remove_style_all(s_fps_label);          /* 去掉默认白色背景 */
    lv_obj_set_style_bg_opa(s_fps_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_fps_label, 0, 0);
    lv_obj_set_style_pad_all(s_fps_label, 0, 0);
    lv_obj_set_style_text_color(s_fps_label, lv_color_hex(0x2A7FFF), 0);
    lv_label_set_text(s_fps_label, "FPS --");
    lv_obj_align(s_fps_label, LV_ALIGN_BOTTOM_RIGHT, -6, -6);
    lv_obj_add_flag(s_fps_label, LV_OBJ_FLAG_HIDDEN); /* 默认隐藏，无任务时不再显示 UI FPS */
    lv_obj_move_foreground(s_fps_label);

    s_main_screen_ready = true;
}

static void ui_load_main_screen(void)
{
    ui_create_main_screen();
    if (s_main_screen != NULL) {
        lv_scr_load(s_main_screen);
        s_is_main_screen_active = true;
    }

    if (s_group != NULL) {
        lv_group_remove_all_objs(s_group);
        if (s_dropdown_task) lv_group_add_obj(s_group, s_dropdown_task);
        if (s_btn_control)   lv_group_add_obj(s_group, s_btn_control);
        if (s_btn_open)      lv_group_add_obj(s_group, s_btn_open);
        if (s_btn_close)     lv_group_add_obj(s_group, s_btn_close);
        if (s_btn_web)       lv_group_add_obj(s_group, s_btn_web);
        if (s_btn_about)     lv_group_add_obj(s_group, s_btn_about);

        if (s_dropdown_task) {
            lv_group_focus_obj(s_dropdown_task);
        }
    }
    apply_runtime_settings();

    if (s_fps_timer == NULL) {
        s_fps_timer = lv_timer_create(fps_timer_cb, 400, NULL);
    }
    s_fps_hidden_for_lcd = false;

    fw_update_notify_main_screen_entered();
}

/**********************
 *  对外接口
 **********************/

void Lvgl_Example1(void)
{
    /* 启动时先进入 Wi-Fi 配置页 */
    ui_create_wifi_screen();

    s_group = lv_group_create();
    lv_group_set_default(s_group);
    wifi_refresh_group();

    /* 注册按键输入设备 */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = keypad_read_cb;

    s_keypad = lv_indev_drv_register(&indev_drv);
    lv_indev_set_group(s_keypad, s_group);

    /* 跨线程 UI 消息分发：由 LVGL 线程（lv_timer_handler）定期执行 */
    ui_async_ensure();
    if (!s_ui_async_timer) {
        s_ui_async_timer = lv_timer_create(ui_async_timer_cb, 10, NULL);
    }

    /* 默认先启动 AP 模式，保证上电后在 Wi-Fi 配置页即可被电脑发现并连接 KPUAV 热点；
     * 若用户选择 STA 连接，会自动切换到 STA 模式并停止 AP。 */
    if (wifi_get_run_mode() != WIFI_RUN_MODE_AP) {
        /* 将 AP 启动从 UI 初始化路径中挪出：
         * - 避免与 LCD/背光/SD 初始化并发导致瞬态电流峰值
         * - 避免在 LVGL 初始化阶段长时间阻塞
         */
        lvgl_ui_wifi_set_status("Starting AP...");
        if (s_wifi_ap_autostart_task == NULL) {
            (void)xTaskCreate(wifi_ap_autostart_task, "wifi_ap_auto", 4096, NULL, 3, &s_wifi_ap_autostart_task);
        }
    }

    if (s_fps_timer == NULL) {
        s_fps_timer = lv_timer_create(fps_timer_cb, 400, NULL);
    }

    ESP_LOGI(TAG_UI, "LVGL WiFi setup ready");
}

static void wifi_ap_autostart_task(void *arg)
{
    (void)arg;

    /* 给 LCD/LVGL 一个稳定显示窗口，降低“重启就花屏”的观感与棕断风险 */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) > 0) {
        goto out;
    }
    /* 若用户在延时窗口后触发取消，也不要再启动 AP */
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
        goto out;
    }

    if (wifi_get_run_mode() != WIFI_RUN_MODE_AP) {
        bool ok = wifi_start_ap_mode();
        lvgl_ui_wifi_set_status(ok ? "AP mode ready" : "AP start failed");
    }

out:
    s_wifi_ap_autostart_task = NULL;
    vTaskDelete(NULL);
}

bool lvgl_ui_async_call(void (*cb)(void *), void *user_data)
{
    if (!cb) return false;
    ui_async_ensure();
    if (!s_ui_async_q) return false;

    lvgl_ui_async_msg_t msg = {
        .cb = (lvgl_ui_async_cb_t)cb,
        .user_data = user_data,
    };
    return xQueueSend(s_ui_async_q, &msg, 0) == pdTRUE;
}

bool lvgl_ui_is_main_screen_active(void)
{
    return s_is_main_screen_active;
}

static uint16_t task_to_index(lvgl_ui_task_t task)
{
    switch (task) {
    case LVGL_UI_TASK_FIND_COLOR:   return 1;
    case LVGL_UI_TASK_COLOR_DETECT: return 2;
    case LVGL_UI_TASK_APRILTAG:     return 3;
    case LVGL_UI_TASK_LINE_DETECT:  return 4;
    case LVGL_UI_TASK_AI_DETECT:    return 5;
    case LVGL_UI_TASK_NONE:
    default:                        return 0;
    }
}

static uint16_t tag_dict_to_index(lvgl_ui_tag_dict_t dict)
{
    switch (dict) {
    case LVGL_UI_TAG_16H5:  return 1;
    case LVGL_UI_TAG_36H11: return 2;
    case LVGL_UI_TAG_NONE:
    default:                return 0;
    }
}

static uint16_t led_mode_to_index(lvgl_ui_led_mode_t mode)
{
    switch (mode) {
    case LVGL_UI_LED_ON:  return 1;
    case LVGL_UI_LED_OFF: return 2;
    case LVGL_UI_LED_NONE:
    default:              return 0;
    }
}

static uint16_t frame_mode_to_index(lvgl_ui_frame_mode_t mode)
{
    switch (mode) {
    case LVGL_UI_FRAME_ON:  return 1;
    case LVGL_UI_FRAME_OFF: return 2;
    case LVGL_UI_FRAME_NONE:
    default:                return 0;
    }
}

typedef struct {
    lv_obj_t *dd;
    uint16_t sel;
} dropdown_set_req_t;

static void dropdown_set_req_cb(void *user_data)
{
    dropdown_set_req_t *req = (dropdown_set_req_t *)user_data;
    if (!req) return;
    if (req->dd) {
        uint16_t cnt = lv_dropdown_get_option_cnt(req->dd);
        uint16_t sel = req->sel;
        if (cnt > 0 && sel >= cnt) {
            sel = cnt - 1;
        }
        s_programmatic_dropdown = true;
        lv_dropdown_set_selected(req->dd, sel);
        s_programmatic_dropdown = false;
    }
    free(req);
}

void lvgl_ui_set_selected_task(lvgl_ui_task_t task)
{
    uint16_t idx = task_to_index(task);
    s_sel_task = idx;

    if (s_dropdown_task) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dropdown_task;
            req->sel = idx;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_color_profile(uint8_t color_id)
{
    if (color_id > 7) color_id = 0;
    s_sel_color_profile = (uint16_t)color_id;

    if (s_dd_color_profile) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_color_profile;
            req->sel = s_sel_color_profile;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_tag_dict(lvgl_ui_tag_dict_t tag_dict)
{
    s_sel_tag_dict = tag_dict_to_index(tag_dict);

    if (s_dd_tag_dict) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_tag_dict;
            req->sel = s_sel_tag_dict;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_ai_model(uint8_t model_index)
{
    s_sel_ai_model = (uint16_t)model_index;

    if (s_dd_ai_model) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_ai_model;
            req->sel = s_sel_ai_model;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_ai_score(uint8_t score_index)
{
    if (score_index > 7) score_index = 0;
    s_sel_ai_score = (uint16_t)score_index;

    if (s_dd_ai_score) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_ai_score;
            req->sel = s_sel_ai_score;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_led_mode(lvgl_ui_led_mode_t mode)
{
    s_sel_fill_light = led_mode_to_index(mode);
    apply_led_setting();

    if (s_dd_fill_light) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_fill_light;
            req->sel = s_sel_fill_light;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_frame_mode(lvgl_ui_frame_mode_t mode)
{
    s_sel_frame_overlay = frame_mode_to_index(mode);
    apply_frame_overlay_setting();

    if (s_dd_frame_overlay) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_frame_overlay;
            req->sel = s_sel_frame_overlay;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_msroi_enabled(bool enable)
{
    s_sel_msroi = enable ? 1 : 2;
    apply_msroi_setting();

    if (s_dd_msroi) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_msroi;
            req->sel = s_sel_msroi;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

void lvgl_ui_set_selected_msroi_modules(bool module1_enable, bool module2_enable, bool module3_enable)
{
    s_sel_m1_coarse_mode = module1_enable ? 1 : 2;
    s_sel_m2_fallback = module2_enable ? 1 : 2;
    s_sel_m3_roi_refine = module3_enable ? 1 : 2;
    apply_msroi_module_settings();

    if (s_dd_m1_coarse_mode) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_m1_coarse_mode;
            req->sel = s_sel_m1_coarse_mode;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
    if (s_dd_m2_fallback) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_m2_fallback;
            req->sel = s_sel_m2_fallback;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
    if (s_dd_m3_roi_refine) {
        dropdown_set_req_t *req = (dropdown_set_req_t *)malloc(sizeof(*req));
        if (req) {
            req->dd = s_dd_m3_roi_refine;
            req->sel = s_sel_m3_roi_refine;
            if (!lvgl_ui_async_call(dropdown_set_req_cb, req)) {
                free(req);
            }
        }
    }
}

lvgl_ui_task_t lvgl_ui_get_selected_task(void)
{
    switch (s_sel_task) {
    case 0: return LVGL_UI_TASK_NONE;
    case 1: return LVGL_UI_TASK_FIND_COLOR;
    case 2: return LVGL_UI_TASK_COLOR_DETECT;
    case 3: return LVGL_UI_TASK_APRILTAG;
    case 4: return LVGL_UI_TASK_LINE_DETECT;
    case 5: return LVGL_UI_TASK_AI_DETECT;
    default: return LVGL_UI_TASK_NONE;
    }
}

uint8_t lvgl_ui_get_selected_color_profile(void)
{
    return (s_sel_color_profile == 0) ? 0 : (uint8_t)s_sel_color_profile;
}

lvgl_ui_tag_dict_t lvgl_ui_get_selected_tag_dict(void)
{
    switch (s_sel_tag_dict) {
    case 1: return LVGL_UI_TAG_16H5;
    case 2: return LVGL_UI_TAG_36H11;
    default: return LVGL_UI_TAG_NONE;
    }
}

lvgl_ui_led_mode_t lvgl_ui_get_selected_led_mode(void)
{
    switch (s_sel_fill_light) {
    case 1: return LVGL_UI_LED_ON;
    case 2: return LVGL_UI_LED_OFF;
    default: return LVGL_UI_LED_NONE;
    }
}

lvgl_ui_frame_mode_t lvgl_ui_get_selected_frame_mode(void)
{
    switch (s_sel_frame_overlay) {
    case 1: return LVGL_UI_FRAME_ON;
    case 2: return LVGL_UI_FRAME_OFF;
    default: return LVGL_UI_FRAME_NONE;
    }
}

lvgl_ui_stream_mode_t lvgl_ui_get_selected_stream_mode(void)
{
    switch (s_sel_stream_mode) {
    case 1: return LVGL_UI_STREAM_SINGLE;
    case 2: return LVGL_UI_STREAM_STREAM;
    default: return LVGL_UI_STREAM_NONE;
    }
}

bool lvgl_ui_get_selected_model(char *buf, size_t len)
{
    if (!buf || len == 0) return false;
    buf[0] = '\0';
    if (s_sel_ai_model == 0) return false;
    if (s_dd_ai_model) {
        lv_dropdown_get_selected_str(s_dd_ai_model, buf, len);
        if (buf[0] != '\0') return true;
    }

    char *opts = sd_list_models_alloc();
    if (!opts) return false;
    uint16_t idx = 0;
    const char *line = opts;
    const char *p = opts;
    bool found = false;
    while (*p) {
        if (*p == '\n') {
            if (idx == s_sel_ai_model) {
                size_t n = (size_t)(p - line);
                if (n >= len) n = len - 1;
                memcpy(buf, line, n);
                buf[n] = '\0';
                found = (buf[0] != '\0');
                break;
            }
            idx++;
            line = p + 1;
        }
        p++;
    }
    if (!found && idx == s_sel_ai_model) {
        size_t n = strlen(line);
        if (n >= len) n = len - 1;
        memcpy(buf, line, n);
        buf[n] = '\0';
        found = (buf[0] != '\0');
    }
    free(opts);
    return found;
}

bool lvgl_ui_get_selected_ai_score(float *out_score)
{
    if (!out_score) return false;
    switch (s_sel_ai_score) {
    case 1: *out_score = 0.25f; return true;
    case 2: *out_score = 0.30f; return true;
    case 3: *out_score = 0.40f; return true;
    case 4: *out_score = 0.50f; return true;
    case 5: *out_score = 0.60f; return true;
    case 6: *out_score = 0.70f; return true;
    case 7: *out_score = 0.80f; return true;
    default: return false;
    }
}

typedef struct {
    char request_id[40];
    char filename[260];
    char type[16];
    char sha256[80];
    uint32_t size_bytes;
    bool has_sha256;
} ui_upload_prompt_args_t;

typedef struct {
    char request_id[40];
} ui_upload_reqid_args_t;

typedef struct {
    char request_id[40];
    uint32_t received;
    uint32_t total;
} ui_upload_progress_args_t;

typedef struct {
    char request_id[40];
    char message[160];
    bool success;
} ui_upload_finish_args_t;

static bool upload_event_is_activate(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) return true;
    if (code == LV_EVENT_KEY && lv_event_get_key(e) == LV_KEY_ENTER) return true;
    return false;
}

static void upload_keypad_wait_release(void)
{
    if (s_keypad) {
        lv_indev_wait_release(s_keypad);
    }
}

static bool upload_btn_is_addable(lv_obj_t *obj)
{
    if (!obj) return false;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN)) return false;
    return true;
}

static bool upload_btn_is_focusable(lv_obj_t *obj)
{
    if (!upload_btn_is_addable(obj)) return false;
    if (lv_obj_has_state(obj, LV_STATE_DISABLED)) return false;
    return true;
}

static void upload_modal_begin(void)
{
    if (!s_keypad) return;
    if (!s_group) return;

    if (!s_upload_prev_focus && s_group) {
        s_upload_prev_focus = lv_group_get_focused(s_group);
    }

    if (!s_upload_group) {
        s_upload_group = lv_group_create();
        if (!s_upload_group) return;
    }
    lv_group_remove_all_objs(s_upload_group);
    lv_indev_set_group(s_keypad, s_upload_group);
}

static void upload_modal_refresh_group(void)
{
    if (!s_upload_group) return;
    lv_group_remove_all_objs(s_upload_group);

    lv_obj_t *focus = NULL;
    lv_obj_t *first_added = NULL;

    if (upload_btn_is_addable(s_upload_btn_reject)) {
        lv_group_add_obj(s_upload_group, s_upload_btn_reject);
        if (!first_added) first_added = s_upload_btn_reject;
        if (!focus && upload_btn_is_focusable(s_upload_btn_reject)) focus = s_upload_btn_reject;
    }
    if (upload_btn_is_addable(s_upload_btn_accept)) {
        lv_group_add_obj(s_upload_group, s_upload_btn_accept);
        if (!first_added) first_added = s_upload_btn_accept;
        if (!focus && upload_btn_is_focusable(s_upload_btn_accept)) focus = s_upload_btn_accept;
    }
    if (upload_btn_is_addable(s_upload_btn_cancel)) {
        lv_group_add_obj(s_upload_group, s_upload_btn_cancel);
        if (!first_added) first_added = s_upload_btn_cancel;
        if (!focus && upload_btn_is_focusable(s_upload_btn_cancel)) focus = s_upload_btn_cancel;
    }

    if (!focus) {
        focus = first_added;
    }

    if (focus) {
        lv_group_focus_obj(focus);
    }
}

static void upload_modal_end(void)
{
    if (!s_upload_group) return;

    if (s_keypad && s_group && s_keypad->group == s_upload_group) {
        lv_indev_set_group(s_keypad, s_group);
        if (s_upload_prev_focus &&
            lv_obj_is_valid(s_upload_prev_focus) &&
            lv_obj_get_group(s_upload_prev_focus) == s_group) {
            lv_group_focus_obj(s_upload_prev_focus);
        }
    }

    s_upload_prev_focus = NULL;

    lv_group_del(s_upload_group);
    s_upload_group = NULL;
}

static void upload_ui_destroy(void)
{
    upload_keypad_wait_release();
    upload_modal_end();
    if (s_upload_timeout_timer) {
        lv_timer_del(s_upload_timeout_timer);
        s_upload_timeout_timer = NULL;
    }
    if (s_upload_close_timer) {
        lv_timer_del(s_upload_close_timer);
        s_upload_close_timer = NULL;
    }
    if (s_upload_overlay) {
        lv_obj_del_async(s_upload_overlay);
        s_upload_overlay = NULL;
    }
    s_upload_panel = NULL;
    s_upload_title_label = NULL;
    s_upload_meta_label = NULL;
    s_upload_progress_label = NULL;
    s_upload_progress_bar = NULL;
    s_upload_btn_accept = NULL;
    s_upload_btn_reject = NULL;
    s_upload_btn_cancel = NULL;
    s_upload_request_id[0] = '\0';
    s_upload_total_bytes = 0;
    s_upload_type[0] = '\0';
}

static void upload_close_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    upload_ui_destroy();
}

static void upload_timeout_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (s_upload_request_id[0] != '\0') {
        wifi_upload_user_decide(s_upload_request_id, WIFI_UPLOAD_DECISION_TIMEOUT);
    }
    upload_ui_destroy();
    ui_show_hint_ms("Request timeout", 3000);
}

static lv_obj_t *upload_create_btn(lv_obj_t *parent, const char *text)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 36);
    lv_obj_set_flex_grow(btn, 1);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_pad_hor(btn, 10, 0);
    lv_obj_set_style_pad_ver(btn, 6, 0);
    apply_focus_border_style(btn);

    lv_obj_t *lab = lv_label_create(btn);
    lv_label_set_text(lab, text ? text : "");
    lv_obj_set_style_text_font(lab, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(lab, lv_color_hex(0x111827), 0);
    lv_obj_center(lab);
    return btn;
}

static void upload_btn_accept_event_cb(lv_event_t *e)
{
    if (!upload_event_is_activate(e)) return;
    if (s_upload_request_id[0] == '\0') return;

    upload_keypad_wait_release();

    if (s_upload_timeout_timer) {
        lv_timer_del(s_upload_timeout_timer);
        s_upload_timeout_timer = NULL;
    }

    if (s_upload_btn_accept) lv_obj_add_state(s_upload_btn_accept, LV_STATE_DISABLED);
    if (s_upload_btn_reject) lv_obj_add_state(s_upload_btn_reject, LV_STATE_DISABLED);
    upload_modal_refresh_group();

    if (s_upload_progress_label) {
        const bool is_app = (strcmp(s_upload_type, "app") == 0);
        lv_label_set_text(s_upload_progress_label, is_app ? "检查分区..." : "检测SD...");
        lv_obj_clear_flag(s_upload_progress_label, LV_OBJ_FLAG_HIDDEN);
    }

    wifi_upload_user_decide(s_upload_request_id, WIFI_UPLOAD_DECISION_ACCEPT);
}

static void upload_btn_reject_event_cb(lv_event_t *e)
{
    if (!upload_event_is_activate(e)) return;
    upload_keypad_wait_release();
    if (s_upload_request_id[0] != '\0') {
        wifi_upload_user_decide(s_upload_request_id, WIFI_UPLOAD_DECISION_REJECT);
    }
    upload_ui_destroy();
}

static void upload_btn_cancel_event_cb(lv_event_t *e)
{
    if (!upload_event_is_activate(e)) return;
    if (s_upload_request_id[0] == '\0') return;
    upload_keypad_wait_release();
    if (s_upload_btn_cancel) lv_obj_add_state(s_upload_btn_cancel, LV_STATE_DISABLED);
    upload_modal_refresh_group();
    if (s_upload_progress_label) {
        lv_label_set_text(s_upload_progress_label, "返回...");
    }
    wifi_upload_user_decide(s_upload_request_id, WIFI_UPLOAD_DECISION_CANCEL);
}

static void upload_prompt_async(void *user_data)
{
    ui_upload_prompt_args_t *args = (ui_upload_prompt_args_t *)user_data;
    if (!args) return;

    upload_ui_destroy();
    snprintf(s_upload_request_id, sizeof(s_upload_request_id), "%s", args->request_id);
    s_upload_total_bytes = args->size_bytes;
    snprintf(s_upload_type, sizeof(s_upload_type), "%s", args->type[0] ? args->type : "");
    const bool is_app = (strcmp(s_upload_type, "app") == 0);

    lv_obj_t *scr = lv_layer_top();
    s_upload_overlay = lv_obj_create(scr);
    lv_obj_set_size(s_upload_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(s_upload_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_upload_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_upload_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_upload_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_upload_overlay, LV_OPA_50, 0);
    lv_obj_set_style_border_width(s_upload_overlay, 0, 0);

    s_upload_panel = lv_obj_create(s_upload_overlay);
    lv_obj_set_width(s_upload_panel, LV_PCT(92));
    lv_obj_set_height(s_upload_panel, LV_SIZE_CONTENT);
    lv_obj_align(s_upload_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_upload_panel, 12, 0);
    lv_obj_set_style_pad_gap(s_upload_panel, 10, 0);
    lv_obj_set_style_radius(s_upload_panel, 16, 0);
    lv_obj_set_style_bg_color(s_upload_panel, lv_color_hex(0xF0F0F0), 0);
    lv_obj_set_style_bg_opa(s_upload_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_upload_panel, lv_color_hex(0xC8CDD5), 0);
    lv_obj_set_style_border_width(s_upload_panel, 2, 0);
    lv_obj_set_style_shadow_color(s_upload_panel, lv_color_hex(0x9FA6B5), 0);
    lv_obj_set_style_shadow_opa(s_upload_panel, LV_OPA_30, 0);
    lv_obj_set_style_shadow_width(s_upload_panel, 10, 0);
    lv_obj_set_style_shadow_ofs_x(s_upload_panel, 0, 0);
    lv_obj_set_style_shadow_ofs_y(s_upload_panel, 3, 0);
    lv_obj_set_style_shadow_spread(s_upload_panel, 0, 0);
    lv_obj_set_flex_flow(s_upload_panel, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *title_chip = lv_obj_create(s_upload_panel);
    lv_obj_remove_style_all(title_chip);
    lv_obj_set_width(title_chip, LV_PCT(100));
    lv_obj_set_height(title_chip, LV_SIZE_CONTENT);
    lv_obj_clear_flag(title_chip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(title_chip, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(title_chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(title_chip, 12, 0);
    lv_obj_set_style_pad_hor(title_chip, 12, 0);
    lv_obj_set_style_pad_ver(title_chip, 8, 0);
    lv_obj_set_style_border_width(title_chip, 0, 0);

    s_upload_title_label = lv_label_create(title_chip);
    lv_obj_set_width(s_upload_title_label, LV_PCT(100));
    lv_label_set_text(s_upload_title_label, is_app ? "固件升级" : "模型接收");
    lv_obj_set_style_text_font(s_upload_title_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_upload_title_label, lv_color_hex(0x111827), 0);
    lv_obj_set_style_text_align(s_upload_title_label, LV_TEXT_ALIGN_CENTER, 0);

    char meta[768];
    if (args->has_sha256 && args->sha256[0] != '\0') {
        snprintf(meta, sizeof(meta),
                 "File: %s\nSize: %lu bytes\nType: %s\nSHA256: %s",
                 args->filename,
                 (unsigned long)args->size_bytes,
                 args->type[0] ? args->type : "espdl",
                 args->sha256);
    } else {
        snprintf(meta, sizeof(meta),
                 "File: %s\nSize: %lu bytes\nType: %s",
                 args->filename,
                 (unsigned long)args->size_bytes,
                 args->type[0] ? args->type : "espdl");
    }

    s_upload_meta_label = lv_label_create(s_upload_panel);
    lv_obj_set_width(s_upload_meta_label, LV_PCT(100));
    lv_label_set_long_mode(s_upload_meta_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_upload_meta_label, meta);
    lv_obj_set_style_text_font(s_upload_meta_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_upload_meta_label, lv_color_hex(0x111827), 0);

    s_upload_progress_label = lv_label_create(s_upload_panel);
    lv_obj_set_width(s_upload_progress_label, LV_PCT(100));
    lv_label_set_long_mode(s_upload_progress_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_upload_progress_label, "");
    lv_obj_set_style_text_font(s_upload_progress_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_upload_progress_label, lv_color_hex(0x111827), 0);
    lv_obj_add_flag(s_upload_progress_label, LV_OBJ_FLAG_HIDDEN);

    s_upload_progress_bar = lv_bar_create(s_upload_panel);
    lv_obj_set_width(s_upload_progress_bar, LV_PCT(100));
    lv_bar_set_range(s_upload_progress_bar, 0, 100);
    lv_bar_set_value(s_upload_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_add_flag(s_upload_progress_bar, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *row = lv_obj_create(s_upload_panel);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_pad_gap(row, 10, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);

    s_upload_btn_reject = upload_create_btn(row, "拒绝");
    lv_obj_add_event_cb(s_upload_btn_reject, upload_btn_reject_event_cb, LV_EVENT_ALL, NULL);

    s_upload_btn_accept = upload_create_btn(row, is_app ? "升级" : "接收");
    lv_obj_add_event_cb(s_upload_btn_accept, upload_btn_accept_event_cb, LV_EVENT_ALL, NULL);

    s_upload_btn_cancel = upload_create_btn(s_upload_panel, "返回");
    lv_obj_add_event_cb(s_upload_btn_cancel, upload_btn_cancel_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(s_upload_btn_cancel, LV_OBJ_FLAG_HIDDEN);

    s_upload_timeout_timer = lv_timer_create(upload_timeout_timer_cb, 30000, NULL);
    lv_timer_set_repeat_count(s_upload_timeout_timer, 1);

    upload_modal_begin();
    upload_modal_refresh_group();

    free(args);
}

static void upload_set_waiting_async(void *user_data)
{
    ui_upload_reqid_args_t *args = (ui_upload_reqid_args_t *)user_data;
    if (!args) return;

    if (s_upload_overlay && s_upload_request_id[0] != '\0' &&
        strcmp(args->request_id, s_upload_request_id) == 0) {

        if (s_upload_btn_accept) lv_obj_add_flag(s_upload_btn_accept, LV_OBJ_FLAG_HIDDEN);
        if (s_upload_btn_reject) lv_obj_add_flag(s_upload_btn_reject, LV_OBJ_FLAG_HIDDEN);
        if (s_upload_btn_cancel) {
            lv_obj_clear_flag(s_upload_btn_cancel, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_state(s_upload_btn_cancel, LV_STATE_DISABLED);
        }
        upload_modal_refresh_group();

        if (s_upload_progress_label) {
            lv_label_set_text(s_upload_progress_label, "Waiting upload...");
            lv_obj_clear_flag(s_upload_progress_label, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_upload_progress_bar) {
            lv_bar_set_value(s_upload_progress_bar, 0, LV_ANIM_OFF);
            lv_obj_clear_flag(s_upload_progress_bar, LV_OBJ_FLAG_HIDDEN);
        }
    }

    free(args);
}

static void upload_progress_async(void *user_data)
{
    ui_upload_progress_args_t *args = (ui_upload_progress_args_t *)user_data;
    if (!args) return;

    if (s_upload_overlay && s_upload_request_id[0] != '\0' &&
        strcmp(args->request_id, s_upload_request_id) == 0) {

        uint32_t total = args->total ? args->total : s_upload_total_bytes;
        uint32_t percent = 0;
        if (total > 0) {
            percent = (uint32_t)(((uint64_t)args->received * 100ULL) / total);
            if (percent > 100) percent = 100;
        }

        if (s_upload_progress_bar) {
            lv_bar_set_value(s_upload_progress_bar, (int32_t)percent, LV_ANIM_OFF);
            lv_obj_clear_flag(s_upload_progress_bar, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_upload_progress_label) {
            char buf[96];
            snprintf(buf, sizeof(buf), "接收: %lu/%lu (%lu%%)",
                     (unsigned long)args->received,
                     (unsigned long)total,
                     (unsigned long)percent);
            lv_label_set_text(s_upload_progress_label, buf);
            lv_obj_clear_flag(s_upload_progress_label, LV_OBJ_FLAG_HIDDEN);
        }
    }

    free(args);
}

static void upload_finish_async(void *user_data)
{
    ui_upload_finish_args_t *args = (ui_upload_finish_args_t *)user_data;
    if (!args) return;

    if (s_upload_overlay && s_upload_request_id[0] != '\0' &&
        strcmp(args->request_id, s_upload_request_id) == 0) {

        if (s_upload_timeout_timer) {
            lv_timer_del(s_upload_timeout_timer);
            s_upload_timeout_timer = NULL;
        }

        if (s_upload_title_label) {
            lv_label_set_text(s_upload_title_label, args->success ? "OK" : "FAIL");
        }
        if (s_upload_btn_cancel) lv_obj_add_flag(s_upload_btn_cancel, LV_OBJ_FLAG_HIDDEN);
        upload_modal_refresh_group();

        if (s_upload_progress_label) {
            lv_label_set_text(s_upload_progress_label, args->message[0] ? args->message : "");
            lv_obj_clear_flag(s_upload_progress_label, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_upload_progress_bar) {
            lv_bar_set_value(s_upload_progress_bar, args->success ? 100 : lv_bar_get_value(s_upload_progress_bar), LV_ANIM_OFF);
            lv_obj_clear_flag(s_upload_progress_bar, LV_OBJ_FLAG_HIDDEN);
        }

        if (s_upload_close_timer) {
            lv_timer_del(s_upload_close_timer);
            s_upload_close_timer = NULL;
        }
        s_upload_close_timer = lv_timer_create(upload_close_timer_cb, 1500, NULL);
        lv_timer_set_repeat_count(s_upload_close_timer, 1);
    }

    free(args);
}

void lvgl_ui_upload_show_request(const char *request_id,
                                const char *filename,
                                uint32_t size_bytes,
                                const char *type,
                                const char *sha256)
{
    if (!request_id || request_id[0] == '\0') return;
    ui_upload_prompt_args_t *args = (ui_upload_prompt_args_t *)malloc(sizeof(ui_upload_prompt_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    snprintf(args->request_id, sizeof(args->request_id), "%s", request_id);
    snprintf(args->filename, sizeof(args->filename), "%s", filename ? filename : "");
    snprintf(args->type, sizeof(args->type), "%s", type ? type : "espdl");
    args->size_bytes = size_bytes;
    if (sha256 && sha256[0] != '\0') {
        snprintf(args->sha256, sizeof(args->sha256), "%s", sha256);
        args->has_sha256 = true;
    }
    if (!lvgl_ui_async_call(upload_prompt_async, args)) {
        free(args);
    }
}

void lvgl_ui_upload_set_waiting(const char *request_id)
{
    if (!request_id || request_id[0] == '\0') return;
    ui_upload_reqid_args_t *args = (ui_upload_reqid_args_t *)malloc(sizeof(ui_upload_reqid_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    snprintf(args->request_id, sizeof(args->request_id), "%s", request_id);
    if (!lvgl_ui_async_call(upload_set_waiting_async, args)) {
        free(args);
    }
}

void lvgl_ui_upload_update_progress(const char *request_id, uint32_t received, uint32_t total)
{
    if (!request_id || request_id[0] == '\0') return;
    ui_upload_progress_args_t *args = (ui_upload_progress_args_t *)malloc(sizeof(ui_upload_progress_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    snprintf(args->request_id, sizeof(args->request_id), "%s", request_id);
    args->received = received;
    args->total = total;
    if (!lvgl_ui_async_call(upload_progress_async, args)) {
        free(args);
    }
}

void lvgl_ui_upload_finish(const char *request_id, bool success, const char *message)
{
    if (!request_id || request_id[0] == '\0') return;
    ui_upload_finish_args_t *args = (ui_upload_finish_args_t *)malloc(sizeof(ui_upload_finish_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    snprintf(args->request_id, sizeof(args->request_id), "%s", request_id);
    snprintf(args->message, sizeof(args->message), "%s", message ? message : "");
    args->success = success;
    if (!lvgl_ui_async_call(upload_finish_async, args)) {
        free(args);
    }
}

typedef struct {
    char current_ver[32];
    char latest_ver[32];
} ui_fwupd_prompt_args_t;

typedef struct {
    int percent;
    char message[96];
} ui_fwupd_progress_args_t;

typedef struct {
    bool success;
    char message[96];
} ui_fwupd_finish_args_t;

static bool fwupd_btn_is_addable(lv_obj_t *obj)
{
    if (!obj) return false;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN)) return false;
    return true;
}

static bool fwupd_btn_is_focusable(lv_obj_t *obj)
{
    if (!fwupd_btn_is_addable(obj)) return false;
    if (lv_obj_has_state(obj, LV_STATE_DISABLED)) return false;
    return true;
}

static void fwupd_modal_begin(void)
{
    if (!s_keypad) return;
    if (!s_group) return;

    if (!s_fwupd_prev_focus && s_group) {
        s_fwupd_prev_focus = lv_group_get_focused(s_group);
    }

    if (!s_fwupd_group) {
        s_fwupd_group = lv_group_create();
        if (!s_fwupd_group) return;
    }
    lv_group_remove_all_objs(s_fwupd_group);
    lv_indev_set_group(s_keypad, s_fwupd_group);
}

static void fwupd_modal_refresh_group(void)
{
    if (!s_fwupd_group) return;
    lv_group_remove_all_objs(s_fwupd_group);

    lv_obj_t *focus = NULL;
    lv_obj_t *first_added = NULL;

    if (fwupd_btn_is_addable(s_fwupd_btn_reject)) {
        lv_group_add_obj(s_fwupd_group, s_fwupd_btn_reject);
        if (!first_added) first_added = s_fwupd_btn_reject;
        if (!focus && fwupd_btn_is_focusable(s_fwupd_btn_reject)) focus = s_fwupd_btn_reject;
    }
    if (fwupd_btn_is_addable(s_fwupd_btn_accept)) {
        lv_group_add_obj(s_fwupd_group, s_fwupd_btn_accept);
        if (!first_added) first_added = s_fwupd_btn_accept;
        if (!focus && fwupd_btn_is_focusable(s_fwupd_btn_accept)) focus = s_fwupd_btn_accept;
    }

    if (!focus) {
        focus = first_added;
    }

    if (focus) {
        lv_group_focus_obj(focus);
    }
}

static void fwupd_modal_end(void)
{
    if (!s_fwupd_group) return;

    if (s_keypad && s_group && s_keypad->group == s_fwupd_group) {
        lv_indev_set_group(s_keypad, s_group);
        if (s_fwupd_prev_focus &&
            lv_obj_is_valid(s_fwupd_prev_focus) &&
            lv_obj_get_group(s_fwupd_prev_focus) == s_group) {
            lv_group_focus_obj(s_fwupd_prev_focus);
        }
    }

    s_fwupd_prev_focus = NULL;
    lv_group_del(s_fwupd_group);
    s_fwupd_group = NULL;
}

static void fwupd_ui_destroy(void)
{
    upload_keypad_wait_release();
    fwupd_modal_end();
    if (s_fwupd_close_timer) {
        lv_timer_del(s_fwupd_close_timer);
        s_fwupd_close_timer = NULL;
    }
    if (s_fwupd_overlay) {
        lv_obj_del_async(s_fwupd_overlay);
        s_fwupd_overlay = NULL;
    }
    s_fwupd_panel = NULL;
    s_fwupd_title_label = NULL;
    s_fwupd_meta_label = NULL;
    s_fwupd_progress_label = NULL;
    s_fwupd_progress_bar = NULL;
    s_fwupd_btn_accept = NULL;
    s_fwupd_btn_reject = NULL;
}

static void fwupd_close_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    fwupd_ui_destroy();
}

static void fwupd_btn_reject_event_cb(lv_event_t *e)
{
    if (!upload_event_is_activate(e)) return;
    upload_keypad_wait_release();
    fw_update_user_decide(false);
    fwupd_ui_destroy();
}

static void fwupd_btn_accept_event_cb(lv_event_t *e)
{
    if (!upload_event_is_activate(e)) return;
    upload_keypad_wait_release();

    if (s_fwupd_btn_accept) lv_obj_add_state(s_fwupd_btn_accept, LV_STATE_DISABLED);
    if (s_fwupd_btn_reject) lv_obj_add_state(s_fwupd_btn_reject, LV_STATE_DISABLED);
    fwupd_modal_refresh_group();

    if (s_fwupd_progress_label) {
        lv_label_set_text(s_fwupd_progress_label, "检查分区...");
        lv_obj_clear_flag(s_fwupd_progress_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_fwupd_progress_bar) {
        lv_bar_set_value(s_fwupd_progress_bar, 0, LV_ANIM_OFF);
        lv_obj_clear_flag(s_fwupd_progress_bar, LV_OBJ_FLAG_HIDDEN);
    }

    fw_update_user_decide(true);
}

static void fwupd_prompt_async(void *user_data)
{
    ui_fwupd_prompt_args_t *args = (ui_fwupd_prompt_args_t *)user_data;
    if (!args) return;

    fwupd_ui_destroy();

    lv_obj_t *scr = lv_layer_top();
    s_fwupd_overlay = lv_obj_create(scr);
    lv_obj_set_size(s_fwupd_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(s_fwupd_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_fwupd_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_fwupd_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_fwupd_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_fwupd_overlay, LV_OPA_50, 0);
    lv_obj_set_style_border_width(s_fwupd_overlay, 0, 0);

    s_fwupd_panel = lv_obj_create(s_fwupd_overlay);
    lv_obj_set_width(s_fwupd_panel, LV_PCT(92));
    lv_obj_set_height(s_fwupd_panel, LV_SIZE_CONTENT);
    lv_obj_align(s_fwupd_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(s_fwupd_panel, 12, 0);
    lv_obj_set_style_pad_gap(s_fwupd_panel, 10, 0);
    lv_obj_set_style_radius(s_fwupd_panel, 16, 0);
    lv_obj_set_style_bg_color(s_fwupd_panel, lv_color_hex(0xF0F0F0), 0);
    lv_obj_set_style_bg_opa(s_fwupd_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_fwupd_panel, lv_color_hex(0xC8CDD5), 0);
    lv_obj_set_style_border_width(s_fwupd_panel, 2, 0);
    lv_obj_set_style_shadow_color(s_fwupd_panel, lv_color_hex(0x9FA6B5), 0);
    lv_obj_set_style_shadow_opa(s_fwupd_panel, LV_OPA_30, 0);
    lv_obj_set_style_shadow_width(s_fwupd_panel, 10, 0);
    lv_obj_set_style_shadow_ofs_x(s_fwupd_panel, 0, 0);
    lv_obj_set_style_shadow_ofs_y(s_fwupd_panel, 3, 0);
    lv_obj_set_style_shadow_spread(s_fwupd_panel, 0, 0);
    lv_obj_set_flex_flow(s_fwupd_panel, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *title_chip = lv_obj_create(s_fwupd_panel);
    lv_obj_remove_style_all(title_chip);
    lv_obj_set_width(title_chip, LV_PCT(100));
    lv_obj_set_height(title_chip, LV_SIZE_CONTENT);
    lv_obj_clear_flag(title_chip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(title_chip, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_bg_opa(title_chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(title_chip, 12, 0);
    lv_obj_set_style_pad_hor(title_chip, 12, 0);
    lv_obj_set_style_pad_ver(title_chip, 8, 0);
    lv_obj_set_style_border_width(title_chip, 0, 0);

    s_fwupd_title_label = lv_label_create(title_chip);
    lv_obj_set_width(s_fwupd_title_label, LV_PCT(100));
    lv_label_set_text(s_fwupd_title_label, "固件升级");
    lv_obj_set_style_text_font(s_fwupd_title_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_fwupd_title_label, lv_color_hex(0x111827), 0);
    lv_obj_set_style_text_align(s_fwupd_title_label, LV_TEXT_ALIGN_CENTER, 0);

    char meta[256];
    snprintf(meta, sizeof(meta),
             "Cur: %s\nNew: %s\nUpgrade?",
             args->current_ver[0] ? args->current_ver : "-",
             args->latest_ver[0] ? args->latest_ver : "-");

    s_fwupd_meta_label = lv_label_create(s_fwupd_panel);
    lv_obj_set_width(s_fwupd_meta_label, LV_PCT(100));
    lv_label_set_long_mode(s_fwupd_meta_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_fwupd_meta_label, meta);
    lv_obj_set_style_text_font(s_fwupd_meta_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_fwupd_meta_label, lv_color_hex(0x111827), 0);

    s_fwupd_progress_label = lv_label_create(s_fwupd_panel);
    lv_obj_set_width(s_fwupd_progress_label, LV_PCT(100));
    lv_label_set_long_mode(s_fwupd_progress_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_fwupd_progress_label, "");
    lv_obj_set_style_text_font(s_fwupd_progress_label, &lv_font_source_han_sans_sc_14_system, 0);
    lv_obj_set_style_text_color(s_fwupd_progress_label, lv_color_hex(0x111827), 0);
    lv_obj_add_flag(s_fwupd_progress_label, LV_OBJ_FLAG_HIDDEN);

    s_fwupd_progress_bar = lv_bar_create(s_fwupd_panel);
    lv_obj_set_width(s_fwupd_progress_bar, LV_PCT(100));
    lv_bar_set_range(s_fwupd_progress_bar, 0, 100);
    lv_bar_set_value(s_fwupd_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_add_flag(s_fwupd_progress_bar, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *row = lv_obj_create(s_fwupd_panel);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_pad_gap(row, 10, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);

    s_fwupd_btn_reject = upload_create_btn(row, "拒绝");
    lv_obj_add_event_cb(s_fwupd_btn_reject, fwupd_btn_reject_event_cb, LV_EVENT_ALL, NULL);

    s_fwupd_btn_accept = upload_create_btn(row, "升级");
    lv_obj_add_event_cb(s_fwupd_btn_accept, fwupd_btn_accept_event_cb, LV_EVENT_ALL, NULL);

    fwupd_modal_begin();
    fwupd_modal_refresh_group();

    free(args);
}

static void fwupd_progress_async(void *user_data)
{
    ui_fwupd_progress_args_t *args = (ui_fwupd_progress_args_t *)user_data;
    if (!args) return;

    if (s_fwupd_overlay) {
        if (s_fwupd_btn_accept) lv_obj_add_flag(s_fwupd_btn_accept, LV_OBJ_FLAG_HIDDEN);
        if (s_fwupd_btn_reject) lv_obj_add_flag(s_fwupd_btn_reject, LV_OBJ_FLAG_HIDDEN);
        fwupd_modal_refresh_group();

        if (s_fwupd_progress_label) {
            lv_label_set_text(s_fwupd_progress_label, args->message[0] ? args->message : "");
            lv_obj_clear_flag(s_fwupd_progress_label, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_fwupd_progress_bar) {
            int p = args->percent;
            if (p < 0) p = 0;
            if (p > 100) p = 100;
            lv_bar_set_value(s_fwupd_progress_bar, p, LV_ANIM_OFF);
            lv_obj_clear_flag(s_fwupd_progress_bar, LV_OBJ_FLAG_HIDDEN);
        }
    }

    free(args);
}

static void fwupd_finish_async(void *user_data)
{
    ui_fwupd_finish_args_t *args = (ui_fwupd_finish_args_t *)user_data;
    if (!args) return;

    if (s_fwupd_overlay) {
        if (s_fwupd_close_timer) {
            lv_timer_del(s_fwupd_close_timer);
            s_fwupd_close_timer = NULL;
        }

        if (s_fwupd_title_label) {
            lv_label_set_text(s_fwupd_title_label, args->success ? "OK" : "FAIL");
        }

        if (s_fwupd_btn_accept) lv_obj_add_flag(s_fwupd_btn_accept, LV_OBJ_FLAG_HIDDEN);
        if (s_fwupd_btn_reject) lv_obj_add_flag(s_fwupd_btn_reject, LV_OBJ_FLAG_HIDDEN);
        fwupd_modal_refresh_group();

        if (s_fwupd_progress_label) {
            lv_label_set_text(s_fwupd_progress_label, args->message[0] ? args->message : "");
            lv_obj_clear_flag(s_fwupd_progress_label, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_fwupd_progress_bar) {
            lv_obj_clear_flag(s_fwupd_progress_bar, LV_OBJ_FLAG_HIDDEN);
        }

        s_fwupd_close_timer = lv_timer_create(fwupd_close_timer_cb, 1500, NULL);
        lv_timer_set_repeat_count(s_fwupd_close_timer, 1);
    }

    free(args);
}

void lvgl_ui_fw_update_show_prompt(const char *current_ver, const char *latest_ver)
{
    ui_fwupd_prompt_args_t *args = (ui_fwupd_prompt_args_t *)malloc(sizeof(ui_fwupd_prompt_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    snprintf(args->current_ver, sizeof(args->current_ver), "%s", current_ver ? current_ver : "");
    snprintf(args->latest_ver, sizeof(args->latest_ver), "%s", latest_ver ? latest_ver : "");
    if (!lvgl_ui_async_call(fwupd_prompt_async, args)) {
        free(args);
    }
}

void lvgl_ui_fw_update_set_progress(int percent, const char *message)
{
    ui_fwupd_progress_args_t *args = (ui_fwupd_progress_args_t *)malloc(sizeof(ui_fwupd_progress_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    args->percent = percent;
    snprintf(args->message, sizeof(args->message), "%s", message ? message : "");
    if (!lvgl_ui_async_call(fwupd_progress_async, args)) {
        free(args);
    }
}

void lvgl_ui_fw_update_finish(bool success, const char *message)
{
    ui_fwupd_finish_args_t *args = (ui_fwupd_finish_args_t *)malloc(sizeof(ui_fwupd_finish_args_t));
    if (!args) return;
    memset(args, 0, sizeof(*args));
    args->success = success;
    snprintf(args->message, sizeof(args->message), "%s", message ? message : "");
    if (!lvgl_ui_async_call(fwupd_finish_async, args)) {
        free(args);
    }
}

typedef struct {
    char text[96];
    uint32_t duration_ms;
} ui_hint_args_t;

static void hint_async(void *user_data)
{
    ui_hint_args_t *args = (ui_hint_args_t *)user_data;
    if (args) {
        ui_show_hint_ms(args->text, args->duration_ms);
        free(args);
    } else {
        ui_show_hint_ms("", 0);
    }
}

void lvgl_ui_show_hint(const char *text)
{
    lvgl_ui_show_hint_ms(text, 3000);
}

void lvgl_ui_show_hint_ms(const char *text, uint32_t duration_ms)
{
    ui_hint_args_t *args = (ui_hint_args_t *)malloc(sizeof(ui_hint_args_t));
    if (!args) {
        return;
    }
    snprintf(args->text, sizeof(args->text), "%s", text ? text : "");
    args->duration_ms = duration_ms;
    if (!lvgl_ui_async_call(hint_async, args)) {
        free(args);
    }
}

static void refresh_model_list_async(void *user_data)
{
    LV_UNUSED(user_data);
    refresh_settings_dropdowns();
}

void lvgl_ui_refresh_model_list(void)
{
    (void)lvgl_ui_async_call(refresh_model_list_async, NULL);
}

bool lvgl_ui_consume_open_clicked(void)
{
    bool hit = s_open_clicked;
    s_open_clicked = false;
    return hit;
}

bool lvgl_ui_consume_close_clicked(void)
{
    bool hit = s_close_clicked;
    s_close_clicked = false;
    return hit;
}

bool lvgl_ui_consume_web_clicked(void)
{
    bool hit = s_web_clicked;
    s_web_clicked = false;
    return hit;
}
