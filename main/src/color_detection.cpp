// 轻量级 RGB 检测实现：直接在 RGB565 帧上做阈值判断，避免重度 OpenCV 运算
#include "color_detection.h"
#include "vision_msroi.h"
#include "vision_metrics.h"
#include "esp_timer.h"
#include "lcd.h"  // 用于投递叠加框到 LCD 任务

static const char *TAG = "color_detect";
static QueueHandle_t xQueueFrameIn = NULL;
static QueueHandle_t xQueueFrameOut = NULL;
static TaskHandle_t s_color_detect_task = NULL;

typedef struct {
	int x1, y1, x2, y2;
	uint8_t lost;
	bool valid;
} roi_state_t;

static roi_state_t s_roi = {0};

static inline void roi_reset(roi_state_t *r)
{
	if (!r) return;
	r->x1 = r->y1 = 0;
	r->x2 = r->y2 = 0;
	r->lost = 0;
	r->valid = false;
}

static inline void roi_apply_pad(roi_state_t *r, int pad, int w, int h)
{
	if (!r || !r->valid) return;
	r->x1 -= pad; r->y1 -= pad;
	r->x2 += pad; r->y2 += pad;
	if (r->x1 < 0) r->x1 = 0;
	if (r->y1 < 0) r->y1 = 0;
	if (r->x2 >= w) r->x2 = w - 1;
	if (r->y2 >= h) r->y2 = h - 1;
	if (r->x1 > r->x2 || r->y1 > r->y2) {
		roi_reset(r);
	}
}

// 工具：5/6bit → 8bit
static inline uint8_t up5_to_8(uint8_t v5) { return (uint8_t)((v5 * 255 + 15) / 31); }
static inline uint8_t up6_to_8(uint8_t v6) { return (uint8_t)((v6 * 255 + 31) / 63); }

// 从高字节在前的 RGB565 两字节提取 8bit R/G/B
static inline void rgb565_hi_first_to_rgb8(uint8_t hi, uint8_t lo,
										   uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
	// HI: rrrrrggg , LO: gggbbbbb
	uint16_t pix = ((uint16_t)hi << 8) | lo;
	uint8_t r5 = (pix >> 11) & 0x1F;
	uint8_t g6 = (pix >>  5) & 0x3F;
	uint8_t b5 =  pix        & 0x1F;
	*r8 = up5_to_8(r5);
	*g8 = up6_to_8(g6);
	*b8 = up5_to_8(b5);
}

// 纯 RGB 检测：在 RGB565 帧上以绝对差阈值判断像素是否接近目标颜色，返回质心与面积占比
// 增强：同时返回包围目标的轴对齐矩形框 [x1,y1,x2,y2]
static bool detect_color_rgb565_roi(const camera_fb_t *fb,
                                    uint8_t r_t, uint8_t g_t, uint8_t b_t,
                                    int tol, int step,
                                    int rx0, int ry0, int rx1, int ry1,
                                    int &cx, int &cy, float &area_pct,
                                    int &x1, int &y1, int &x2, int &y2,
                                    uint32_t *out_cnt)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) return false;
    const int w = fb->width;
    const int h = fb->height;
    const uint8_t *base = (const uint8_t *)fb->buf; // RGB565 buffer

    if (step < 1) step = 1;
    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    if (rx1 >= w) rx1 = w - 1;
    if (ry1 >= h) ry1 = h - 1;
    if (rx0 > rx1 || ry0 > ry1) {
        if (out_cnt) *out_cnt = 0;
        cx = cy = 0; area_pct = 0.f;
        return false;
    }

    uint64_t sum_x = 0, sum_y = 0;
    uint32_t cnt = 0;
    int minx = w, miny = h, maxx = -1, maxy = -1;

    for (int y = ry0; y <= ry1; y += step) {
        const uint8_t *p = base + (size_t)y * w * 2 + (size_t)rx0 * 2;
        for (int x = rx0; x <= rx1; x += step) {
            uint8_t r8, g8, b8;
            rgb565_hi_first_to_rgb8(p[0], p[1], &r8, &g8, &b8);
            int dr = (int)r8 - (int)r_t; if (dr < 0) dr = -dr;
            int dg = (int)g8 - (int)g_t; if (dg < 0) dg = -dg;
            int db = (int)b8 - (int)b_t; if (db < 0) db = -db;
            if (dr <= tol && dg <= tol && db <= tol) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                cnt++;
                if (x < minx) minx = x; 
                if (x > maxx) maxx = x;
                if (y < miny) miny = y; 
                if (y > maxy) maxy = y;
            }
            p += 2 * step;
        }
    }
    if (out_cnt) *out_cnt = cnt;
    if (cnt == 0) { cx = cy = 0; area_pct = 0.f; return false; }
    cx = (int)(sum_x / cnt);
    cy = (int)(sum_y / cnt);
    area_pct = (float)cnt * (float)(step * step) / (float)(w * h);
    x1 = minx; y1 = miny; x2 = maxx; y2 = maxy;
    return true;
}

static void color_detection_task(void *arg)
{
    uint8_t color_id = (uint32_t)arg & 0xFF;

    uint8_t r=255,g=0,b=0; // default red

    if (!nvs_read_rgb(color_id, &r, &g, &b)) {
        ESP_LOGW(TAG, "NVS no RGB for id=%u, use default R=%u G=%u B=%u", color_id, r, g, b);
    } else {
        ESP_LOGI(TAG, "Use NVS RGB id=%u -> R=%u G=%u B=%u", color_id, r, g, b);
    }

    int tol = 25;
    ESP_LOGI(TAG, "RGB target=(%u,%u,%u) tol=%d", r, g, b, tol);

    camera_fb_t *frame = NULL;

    // metrics
    uint32_t frame_cnt = 0;
    uint32_t roi_used_cnt = 0;
    uint32_t roi_ok_cnt = 0;
    uint32_t fallback_cnt = 0;
    uint64_t coarse_us_sum = 0;
    uint64_t refine_us_sum = 0;
    uint64_t busy_us_sum = 0;
    double lat_ms_sum = 0.0;
    uint32_t lat_cnt = 0;
    float roi_area_sum = 0.0f;
    uint32_t last_q_depth = 0;
    int64_t last_report_us = esp_timer_get_time();
    uint32_t seq = 0;

    roi_reset(&s_roi);

    while (1) {
        if (xQueueReceive(xQueueFrameIn, &frame, portMAX_DELAY)) {
            if (!frame) continue;
            const int src_w_local = (int)frame->width;
            const int src_h_local = (int)frame->height;
            bool ok_flag = false; int bx1=0, by1=0, bx2=0, by2=0;
            float area_val = 0.f;

            bool msroi = vision_msroi_is_enabled();
            if (!msroi) {
                roi_reset(&s_roi);
            }

            vision_msroi_config_t cfg = {0};
            if (msroi) {
                vision_msroi_get_config(&cfg);
                if (cfg.coarse_stride < 1) cfg.coarse_stride = 1;
                if (cfg.coarse_interval < 1) cfg.coarse_interval = 1;
                if (cfg.roi_max_area_pct <= 0.0f) cfg.roi_max_area_pct = 45.0f;
            }
            const bool mod_m1 = (msroi && cfg.module_coarse_mode);
            const bool mod_m2 = (msroi && cfg.module_valid_fallback);
            const bool mod_m3 = (msroi && cfg.module_roi_refine);

            int roi_x0 = 0, roi_y0 = 0, roi_x1 = src_w_local - 1, roi_y1 = src_h_local - 1;
            bool roi_used = false;
            bool coarse_ran = false;

            int64_t t0 = esp_timer_get_time();
            if (msroi && src_w_local > 0 && src_h_local > 0) {
                seq++;
                bool run_coarse = true;
                if (mod_m1) {
                    run_coarse = (!s_roi.valid) || (cfg.coarse_interval <= 1) || ((seq % cfg.coarse_interval) == 0);
                }
                if (run_coarse) {
                    coarse_ran = true;
                    uint32_t hits = 0;
                    int cx=0, cy=0; float area=0.f;
                    int cx1=0, cy1=0, cx2=0, cy2=0;
                    bool coarse_ok = detect_color_rgb565_roi(frame, r, g, b, tol, cfg.coarse_stride,
                                                            0, 0, src_w_local - 1, src_h_local - 1,
                                                            cx, cy, area, cx1, cy1, cx2, cy2, &hits);
                    if (coarse_ok && hits >= cfg.roi_min_hits) {
                        s_roi.x1 = cx1; s_roi.y1 = cy1; s_roi.x2 = cx2; s_roi.y2 = cy2;
                        s_roi.valid = true;
                        s_roi.lost = 0;
                        roi_apply_pad(&s_roi, cfg.roi_pad, src_w_local, src_h_local);
                        if (mod_m2 && s_roi.valid) {
                            const int rw = s_roi.x2 - s_roi.x1 + 1;
                            const int rh = s_roi.y2 - s_roi.y1 + 1;
                            const float area_pct = (rw > 0 && rh > 0)
                                ? ((float)(rw * rh) * 100.0f / (float)(src_w_local * src_h_local))
                                : 100.0f;
                            if (rw < 4 || rh < 4 || area_pct > cfg.roi_max_area_pct) {
                                s_roi.valid = false;
                                s_roi.lost = 0;
                            }
                        }
                    } else {
                        if (s_roi.valid) s_roi.lost++;
                        if (s_roi.lost > cfg.roi_hold_frames) {
                            roi_reset(&s_roi);
                        }
                    }
                }

                if (s_roi.valid) {
                    roi_x0 = s_roi.x1; roi_y0 = s_roi.y1;
                    roi_x1 = s_roi.x2; roi_y1 = s_roi.y2;
                }
            }
            int64_t t1 = esp_timer_get_time();

            if (frame->format != PIXFORMAT_RGB565) {
                ESP_LOGW(TAG, "Skip fmt=%d", frame->format);
            } else {
                int cx=0, cy=0; float area=0.f; bool ok=false; uint32_t hits=0;
                bool roi_ready = (s_roi.valid);
                if (roi_ready && mod_m2) {
                    const int rw = roi_x1 - roi_x0 + 1;
                    const int rh = roi_y1 - roi_y0 + 1;
                    const float area_pct = (rw > 0 && rh > 0)
                        ? ((float)(rw * rh) * 100.0f / (float)(src_w_local * src_h_local))
                        : 100.0f;
                    if (rw < 4 || rh < 4 || area_pct > cfg.roi_max_area_pct) {
                        roi_ready = false;
                    }
                }

                if (msroi && mod_m3 && roi_ready) {
                    roi_used = true;
                    ok = detect_color_rgb565_roi(frame, r, g, b, tol, 1,
                                                 roi_x0, roi_y0, roi_x1, roi_y1,
                                                 cx, cy, area, bx1, by1, bx2, by2, &hits);
                } else {
                    if (msroi) fallback_cnt++;
                    ok = detect_color_rgb565_roi(frame, r, g, b, tol, 1,
                                                 0, 0, src_w_local - 1, src_h_local - 1,
                                                 cx, cy, area, bx1, by1, bx2, by2, &hits);
                }

                if (msroi) {
                    if (ok) {
                        s_roi.x1 = bx1; s_roi.y1 = by1; s_roi.x2 = bx2; s_roi.y2 = by2;
                        s_roi.valid = true;
                        s_roi.lost = 0;
                        roi_apply_pad(&s_roi, cfg.roi_pad, src_w_local, src_h_local);
                    } else if (s_roi.valid) {
                        s_roi.lost++;
                        if (mod_m2 && s_roi.lost == 1) {
                            roi_apply_pad(&s_roi, cfg.roi_pad + 2, src_w_local, src_h_local);
                        }
                        if (s_roi.lost > cfg.roi_hold_frames) {
                            roi_reset(&s_roi);
                        }
                    }
                }

                uartp_post_color_detect(ok, cx, cy, area);
                ESP_LOGI(TAG, "color_id=%u detect=%s cx=%d cy=%d area=%.2f%%",
                         color_id, ok?"OK":"NO", cx, cy, area*100.0f);
                ok_flag = ok;
                area_val = area;
            }
            int64_t t2 = esp_timer_get_time();

            // metrics accumulate
            busy_us_sum += (uint64_t)(t2 - t0);
            if (frame && (frame->timestamp.tv_sec || frame->timestamp.tv_usec)) {
                int64_t frame_ts_us = (int64_t)frame->timestamp.tv_sec * 1000000LL + (int64_t)frame->timestamp.tv_usec;
                if (frame_ts_us > 0 && t2 >= frame_ts_us) {
                    lat_ms_sum += (double)(t2 - frame_ts_us) / 1000.0;
                    lat_cnt++;
                }
            }
            frame_cnt++;
            if (msroi && coarse_ran) {
                coarse_us_sum += (uint64_t)(t1 - t0);
            }
            refine_us_sum += (uint64_t)(t2 - t1);
            if (msroi && roi_used) {
                roi_used_cnt++;
                if (ok_flag) roi_ok_cnt++;
                float roi_area_pct = 0.0f;
                if (s_roi.valid) {
                    const int rw = (s_roi.x2 - s_roi.x1 + 1);
                    const int rh = (s_roi.y2 - s_roi.y1 + 1);
                    if (rw > 0 && rh > 0 && src_w_local > 0 && src_h_local > 0) {
                        roi_area_pct = (float)(rw * rh) * 100.0f / (float)(src_w_local * src_h_local);
                    }
                }
                roi_area_sum += roi_area_pct;
            }
            last_q_depth = xQueueFrameIn ? (uint32_t)uxQueueMessagesWaiting(xQueueFrameIn) : 0;

            if (t2 - last_report_us >= 1000000) {
                const float elapsed = (float)(t2 - last_report_us) / 1000000.0f;
                vision_metrics_t m{};
                m.task = VISION_METRICS_TASK_COLOR;
                m.msroi_enabled = msroi;
                m.fps = (elapsed > 0.0f) ? ((float)frame_cnt / elapsed) : 0.0f;
                m.coarse_ms = (frame_cnt > 0) ? ((float)coarse_us_sum / 1000.0f / (float)frame_cnt) : 0.0f;
                m.refine_ms = (frame_cnt > 0) ? ((float)refine_us_sum / 1000.0f / (float)frame_cnt) : 0.0f;
                m.roi_hit_rate = (msroi && frame_cnt > 0) ? ((float)roi_ok_cnt * 100.0f / (float)frame_cnt) : 0.0f;
                m.roi_area_pct = (msroi && roi_used_cnt > 0) ? (roi_area_sum / (float)roi_used_cnt) : 0.0f;
                m.latency_ms = (lat_cnt > 0) ? (float)(lat_ms_sum / (double)lat_cnt) : 0.0f;
                m.cpu_pct = (elapsed > 0.0f) ? ((float)busy_us_sum / (elapsed * 1000000.0f) * 100.0f) : 0.0f;
                m.fallback_full_scan = fallback_cnt;
                m.queue_depth = last_q_depth;
                vision_metrics_publish(&m);
                vision_msroi_feedback(&m);
                char route_tag[48] = {0};
                vision_msroi_format_route_label(msroi, mod_m1, mod_m2, mod_m3, route_tag, sizeof(route_tag));

                ESP_LOGI(TAG, "metrics route=%s msroi=%d fps=%.1f coarse=%.2fms refine=%.2fms cpu=%.1f%% lat=%.2fms roi_hit=%.1f%% roi_area=%.1f%% fb=%u q=%u",
                         route_tag, m.msroi_enabled ? 1 : 0, (double)m.fps, (double)m.coarse_ms, (double)m.refine_ms,
                         (double)m.cpu_pct, (double)m.latency_ms, (double)m.roi_hit_rate, (double)m.roi_area_pct,
                         (unsigned)m.fallback_full_scan, (unsigned)m.queue_depth);

                frame_cnt = 0;
                roi_used_cnt = 0;
                roi_ok_cnt = 0;
                fallback_cnt = 0;
                coarse_us_sum = 0;
                refine_us_sum = 0;
                busy_us_sum = 0;
                lat_ms_sum = 0.0;
                lat_cnt = 0;
                roi_area_sum = 0.0f;
                last_report_us = t2;
            }

            bool handed = false;
            if (xQueueFrameOut) {
                if (xQueueSend(xQueueFrameOut, &frame, portMAX_DELAY) == pdPASS) {
                    handed = true;
                    frame = NULL;
                }
            }
            if (handed && ok_flag) {
                float area_max = 0.85f;
                if (area_val >= 0.0005f && area_val <= area_max) {
                    lcd_overlay_post_bbox_from_src(src_w_local, src_h_local, bx1, by1, bx2, by2, 0x07E0);
                }
            }
            if (frame) {
                esp_camera_fb_return(frame);
                frame = NULL;
            }
            vTaskDelay(1);
        }
    }
}

void register_color_detection(QueueHandle_t frame_in, QueueHandle_t frame_out, uint8_t color_id)
{
	xQueueFrameIn = frame_in;
	xQueueFrameOut = frame_out;
	if (s_color_detect_task) {
		vTaskDelete(s_color_detect_task);
		s_color_detect_task = NULL;
	}
	xTaskCreatePinnedToCore(color_detection_task, TAG, 8 * 1024, (void*)(uint32_t)color_id, 5, &s_color_detect_task, 1);
}

void unregister_color_detection(void)
{
	if (s_color_detect_task) {
		vTaskDelete(s_color_detect_task);
		s_color_detect_task = NULL;
	}
	xQueueFrameIn = NULL;
	xQueueFrameOut = NULL;
}











