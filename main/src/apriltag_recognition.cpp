#include "apriltag_recognition.h"
#include "vision_msroi.h"
#include "vision_metrics.h"
#include "esp_timer.h"
#include "lcd.h"  // 用于� LCD 任务投�叠加�

static const char *TAG = "apriltag_recognition";
static tagformat_t tag_format;
static QueueHandle_t xQueueFrameIn = NULL;
static QueueHandle_t xQueueFrameOut = NULL;
static TaskHandle_t s_apriltag_task = NULL;

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
    if (pad < 0) pad = 0;
    if (pad > 4) pad = 4; // cap ROI padding to avoid oversized ROI in AprilTag
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

apriltag_detector_t *td=NULL;
apriltag_family_t *tf=NULL;
static void apriltag_cleanup(void)
{
    if (td && tf) {
        apriltag_detector_remove_family(td, tf);
    }
    if (tf) {
        // Destroy the tag family with the matching helper
        if (tag_format == tag16h5) {
            tag16h5_destroy(tf);
        } else if (tag_format == tag36h11) {
            tag36h11_destroy(tf);
        }
        tf = NULL;
    }
    if (td) {
        apriltag_detector_destroy(td);
        td = NULL;
    }
}


// 物理参数：标准标签边长（米）用于距�估�
// 与表格一致：�识别时返� -1
static constexpr float TAG_SIZE_M = 0.05f; // 5cm
// 镜头焦距与传感器宽度（OV2640�1/4" 光�幅�，�约 2.56mm�
static constexpr float LENS_F_MM        = 3.6f;   // 用户提供� 3.6mm
static constexpr float SENSOR_WIDTH_MM  = 2.56f;  // 近似值，�按实测修�
// MSROI tuning for AprilTag (area is percent of full frame)
static constexpr float TAG_ROI_MAX_AREA_PCT  = 50.0f;
static constexpr float TAG_ROI_DEC2_AREA_PCT = 10.0f;
static constexpr float TAG_ROI_DEC1_AREA_PCT = 4.0f;

static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo; 
    if (v > hi) return hi; 
    return v;
}

// 计算四边形面�（顶点按顺时�/逆时针顺序）
static float quad_area(const apriltag_detection_t *det){
    // 使用 Shoelace �式分成两�三�形
    const float x0 = det->p[0][0], y0 = det->p[0][1];
    const float x1 = det->p[1][0], y1 = det->p[1][1];
    const float x2 = det->p[2][0], y2 = det->p[2][1];
    const float x3 = det->p[3][0], y3 = det->p[3][1];
    float a1 = 0.5f * fabsf(x0*y1 + x1*y2 + x2*y0 - y0*x1 - y1*x2 - y2*x0);
    float a2 = 0.5f * fabsf(x0*y2 + x2*y3 + x3*y0 - y0*x2 - y2*x3 - y3*x0);
    return a1 + a2;
}

// 估�标签�边长�像素（取四条边的平均像素长度）
static float mean_side_px(const apriltag_detection_t *det){
    auto L = [](float x0,float y0,float x1,float y1){ float dx=x1-x0, dy=y1-y0; return sqrtf(dx*dx+dy*dy); };
    float d01 = L(det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1]);
    float d12 = L(det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1]);
    float d23 = L(det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1]);
    float d30 = L(det->p[3][0], det->p[3][1], det->p[0][0], det->p[0][1]);
    return 0.25f * (d01 + d12 + d23 + d30);
}

static inline int clampi(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

typedef struct {
    int id;
    float cx;
    float cy;
    float area_px;
    float yaw_deg;
    float side_px;
    int x1, y1, x2, y2;
} apriltag_det_t;

static bool apriltag_detect_best(image_u8_t *im, float decimate,
                                 int offset_x, int offset_y,
                                 int full_w, int full_h,
                                 apriltag_det_t *out_det)
{
    if (!td || !im || !out_det) return false;
    if (decimate < 1.0f) decimate = 1.0f;
    float prev_dec = td->quad_decimate;
    td->quad_decimate = decimate;

    zarray_t *detections = apriltag_detector_detect(td, im);
    td->quad_decimate = prev_dec;

    apriltag_detection_t *best = NULL;
    float best_area = 0.0f;
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det; zarray_get(detections, i, &det);
        float a = quad_area(det);
        if (a > best_area) { best_area = a; best = det; }
    }

    if (!best) {
        apriltag_detections_destroy(detections);
        return false;
    }

    // compute bbox in full image coordinates
    float minx = fminf(fminf(best->p[0][0], best->p[1][0]), fminf(best->p[2][0], best->p[3][0]));
    float maxx = fmaxf(fmaxf(best->p[0][0], best->p[1][0]), fmaxf(best->p[2][0], best->p[3][0]));
    float miny = fminf(fminf(best->p[0][1], best->p[1][1]), fminf(best->p[2][1], best->p[3][1]));
    float maxy = fmaxf(fmaxf(best->p[0][1], best->p[1][1]), fmaxf(best->p[2][1], best->p[3][1]));
    int x1 = (int)floorf(minx) + offset_x;
    int y1 = (int)floorf(miny) + offset_y;
    int x2 = (int)ceilf(maxx) + offset_x;
    int y2 = (int)ceilf(maxy) + offset_y;
    if (full_w > 0 && full_h > 0) {
        x1 = clampi(x1, 0, full_w - 1);
        y1 = clampi(y1, 0, full_h - 1);
        x2 = clampi(x2, 0, full_w - 1);
        y2 = clampi(y2, 0, full_h - 1);
        if (x2 < x1) x2 = x1;
        if (y2 < y1) y2 = y1;
    }

    // angle from edge p0->p1 (clockwise positive, 0..360)
    float x0 = best->p[0][0], y0 = best->p[0][1];
    float x1p = best->p[1][0], y1p = best->p[1][1];
    float ang = atan2f(-(y1p - y0), (x1p - x0)) * (180.0f / M_PI);
    if (ang < 0) ang += 360.0f;

    out_det->id = best->id;
    out_det->cx = best->c[0] + offset_x;
    out_det->cy = best->c[1] + offset_y;
    out_det->area_px = best_area;
    out_det->yaw_deg = ang;
    out_det->side_px = mean_side_px(best);
    out_det->x1 = x1; out_det->y1 = y1; out_det->x2 = x2; out_det->y2 = y2;

    apriltag_detections_destroy(detections);
    return true;
}

static bool report_apriltag_result(const apriltag_det_t *det,
                                   int full_w, int full_h,
                                   int *out_x1, int *out_y1, int *out_x2, int *out_y2)
{
    if (!det || full_w <= 0 || full_h <= 0) {
        uartp_post_apriltag(false, -1, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f);
        return false;
    }

    float cx_norm = (det->cx - (float)full_w * 0.5f) / ((float)full_w * 0.5f);
    float cy_norm = ((float)full_h * 0.5f - det->cy) / ((float)full_h * 0.5f);
    cx_norm = clampf(cx_norm, -1.0f, 1.0f);
    cy_norm = clampf(cy_norm, -1.0f, 1.0f);

    const float img_area = (float)full_w * (float)full_h;
    float area_pct = img_area > 1.0f ? clampf(det->area_px * 100.0f / img_area, 0.0f, 100.0f) : 0.0f;

    float f_pix = (LENS_F_MM / SENSOR_WIDTH_MM) * (float)full_w;
    float dist_cm = (det->side_px > 1.0f) ? (TAG_SIZE_M * f_pix / det->side_px) * 100.0f : -1.0f;

    ESP_LOGI(TAG, "AprilTag id=%d cx=%.3f cy=%.3f area=%.2f%% yaw=%.1f deg dist=%.1f cm",
             det->id, cx_norm, cy_norm, area_pct, det->yaw_deg, dist_cm);

    uartp_post_apriltag(true, det->id, cx_norm, cy_norm, area_pct, det->yaw_deg, dist_cm);

    if (out_x1 && out_y1 && out_x2 && out_y2) {
        *out_x1 = det->x1; *out_y1 = det->y1;
        *out_x2 = det->x2; *out_y2 = det->y2;
    }
    return true;
}

static void ApriTag_init(tagformat_t families) 
{
    if (families != tag16h5 && families != tag36h11) {
        families = tag16h5;  // 默�使� tag16h5，避免空指针
    }
    tag_format = families;
    if (families == tag16h5) {
        tf = tag16h5_create();   // 仅识� tag16h5
    }
    else if (families == tag36h11) {
        tf = tag36h11_create();  // 仅识� tag36h11
    }
    td = apriltag_detector_create();
    if (td && tf) {
        apriltag_detector_add_family(td, tf);
    }
    td->quad_sigma = 0.0;
    td->quad_decimate = 4.0;  //降低图片分辨率，经实验�参数可大幅提升�测时间且不影响�测结�
    td->refine_edges = 0;
    td->decode_sharpening = 0;
    td->nthreads = 1;
    td->debug = 0;
}

void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;
    apriltag_cleanup();
    ApriTag_init(tag_format);

    // metrics
    uint32_t frame_cnt = 0;
    uint32_t roi_used_cnt = 0;
    uint32_t roi_ok_cnt = 0;
    uint32_t det_ok_cnt = 0;
    uint32_t fallback_cnt = 0;
    uint64_t coarse_us_sum = 0;
    uint64_t refine_us_sum = 0;
    uint64_t busy_us_sum = 0;
    double lat_ms_sum = 0.0;
    uint32_t lat_cnt = 0;
    float roi_area_sum = 0.0f;
    uint32_t last_q_depth = 0;
    int64_t last_report_us = esp_timer_get_time();

    roi_reset(&s_roi);

    while (1)
    {
        if (xQueueReceive(xQueueFrameIn, &frame, portMAX_DELAY))
        {
            if (!frame) continue;

            int src_w = (int)frame->width;
            int src_h = (int)frame->height;
            int bx1 = 0, by1 = 0, bx2 = 0, by2 = 0;
            bool ok = false;

            if (frame->format != PIXFORMAT_GRAYSCALE) {
                ESP_LOGW(TAG, "Skip fmt=%d, need GRAYSCALE", frame->format);
                uartp_post_apriltag(false, -1, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f);
            } else {
                bool msroi = vision_msroi_is_enabled();
                if (!msroi) {
                    roi_reset(&s_roi);
                }

                vision_msroi_config_t cfg = {0};
                if (msroi) {
                    vision_msroi_get_config(&cfg);
                    if (cfg.coarse_stride < 1) cfg.coarse_stride = 1;
                    if (cfg.roi_max_area_pct <= 0.0f) cfg.roi_max_area_pct = TAG_ROI_MAX_AREA_PCT;
                }
                const bool mod_m1 = (msroi && cfg.module_coarse_mode);
                const bool mod_m2 = (msroi && cfg.module_valid_fallback);
                const bool mod_m3 = (msroi && cfg.module_roi_refine);

                bool roi_used = false;
                bool coarse_ran = false;
                bool coarse_ok = false;
                bool refine_ok = false;
                apriltag_det_t det_coarse = {0};
                apriltag_det_t det_refine = {0};

                int64_t t0 = esp_timer_get_time();

                bool roi_usable = false;
                float roi_area_pct = 0.0f;
                int roi_x0 = 0, roi_y0 = 0, roi_x1 = src_w - 1, roi_y1 = src_h - 1;
                if (msroi && s_roi.valid) {
                    const int rw = (s_roi.x2 - s_roi.x1 + 1);
                    const int rh = (s_roi.y2 - s_roi.y1 + 1);
                    if (rw > 0 && rh > 0 && src_w > 0 && src_h > 0) {
                        roi_area_pct = (float)(rw * rh) * 100.0f / (float)(src_w * src_h);
                    }
                    if (roi_area_pct > 0.0f && roi_area_pct <= cfg.roi_max_area_pct) {
                        roi_usable = true;
                        roi_x0 = s_roi.x1; roi_y0 = s_roi.y1;
                        roi_x1 = s_roi.x2; roi_y1 = s_roi.y2;
                    } else if (roi_area_pct > cfg.roi_max_area_pct) {
                        s_roi.lost = (uint8_t)(cfg.roi_hold_frames + 1);
                    }
                }

                // Coarse scan only when ROI is missing/too large or recently lost.
                if (msroi && (!mod_m1 || !roi_usable || s_roi.lost > 0)) {
                    coarse_ran = true;
                    image_u8_t im = {
                        .width = (int32_t)src_w,
                        .height = (int32_t)src_h,
                        .stride = (int32_t)src_w,
                        .buf = frame->buf
                    };
                    float dec = (float)cfg.coarse_stride;
                    if (dec < 4.0f) dec = 4.0f;   // keep coarse no slower than baseline
                    if (dec > 8.0f) dec = 8.0f;
                    coarse_ok = apriltag_detect_best(&im, dec, 0, 0, src_w, src_h, &det_coarse);
                }

                int64_t t1 = esp_timer_get_time();

                bool roi_ready = roi_usable;
                if (roi_ready && mod_m2) {
                    int rw = roi_x1 - roi_x0 + 1;
                    int rh = roi_y1 - roi_y0 + 1;
                    if (rw < 4 || rh < 4) {
                        roi_ready = false;
                    }
                }

                if (msroi && mod_m3 && roi_ready && !(coarse_ran && coarse_ok)) {
                    roi_used = true;
                    int rw = roi_x1 - roi_x0 + 1;
                    int rh = roi_y1 - roi_y0 + 1;
                    if (rw > 0 && rh > 0) {
                        image_u8_t im_roi = {
                            .width = (int32_t)rw,
                            .height = (int32_t)rh,
                            .stride = (int32_t)src_w,
                            .buf = frame->buf + (size_t)roi_y0 * (size_t)src_w + (size_t)roi_x0
                        };
                        float refine_dec = 3.0f;
                        if (roi_area_pct <= TAG_ROI_DEC1_AREA_PCT) {
                            refine_dec = 1.0f;
                        } else if (roi_area_pct <= TAG_ROI_DEC2_AREA_PCT) {
                            refine_dec = 2.0f;
                        }
                        refine_ok = apriltag_detect_best(&im_roi, refine_dec, roi_x0, roi_y0, src_w, src_h, &det_refine);
                    }
                } else {
                    image_u8_t im = {
                        .width = (int32_t)src_w,
                        .height = (int32_t)src_h,
                        .stride = (int32_t)src_w,
                        .buf = frame->buf
                    };
                    refine_ok = apriltag_detect_best(&im, 4.0f, 0, 0, src_w, src_h, &det_refine);
                }

                int64_t t2 = esp_timer_get_time();

                const apriltag_det_t *final_det = NULL;
                if (refine_ok) final_det = &det_refine;
                else if (coarse_ok) final_det = &det_coarse;

                if (msroi && (!roi_used || (mod_m2 && !roi_ready))) {
                    fallback_cnt++;
                }

                if (final_det) {
                    ok = report_apriltag_result(final_det, src_w, src_h, &bx1, &by1, &bx2, &by2);
                } else {
                    uartp_post_apriltag(false, -1, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f);
                }

                if (msroi) {
                    if (final_det) {
                        s_roi.x1 = final_det->x1; s_roi.y1 = final_det->y1;
                        s_roi.x2 = final_det->x2; s_roi.y2 = final_det->y2;
                        s_roi.valid = true;
                        s_roi.lost = 0;
                        roi_apply_pad(&s_roi, cfg.roi_pad, src_w, src_h);
                    } else if (s_roi.valid) {
                        s_roi.lost++;
                        if (mod_m2 && s_roi.lost == 1) {
                            roi_apply_pad(&s_roi, cfg.roi_pad + 2, src_w, src_h);
                        }
                        if (s_roi.lost > cfg.roi_hold_frames) {
                            roi_reset(&s_roi);
                        }
                    }
                }

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
                    if (final_det) roi_ok_cnt++;
                    float roi_area_pct_now = 0.0f;
                    if (s_roi.valid) {
                        const int rw = (s_roi.x2 - s_roi.x1 + 1);
                        const int rh = (s_roi.y2 - s_roi.y1 + 1);
                        if (rw > 0 && rh > 0 && src_w > 0 && src_h > 0) {
                            roi_area_pct_now = (float)(rw * rh) * 100.0f / (float)(src_w * src_h);
                        }
                    }
                    roi_area_sum += roi_area_pct_now;
                }
                if (final_det) det_ok_cnt++;
                last_q_depth = xQueueFrameIn ? (uint32_t)uxQueueMessagesWaiting(xQueueFrameIn) : 0;

                if (t2 - last_report_us >= 1000000) {
                    const float elapsed = (float)(t2 - last_report_us) / 1000000.0f;
                    vision_metrics_t m{};
                    m.task = VISION_METRICS_TASK_TAG;
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

                    const float det_rate = (frame_cnt > 0) ? ((float)det_ok_cnt * 100.0f / (float)frame_cnt) : 0.0f;
                    char route_tag[48] = {0};
                    vision_msroi_format_route_label(msroi, mod_m1, mod_m2, mod_m3, route_tag, sizeof(route_tag));
                    ESP_LOGI(TAG, "metrics route=%s msroi=%d fps=%.1f coarse=%.2fms refine=%.2fms cpu=%.1f%% lat=%.2fms roi_hit=%.1f%% roi_area=%.1f%% det=%.1f%% fb=%u q=%u",
                             route_tag, m.msroi_enabled ? 1 : 0, (double)m.fps, (double)m.coarse_ms, (double)m.refine_ms,
                             (double)m.cpu_pct, (double)m.latency_ms, (double)m.roi_hit_rate, (double)m.roi_area_pct,
                             (double)det_rate, (unsigned)m.fallback_full_scan, (unsigned)m.queue_depth);

                    frame_cnt = 0;
                    roi_used_cnt = 0;
                    roi_ok_cnt = 0;
                    det_ok_cnt = 0;
                    fallback_cnt = 0;
                    coarse_us_sum = 0;
                    refine_us_sum = 0;
                    busy_us_sum = 0;
                    lat_ms_sum = 0.0;
                    lat_cnt = 0;
                    roi_area_sum = 0.0f;
                    last_report_us = t2;
                }
            }

            bool handed = false;
            if (xQueueFrameOut) {
                if (xQueueSend(xQueueFrameOut, &frame, portMAX_DELAY) == pdPASS) {
                    handed = true;
                    frame = NULL;
                }
            }
            if (!handed && frame) {
                esp_camera_fb_return(frame);
                frame = NULL;
            }

            if (handed && ok) {
                lcd_overlay_post_bbox_from_src(src_w, src_h, bx1, by1, bx2, by2, 0xFFE0);
            }
        }
    }
}
void register_apriltag_detection(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const tagformat_t tag)
{
    xQueueFrameIn = frame_i;
    xQueueFrameOut = frame_o;
    tag_format = tag;
    if (s_apriltag_task) {
        vTaskDelete(s_apriltag_task);
        s_apriltag_task = NULL;
    }
    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 4, &s_apriltag_task, 0);
}

void unregister_apriltag_detection(void)
{
    if (s_apriltag_task) {
        vTaskDelete(s_apriltag_task);
        s_apriltag_task = NULL;
    }
    xQueueFrameIn = NULL;
    xQueueFrameOut = NULL;
}
