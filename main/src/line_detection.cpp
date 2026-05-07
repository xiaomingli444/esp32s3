#include "line_detection.h"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdlib>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include "camera.h"
#include "uartp.h"
#include "sys.h"
#include "vision_msroi.h"
#include "vision_metrics.h"
#include "esp_timer.h"

static const char *TAG = "line_follow";
static QueueHandle_t s_in = nullptr;
static QueueHandle_t s_out = nullptr;
static TaskHandle_t s_line_task = nullptr;

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

// OpenCV 中间结果，仅在本任务内部使用
static cv::Mat s_mask;

// ================== 安装方向（固定） ==================
// 相机面向前进方向，“前方”在图像“上方”
static constexpr bool kForwardIsUp = true;

// ===== 基础：RGB565 解码成 8bit RGB（高字节在前） =====
static inline uint8_t up5_to_8(uint8_t v5) { return (uint8_t)((v5 * 255 + 15) / 31); }
static inline uint8_t up6_to_8(uint8_t v6) { return (uint8_t)((v6 * 255 + 31) / 63); }
static inline void rgb565_hi_first_to_rgb8(uint8_t hi, uint8_t lo,
                                           uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    uint16_t pix = ((uint16_t)hi << 8) | lo; // HI: rrrrrggg, LO: gggbbbbb
    uint8_t r5 = (pix >> 11) & 0x1F;
    uint8_t g6 = (pix >>  5) & 0x3F;
    uint8_t b5 =  pix        & 0x1F;
    *r8 = up5_to_8(r5);
    *g8 = up6_to_8(g6);
    *b8 = up5_to_8(b5);
}


// coarse scan for ROI (sampled)
static bool coarse_scan_rgb565(const camera_fb_t *fb,
                               uint8_t R, uint8_t G, uint8_t B, int tol, int step,
                               int &x1, int &y1, int &x2, int &y2, uint32_t &hits)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) return false;
    const int W = fb->width;
    const int H = fb->height;
    const uint8_t *base = (const uint8_t *)fb->buf;

    if (step < 1) step = 1;
    int minx = W, miny = H, maxx = -1, maxy = -1;
    uint32_t cnt = 0;

    for (int y = 0; y < H; y += step) {
        const uint8_t *row = base + (size_t)y * W * 2;
        for (int x = 0; x < W; x += step) {
            uint8_t r8, g8, b8;
            rgb565_hi_first_to_rgb8(row[2 * x], row[2 * x + 1], &r8, &g8, &b8);
            int dr = (int)r8 - (int)R; if (dr < 0) dr = -dr;
            int dg = (int)g8 - (int)G; if (dg < 0) dg = -dg;
            int db = (int)b8 - (int)B; if (db < 0) db = -db;
            if (dr <= tol && dg <= tol && db <= tol) {
                cnt++;
                if (x < minx) minx = x;
                if (x > maxx) maxx = x;
                if (y < miny) miny = y;
                if (y > maxy) maxy = y;
            }
        }
    }

    hits = cnt;
    if (cnt == 0) return false;
    x1 = minx; y1 = miny; x2 = maxx; y2 = maxy;
    return true;
}

// =============== 仅基于 RGB 绝对差的二值化 ===============
// 目标颜色 -> 255（白）；其他 -> 0（黑）
// 判定逻辑与 color_detection.cpp 中 detect_color_rgb565 一致：
//   dr = |r - R|, dg = |g - G|, db = |b - B|
//   dr <= tol && dg <= tol && db <= tol 视为命中目标颜色
static bool build_binary_mask_rgb_only_roi(const camera_fb_t *fb,
                                           uint8_t R, uint8_t G, uint8_t B, int tol,
                                           int rx0, int ry0, int rx1, int ry1,
                                           cv::Mat &mask_out)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) {
        return false;
    }
    const int W = fb->width;
    const int H = fb->height;

    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    if (rx1 >= W) rx1 = W - 1;
    if (ry1 >= H) ry1 = H - 1;
    if (rx0 > rx1 || ry0 > ry1) {
        return false;
    }

    mask_out.create(H, W, CV_8UC1);
    mask_out.setTo(0);

    const uint8_t *base = (const uint8_t *)fb->buf;

    for (int y = ry0; y <= ry1; ++y) {
        const uint8_t *row = base + (size_t)y * W * 2;
        uint8_t *mrow = mask_out.ptr<uint8_t>(y);
        for (int x = rx0; x <= rx1; ++x) {
            uint8_t r8, g8, b8;
            rgb565_hi_first_to_rgb8(row[2 * x], row[2 * x + 1], &r8, &g8, &b8);

            int dr = (int)r8 - (int)R; if (dr < 0) dr = -dr;
            int dg = (int)g8 - (int)G; if (dg < 0) dg = -dg;
            int db = (int)b8 - (int)B; if (db < 0) db = -db;

            mrow[x] = (dr <= tol && dg <= tol && db <= tol) ? 255 : 0;
        }
    }

    return true;
}

// åœ¨äºŒå€¼mask ä¸Šåšä¸€æ¬¡ç®€å•è†¨èƒ€ï¼šçº¢çº¿å˜ç²—ã€å¡«å…¥å°é»‘æ´ž
static void dilate_mask_inplace(cv::Mat &mask)
{
    if (mask.empty()) return;
    static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
}

// =============== 二值 mask 上估计几何量（前方所在带做质心回归） ===============
static bool estimate_line_geometry_mask(const cv::Mat &mask,
                                        float &offset_x, float &heading_deg, float &width_pct)
{
    if (mask.empty() || mask.type() != CV_8UC1) {
        return false;
    }
    const int W = mask.cols;
    const int H = mask.rows;

    // 固定使用“上 1/3 带”作为前方区域
    const int y0 = 0;
    const int y1 = std::max(0, (H / 3) - 1);
    const int band_h = std::max(1, y1 - y0 + 1);

    // 全带质心
    uint32_t count = 0;
    uint64_t sumX = 0;

    // 分片质心用于估算斜率（航向）
    const int S = 5;
    uint32_t cntS[S] = {0};
    uint64_t sumXS[S] = {0};

    for (int y = y0; y <= y1; ++y) {
        const uint8_t *row = mask.ptr<uint8_t>(y);
        int sidx = (int)((int64_t)(y - y0) * S / (band_h));
        if (sidx < 0) sidx = 0;
        if (sidx >= S) sidx = S - 1;
        for (int x = 0; x < W; ++x) {
            if (row[x]) {
                ++count;
                sumX += (uint32_t)x;
                ++cntS[sidx];
                sumXS[sidx] += (uint32_t)x;
            }
        }
    }

    if (count == 0) {
        offset_x = 0.0f;
        heading_deg = 0.0f;
        width_pct = 0.0f;
        return false;
    }

    const float cx = (float)sumX / (float)count;
    offset_x = (cx - (W - 1) * 0.5f) / ((W - 1) * 0.5f); // -1..+1 左负右正

    width_pct = (float)count / (float)(W * H) * 100.0f;
    if (width_pct < 0.0f) width_pct = 0.0f;
    if (width_pct > 100.0f) width_pct = 100.0f;

    // 航向：用片段质心的上下差估计与竖直方向夹角
    int top = -1, bot = -1;
    const uint32_t minPerSlice = (uint32_t)(W * (band_h / (float)S) * 0.01f) + 30; // 至少 ~1% + 30px
    for (int i = 0; i < S; ++i) {
        if (cntS[i] >= minPerSlice) { top = i; break; }
    }
    for (int i = S - 1; i >= 0; --i) {
        if (cntS[i] >= minPerSlice) { bot = i; break; }
    }
    if (top >= 0 && bot >= 0 && bot > top) {
        float cx_top = (float)sumXS[top] / (float)cntS[top];
        float cx_bot = (float)sumXS[bot] / (float)cntS[bot];
        float y_top = (float)y0 + (top + 0.5f) * (band_h / (float)S);
        float y_bot = (float)y0 + (bot + 0.5f) * (band_h / (float)S);
        float dx = cx_top - cx_bot;
        float dy = y_top - y_bot;
        if (dy == 0.0f) dy = 1.0f;
        const float PI_F = 3.14159265358979323846f;
        heading_deg = -atan2f(dx, dy) * 180.0f / PI_F; // 竖直向上为 0°，右倾为正
    } else {
        heading_deg = 0.0f;
    }

    return true;
}

// =============== mask 上的“一行”连续段 ===============
static inline int longest_run_in_row_mask(const cv::Mat &mask,
                                          int y, int xv,
                                          int &left_run_at_xv, int &right_run_at_xv)
{
    const int W = mask.cols;
    const uint8_t *row = mask.ptr<uint8_t>(y);

    left_run_at_xv = 0;
    for (int x = xv; x >= 0; --x) {
        if (row[x]) left_run_at_xv++;
        else break;
    }
    right_run_at_xv = 0;
    for (int x = xv + 1; x < W; ++x) {
        if (row[x]) right_run_at_xv++;
        else break;
    }
    return left_run_at_xv + right_run_at_xv;
}

// =============== mask 上的路口识别（CROSS & T） ===============
static void detect_intersection_mask(const cv::Mat &mask,
                                     uint8_t &type, float &conf)
{
    type = UARTP_INTXN_NONE;
    conf = 0.0f;
    if (mask.empty() || mask.type() != CV_8UC1) {
        return;
    }

    const int W = mask.cols;
    const int H = mask.rows;

    uint16_t *row_cnt = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * H, MALLOC_CAP_DEFAULT);
    uint16_t *col_cnt = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * W, MALLOC_CAP_DEFAULT);
    if (!row_cnt || !col_cnt) {
        if (row_cnt) heap_caps_free(row_cnt);
        if (col_cnt) heap_caps_free(col_cnt);
        ESP_LOGW(TAG, "detect_intersection(mask): OOM row/col");
        return;
    }
    std::memset(row_cnt, 0, sizeof(uint16_t) * H);
    std::memset(col_cnt, 0, sizeof(uint16_t) * W);

    for (int y = 0; y < H; ++y) {
        const uint8_t *row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) {
            if (row[x]) {
                if (row_cnt[y] != 0xFFFF) row_cnt[y]++;
                if (col_cnt[x] != 0xFFFF) col_cnt[x]++;
            }
        }
    }

    // 找到“横线带”（按行覆盖率连续区间）
    int y_peak = 0; uint16_t rv = 0;
    for (int y = 0; y < H; ++y) if (row_cnt[y] > rv) { rv = row_cnt[y]; y_peak = y; }

    const float ROW_STR_TH = 0.45f; // 行覆盖率阈值（判定属于横线带）
    int y_top = y_peak, y_bot = y_peak;
    while (y_top - 1 >= 0 && (float)row_cnt[y_top - 1] / (float)W >= ROW_STR_TH) y_top--;
    while (y_bot + 1 <  H && (float)row_cnt[y_bot + 1] / (float)W >= ROW_STR_TH) y_bot++;
    const int y_mid = (y_top + y_bot) / 2;
    const int band_h = std::max(1, y_bot - y_top + 1);

    const float h_row_ratio = (float)row_cnt[y_mid] / (float)W;

    // 在“横线带外”统计每列命中，寻找竖线列
    uint16_t *col_cnt_ex = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * W, MALLOC_CAP_DEFAULT);
    if (!col_cnt_ex) {
        heap_caps_free(row_cnt);
        heap_caps_free(col_cnt);
        ESP_LOGW(TAG, "detect_intersection(mask): OOM col_cnt_ex");
        return;
    }
    std::memset(col_cnt_ex, 0, sizeof(uint16_t) * W);

    // 上半部分
    for (int y = 0; y < y_top; ++y) {
        const uint8_t *row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) {
            if (row[x]) if (col_cnt_ex[x] != 0xFFFF) col_cnt_ex[x]++;
        }
    }
    // 下半部分
    for (int y = y_bot + 1; y < H; ++y) {
        const uint8_t *row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) {
            if (row[x]) if (col_cnt_ex[x] != 0xFFFF) col_cnt_ex[x]++;
        }
    }

    int best_x = 0; rv = 0;
    for (int x = 0; x < W; ++x) if (col_cnt_ex[x] > rv) { rv = col_cnt_ex[x]; best_x = x; }

    // 在交点 (y_mid, best_x) 精确量两件事：
    // 1) 横线的左右连续长度（以交点列为中心）
    int L = 0, Rr = 0;
    int h_run = longest_run_in_row_mask(mask, y_mid, best_x, L, Rr);
    float h_cont = (float)h_run / (float)W;

    // 2) 竖线在横线带外的上下连续长度
    auto run_in_col_from = [&](int start_y, int step) {
        int cnt = 0;
        for (int y = start_y; y >= 0 && y < H; y += step) {
            if (y >= y_top && y <= y_bot) continue;
            const uint8_t *row = mask.ptr<uint8_t>(y);
            if (row[best_x]) cnt++;
            else break;
        }
        return cnt;
    };
    int up_run   = run_in_col_from(y_top - 1, -1); // 向上（前方）
    int down_run = run_in_col_from(y_bot + 1, +1); // 向下（后方）
    float up_ratio   = (float)up_run   / (float)H;
    float down_ratio = (float)down_run / (float)H;

    heap_caps_free(row_cnt);
    heap_caps_free(col_cnt);
    heap_caps_free(col_cnt_ex);

    // 规则（仅 CROSS/T）：
    // 横向要求：行覆盖率强 + 左右连续段足够
    const float H_ROW_STR_TH  = 0.40f;
    const float H_CONT_STR_TH = 0.25f;
    bool horiz_ok = (h_row_ratio >= H_ROW_STR_TH) && (h_cont >= H_CONT_STR_TH);

    // 纵向强弱（相对整幅高度归一化）
    const float V_CONT_STR_TH  = 0.12f; // 纵向“强”
    const float V_CONT_WEAK_TH = 0.03f; // 纵向“弱/几乎没有”
    bool up_str    = (up_ratio   >= V_CONT_STR_TH);
    bool down_str  = (down_ratio >= V_CONT_STR_TH);
    bool up_weak   = (up_ratio   <= V_CONT_WEAK_TH);
    bool down_weak = (down_ratio <= V_CONT_WEAK_TH);

    ESP_LOGI(TAG, "diag(mask): h_row=%.2f h_cont=%.2f up=%.2f down=%.2f",
             h_row_ratio, h_cont, up_ratio, down_ratio);

    if (horiz_ok) {
        // 优先判 T：横线在前方，竖线“上强下弱”
        if (up_str && down_weak) {
            type = UARTP_INTXN_T;
            conf = std::max(0.0f, std::min(1.0f,
                       (0.6f * h_cont + 0.4f * h_row_ratio) *
                       (0.8f * up_ratio + 0.2f) *
                       (1.0f - 0.5f * down_ratio)));
            return;
        }

        // CROSS：横线强，且竖线上下都强
        if (up_str && down_str) {
            type = UARTP_INTXN_CROSS;
            float symmetry = (float)std::min(up_run, down_run) /
                             (float)std::max(1, std::max(up_run, down_run));
            conf = std::max(0.0f, std::min(1.0f,
                       0.5f * h_cont +
                       0.3f * h_row_ratio +
                       0.2f * symmetry));
            return;
        }
    }

    type = UARTP_INTXN_NONE;
    conf = 0.0f;
}

// =============== 将 mask 写回 RGB565 帧缓冲（黑底白线） ===============
static void overwrite_frame_with_mask_rgb565(const cv::Mat &mask, camera_fb_t *fb)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) {
        return;
    }
    const int W = fb->width;
    const int H = fb->height;
    if (mask.empty() || mask.cols != W || mask.rows != H || mask.type() != CV_8UC1) {
        return;
    }

    uint16_t *dst = (uint16_t *)fb->buf;
    for (int y = 0; y < H; ++y) {
        const uint8_t *mrow = mask.ptr<uint8_t>(y);
        uint16_t *drow = dst + (size_t)y * W;
        for (int x = 0; x < W; ++x) {
            drow[x] = mrow[x] ? 0xFFFF : 0x0000; // 白线、黑背景
        }
    }
}

// ========================= 主任务 =========================
static void line_follow_task(void *arg)
{
    uint8_t color_id = (uint32_t)arg & 0xFF;

    uint8_t r = 255, g = 255, b = 255;
    if (!nvs_read_rgb(color_id, &r, &g, &b)) {
        ESP_LOGW(TAG, "NVS no RGB for id=%u, fallback R=%u G=%u B=%u", color_id, r, g, b);
    } else {
        ESP_LOGI(TAG, "Use NVS RGB id=%u -> R=%u G=%u B=%u", color_id, r, g, b);
    }

    int tol = 25;
    ESP_LOGI(TAG, "Line follow RGB target=(%u,%u,%u) tol=%d", r, g, b, tol);

    camera_fb_t *frame = nullptr;
    static int t_seq = 0, c_seq = 0;
    static float t_conf_s = 0.0f, c_conf_s = 0.0f;

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
        if (!xQueueReceive(s_in, &frame, portMAX_DELAY)) continue;
        if (!frame) continue;

        bool handed = false;
        do {
            if (frame->format != PIXFORMAT_RGB565) {
                ESP_LOGW(TAG, "Skip fmt=%d, need RGB565", frame->format);
                break;
            }

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

            const int src_w = (int)frame->width;
            const int src_h = (int)frame->height;
            int roi_x0 = 0, roi_y0 = 0, roi_x1 = src_w - 1, roi_y1 = src_h - 1;
            bool roi_used = false;
            bool coarse_ran = false;

            int64_t t0 = esp_timer_get_time();
            if (msroi && src_w > 0 && src_h > 0) {
                seq++;
                bool run_coarse = true;
                if (mod_m1) {
                    run_coarse = (!s_roi.valid) || (cfg.coarse_interval <= 1) || ((seq % cfg.coarse_interval) == 0);
                }
                if (run_coarse) {
                    coarse_ran = true;
                    int cx1=0, cy1=0, cx2=0, cy2=0; uint32_t hits=0;
                    bool coarse_ok = coarse_scan_rgb565(frame, r, g, b, tol, cfg.coarse_stride,
                                                       cx1, cy1, cx2, cy2, hits);
                    if (coarse_ok && hits >= cfg.roi_min_hits) {
                        s_roi.x1 = cx1; s_roi.y1 = cy1; s_roi.x2 = cx2; s_roi.y2 = cy2;
                        s_roi.valid = true;
                        s_roi.lost = 0;
                        roi_apply_pad(&s_roi, cfg.roi_pad, src_w, src_h);
                        if (mod_m2 && s_roi.valid) {
                            const int rw = s_roi.x2 - s_roi.x1 + 1;
                            const int rh = s_roi.y2 - s_roi.y1 + 1;
                            const float area_pct = (rw > 0 && rh > 0)
                                ? ((float)(rw * rh) * 100.0f / (float)(src_w * src_h))
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

            float offset_x = 0.0f, heading_deg = 0.0f, width_pct = 0.0f;
            uint8_t ix_type = UARTP_INTXN_NONE; float ix_conf = 0.0f;

            bool mask_ok = false;
            bool roi_ready = (s_roi.valid);
            if (roi_ready && mod_m2) {
                const int rw = roi_x1 - roi_x0 + 1;
                const int rh = roi_y1 - roi_y0 + 1;
                const float area_pct = (rw > 0 && rh > 0)
                    ? ((float)(rw * rh) * 100.0f / (float)(src_w * src_h))
                    : 100.0f;
                if (rw < 4 || rh < 4 || area_pct > cfg.roi_max_area_pct) {
                    roi_ready = false;
                }
            }

            if (msroi && mod_m3 && roi_ready) {
                roi_used = true;
                mask_ok = build_binary_mask_rgb_only_roi(frame, r, g, b, tol,
                                                         roi_x0, roi_y0, roi_x1, roi_y1,
                                                         s_mask);
            } else {
                if (msroi) fallback_cnt++;
                mask_ok = build_binary_mask_rgb_only_roi(frame, r, g, b, tol,
                                                         0, 0, src_w - 1, src_h - 1,
                                                         s_mask);
            }
            bool ok = false;
            if (mask_ok) {
                dilate_mask_inplace(s_mask);
                ok = estimate_line_geometry_mask(s_mask, offset_x, heading_deg, width_pct);
                detect_intersection_mask(s_mask, ix_type, ix_conf);
                overwrite_frame_with_mask_rgb565(s_mask, frame);
            } else {
                offset_x = 0.0f;
                heading_deg = 0.0f;
                width_pct = 0.0f;
                ix_type = UARTP_INTXN_NONE;
                ix_conf = 0.0f;
            }

            if (msroi) {
                if (ok) {
                    s_roi.lost = 0;
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

            if (ix_type == UARTP_INTXN_T && ix_conf > 0.50f) {
                t_seq++; c_seq = 0; t_conf_s = 0.7f * t_conf_s + 0.3f * ix_conf;
            } else if (ix_type == UARTP_INTXN_CROSS && ix_conf > 0.50f) {
                c_seq++; t_seq = 0; c_conf_s = 0.7f * c_conf_s + 0.3f * ix_conf;
            } else {
                t_seq = 0; c_seq = 0; t_conf_s = 0.7f * t_conf_s; c_conf_s = 0.7f * c_conf_s;
            }
            uint8_t ix_effective = UARTP_INTXN_NONE; float ix_conf_eff = 0.0f;
            if (c_seq >= 2) { ix_effective = UARTP_INTXN_CROSS; ix_conf_eff = c_conf_s; }
            else if (t_seq >= 2) { ix_effective = UARTP_INTXN_T; ix_conf_eff = t_conf_s; }
            else { ix_effective = UARTP_INTXN_NONE; ix_conf_eff = 0.0f; }

            ESP_LOGI(TAG, "ok=%d off=%.3f head=%.1f width=%.1f%% ix=%u conf=%.2f",
                     ok ? 1 : 0, offset_x, heading_deg, width_pct, (unsigned)ix_effective, ix_conf_eff);

            offset_x = std::clamp(offset_x, -1.0f, 1.0f);
            if (!std::isfinite(heading_deg)) heading_deg = 0.0f;
            heading_deg = std::max(-180.0f, std::min(180.0f, heading_deg));
            heading_deg = std::round(heading_deg);

            uartp_post_line_follow(ok, offset_x, heading_deg, width_pct, ix_effective, ix_conf_eff);

            int64_t t2 = esp_timer_get_time();

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
                if (ok) roi_ok_cnt++;
                float roi_area_pct = 0.0f;
                if (s_roi.valid) {
                    const int rw = (s_roi.x2 - s_roi.x1 + 1);
                    const int rh = (s_roi.y2 - s_roi.y1 + 1);
                    if (rw > 0 && rh > 0 && src_w > 0 && src_h > 0) {
                        roi_area_pct = (float)(rw * rh) * 100.0f / (float)(src_w * src_h);
                    }
                }
                roi_area_sum += roi_area_pct;
            }
            last_q_depth = s_in ? (uint32_t)uxQueueMessagesWaiting(s_in) : 0;

            if (t2 - last_report_us >= 1000000) {
                const float elapsed = (float)(t2 - last_report_us) / 1000000.0f;
                vision_metrics_t m{};
                m.task = VISION_METRICS_TASK_LINE;
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
        } while (0);

        if (s_out) {
            if (xQueueSend(s_out, &frame, 0) == pdPASS) { handed = true; frame = nullptr; }
        }
        if (!handed && frame) { esp_camera_fb_return(frame); frame = nullptr; }

        vTaskDelay(1);
    }
}

void register_line_detection(QueueHandle_t frame_in,
                             QueueHandle_t frame_out,
                             uint8_t color_id)
{
    s_in = frame_in;
    s_out = frame_out;
    if (s_line_task) {
        vTaskDelete(s_line_task);
        s_line_task = nullptr;
    }
    xTaskCreatePinnedToCore(line_follow_task, TAG, 8 * 1024,
                            (void *)(uint32_t)color_id, 5, &s_line_task, 1);
}

void unregister_line_detection(void)
{
    if (s_line_task) {
        vTaskDelete(s_line_task);
        s_line_task = nullptr;
    }
    s_in = nullptr;
    s_out = nullptr;
}
