#include "vision_msroi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"
#include <stdio.h>

static portMUX_TYPE s_msroi_mux = portMUX_INITIALIZER_UNLOCKED;

static vision_msroi_config_t s_cfg = {
    .enabled         = false,
    .module_coarse_mode = true,
    .module_valid_fallback = true,
    .module_roi_refine = true,
    .dynamic_tune    = true,
    .coarse_stride   = 4,
    .coarse_interval = 2,
    .refine_interval = 2,
    .full_refresh_interval = 15,
    .max_reuse_frames = 3,
    .roi_pad         = 10,
    .roi_hold_frames = 8,
    .roi_min_hits    = 6,
    .reuse_conf_hi   = 0.78f,
    .reuse_conf_lo   = 0.38f,
    .roi_max_area_pct = 45.0f,
};

static int64_t s_last_tune_us = 0;

static inline uint8_t clamp_u8(uint8_t v, uint8_t lo, uint8_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float clamp_f32(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void vision_msroi_get_config(vision_msroi_config_t *out_cfg)
{
    if (!out_cfg) return;
    portENTER_CRITICAL(&s_msroi_mux);
    *out_cfg = s_cfg;
    portEXIT_CRITICAL(&s_msroi_mux);
}

void vision_msroi_set_enabled(bool enable)
{
    portENTER_CRITICAL(&s_msroi_mux);
    s_cfg.enabled = enable;
    portEXIT_CRITICAL(&s_msroi_mux);
}

bool vision_msroi_is_enabled(void)
{
    bool enabled;
    portENTER_CRITICAL(&s_msroi_mux);
    enabled = s_cfg.enabled;
    portEXIT_CRITICAL(&s_msroi_mux);
    return enabled;
}

void vision_msroi_set_modules(bool coarse_mode, bool valid_fallback, bool roi_refine)
{
    portENTER_CRITICAL(&s_msroi_mux);
    s_cfg.module_coarse_mode = coarse_mode;
    s_cfg.module_valid_fallback = valid_fallback;
    s_cfg.module_roi_refine = roi_refine;
    portEXIT_CRITICAL(&s_msroi_mux);
}

void vision_msroi_get_modules(bool *coarse_mode, bool *valid_fallback, bool *roi_refine)
{
    portENTER_CRITICAL(&s_msroi_mux);
    if (coarse_mode) {
        *coarse_mode = s_cfg.module_coarse_mode;
    }
    if (valid_fallback) {
        *valid_fallback = s_cfg.module_valid_fallback;
    }
    if (roi_refine) {
        *roi_refine = s_cfg.module_roi_refine;
    }
    portEXIT_CRITICAL(&s_msroi_mux);
}

void vision_msroi_set_dynamic_tune(bool enable)
{
    portENTER_CRITICAL(&s_msroi_mux);
    s_cfg.dynamic_tune = enable;
    portEXIT_CRITICAL(&s_msroi_mux);
}

bool vision_msroi_dynamic_tune_is_enabled(void)
{
    bool enabled;
    portENTER_CRITICAL(&s_msroi_mux);
    enabled = s_cfg.dynamic_tune;
    portEXIT_CRITICAL(&s_msroi_mux);
    return enabled;
}

void vision_msroi_format_route_label(bool msroi_enabled,
                                     bool module1_enabled,
                                     bool module2_enabled,
                                     bool module3_enabled,
                                     char *out,
                                     size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }

    if (!msroi_enabled) {
        snprintf(out, out_len, "分层检测关闭");
        return;
    }

    if (!module1_enabled && !module2_enabled && !module3_enabled) {
        snprintf(out, out_len, "旧路线");
        return;
    }

    int n = snprintf(out, out_len, "开启");
    if (n < 0 || (size_t)n >= out_len) {
        return;
    }
    size_t pos = (size_t)n;
    bool first = true;

    if (module1_enabled) {
        n = snprintf(out + pos, out_len - pos, "模块1");
        if (n < 0 || (size_t)n >= (out_len - pos)) {
            return;
        }
        pos += (size_t)n;
        first = false;
    }
    if (module2_enabled) {
        n = snprintf(out + pos, out_len - pos, "%s模块2", first ? "" : "、");
        if (n < 0 || (size_t)n >= (out_len - pos)) {
            return;
        }
        pos += (size_t)n;
        first = false;
    }
    if (module3_enabled) {
        (void)snprintf(out + pos, out_len - pos, "%s模块3", first ? "" : "、");
    }
}

void vision_msroi_feedback(const vision_metrics_t *m)
{
    if (!m) return;

    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_msroi_mux);

    if (!s_cfg.enabled || !s_cfg.dynamic_tune || !m->msroi_enabled) {
        portEXIT_CRITICAL(&s_msroi_mux);
        return;
    }

    // Tune at most every ~1.2s to reduce oscillation.
    if (s_last_tune_us > 0 && (now_us - s_last_tune_us) < 1200000) {
        portEXIT_CRITICAL(&s_msroi_mux);
        return;
    }
    s_last_tune_us = now_us;

    const bool pressure_high =
        (m->latency_ms > 95.0f) || (m->cpu_pct > 82.0f) || (m->queue_depth > 1);
    const bool pressure_low =
        (m->latency_ms < 55.0f) && (m->cpu_pct < 58.0f) && (m->queue_depth == 0);
    const bool roi_unstable =
        (m->roi_hit_rate < 22.0f) || (m->fallback_full_scan >= 8);
    const bool roi_too_big = (m->roi_area_pct > 58.0f);
    const bool roi_good = (m->roi_hit_rate > 60.0f) && (m->roi_area_pct > 0.0f) && (m->roi_area_pct < 36.0f);

    if (pressure_high) {
        s_cfg.coarse_stride = clamp_u8((uint8_t)(s_cfg.coarse_stride + 1), 2, 8);
        s_cfg.coarse_interval = clamp_u8((uint8_t)(s_cfg.coarse_interval + 1), 1, 6);
        s_cfg.refine_interval = clamp_u8((uint8_t)(s_cfg.refine_interval + 1), 1, 5);
        if (s_cfg.roi_pad > 2) {
            s_cfg.roi_pad = (uint8_t)(s_cfg.roi_pad - 1);
        }
        s_cfg.max_reuse_frames = clamp_u8((uint8_t)(s_cfg.max_reuse_frames + 1), 1, 8);
    } else if (pressure_low && roi_good) {
        if (s_cfg.coarse_stride > 2) {
            s_cfg.coarse_stride = (uint8_t)(s_cfg.coarse_stride - 1);
        }
        if (s_cfg.coarse_interval > 1) {
            s_cfg.coarse_interval = (uint8_t)(s_cfg.coarse_interval - 1);
        }
        if (s_cfg.refine_interval > 1) {
            s_cfg.refine_interval = (uint8_t)(s_cfg.refine_interval - 1);
        }
        s_cfg.roi_pad = clamp_u8((uint8_t)(s_cfg.roi_pad + 1), 2, 28);
        if (s_cfg.max_reuse_frames > 1) {
            s_cfg.max_reuse_frames = (uint8_t)(s_cfg.max_reuse_frames - 1);
        }
    }

    if (roi_unstable) {
        if (s_cfg.roi_min_hits > 3) {
            s_cfg.roi_min_hits = (uint16_t)(s_cfg.roi_min_hits - 1);
        }
        s_cfg.roi_hold_frames = clamp_u8((uint8_t)(s_cfg.roi_hold_frames + 1), 2, 20);
        s_cfg.roi_pad = clamp_u8((uint8_t)(s_cfg.roi_pad + 1), 2, 28);
        s_cfg.reuse_conf_hi = clamp_f32(s_cfg.reuse_conf_hi + 0.03f, 0.55f, 0.95f);
        s_cfg.reuse_conf_lo = clamp_f32(s_cfg.reuse_conf_lo + 0.02f, 0.20f, 0.85f);
    } else if (roi_good) {
        s_cfg.roi_min_hits = clamp_u16((uint16_t)(s_cfg.roi_min_hits + 1), 3, 48);
        if (s_cfg.roi_hold_frames > 4) {
            s_cfg.roi_hold_frames = (uint8_t)(s_cfg.roi_hold_frames - 1);
        }
        s_cfg.reuse_conf_hi = clamp_f32(s_cfg.reuse_conf_hi - 0.02f, 0.55f, 0.95f);
        s_cfg.reuse_conf_lo = clamp_f32(s_cfg.reuse_conf_lo - 0.02f, 0.20f, 0.85f);
    }

    if (roi_too_big) {
        s_cfg.roi_max_area_pct = clamp_f32(s_cfg.roi_max_area_pct - 2.0f, 20.0f, 80.0f);
        if (s_cfg.roi_pad > 2) {
            s_cfg.roi_pad = (uint8_t)(s_cfg.roi_pad - 1);
        }
        s_cfg.coarse_stride = clamp_u8((uint8_t)(s_cfg.coarse_stride + 1), 2, 8);
    } else if (m->roi_area_pct > 0.0f && m->roi_area_pct < 28.0f && m->roi_hit_rate > 45.0f) {
        s_cfg.roi_max_area_pct = clamp_f32(s_cfg.roi_max_area_pct + 1.0f, 20.0f, 80.0f);
    }

    s_cfg.coarse_stride = clamp_u8(s_cfg.coarse_stride, 2, 8);
    s_cfg.coarse_interval = clamp_u8(s_cfg.coarse_interval, 1, 6);
    s_cfg.refine_interval = clamp_u8(s_cfg.refine_interval, 1, 5);
    s_cfg.full_refresh_interval = clamp_u8(s_cfg.full_refresh_interval, 5, 45);
    s_cfg.max_reuse_frames = clamp_u8(s_cfg.max_reuse_frames, 1, 8);
    s_cfg.roi_pad = clamp_u8(s_cfg.roi_pad, 2, 28);
    s_cfg.roi_hold_frames = clamp_u8(s_cfg.roi_hold_frames, 2, 20);
    s_cfg.roi_min_hits = clamp_u16(s_cfg.roi_min_hits, 3, 48);
    if (s_cfg.reuse_conf_lo > s_cfg.reuse_conf_hi - 0.08f) {
        s_cfg.reuse_conf_lo = s_cfg.reuse_conf_hi - 0.08f;
    }
    s_cfg.reuse_conf_lo = clamp_f32(s_cfg.reuse_conf_lo, 0.20f, 0.85f);
    s_cfg.reuse_conf_hi = clamp_f32(s_cfg.reuse_conf_hi, 0.55f, 0.95f);
    s_cfg.roi_max_area_pct = clamp_f32(s_cfg.roi_max_area_pct, 20.0f, 80.0f);

    portEXIT_CRITICAL(&s_msroi_mux);
}
