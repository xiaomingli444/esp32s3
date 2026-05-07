#include "vision_metrics.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static portMUX_TYPE s_metrics_mux = portMUX_INITIALIZER_UNLOCKED;
static vision_metrics_t s_metrics = {0};
static char s_metrics_text[96] = {0};

static void format_metrics_text(const vision_metrics_t *m, char *out, size_t out_len)
{
    if (!m || !out || out_len == 0) return;
    const int fps_i   = (m->fps < 0.0f) ? 0 : (int)(m->fps + 0.5f);
    const int hit_i   = (m->roi_hit_rate < 0.0f) ? 0 : (int)(m->roi_hit_rate + 0.5f);
    const int area_i  = (m->roi_area_pct < 0.0f) ? 0 : (int)(m->roi_area_pct + 0.5f);
    const unsigned fb = (unsigned)m->fallback_full_scan;
    const unsigned qd = (unsigned)m->queue_depth;

    // Keep lines short for 172px width (scale=2, ~13 chars per line).
    // Line1: M? F?? C?.?
    // Line2: R?.? H?? A??
    // Line3: FB? Q?
    snprintf(out, out_len,
             "M%d F%02d C%.1f\nR%.1f H%02d A%02d\nFB%u Q%u",
             m->msroi_enabled ? 1 : 0,
             fps_i,
             (double)m->coarse_ms,
             (double)m->refine_ms,
             hit_i,
             area_i,
             fb,
             qd);
}

void vision_metrics_publish(const vision_metrics_t *m)
{
    if (!m) return;
    portENTER_CRITICAL(&s_metrics_mux);
    s_metrics = *m;
    s_metrics.ts_us = esp_timer_get_time();
    format_metrics_text(&s_metrics, s_metrics_text, sizeof(s_metrics_text));
    portEXIT_CRITICAL(&s_metrics_mux);
}

bool vision_metrics_get(vision_metrics_t *out)
{
    if (!out) return false;
    bool ok = false;
    portENTER_CRITICAL(&s_metrics_mux);
    *out = s_metrics;
    ok = (s_metrics.task != VISION_METRICS_TASK_NONE);
    portEXIT_CRITICAL(&s_metrics_mux);

    if (!ok) return false;
    const int64_t now = esp_timer_get_time();
    if (now - out->ts_us > 1500000) { // 1.5s stale
        return false;
    }
    return true;
}

bool vision_metrics_get_text(char *out, size_t out_len)
{
    if (!out || out_len == 0) return false;
    vision_metrics_t snap;
    if (!vision_metrics_get(&snap)) {
        out[0] = '\0';
        return false;
    }

    portENTER_CRITICAL(&s_metrics_mux);
    size_t len = strnlen(s_metrics_text, sizeof(s_metrics_text));
    if (len >= out_len) len = out_len - 1;
    memcpy(out, s_metrics_text, len);
    out[len] = '\0';
    portEXIT_CRITICAL(&s_metrics_mux);
    return true;
}
