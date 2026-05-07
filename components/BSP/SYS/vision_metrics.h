#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VISION_METRICS_TASK_NONE  = 0,
    VISION_METRICS_TASK_COLOR = 1,
    VISION_METRICS_TASK_LINE  = 2,
    VISION_METRICS_TASK_AI    = 3,
    VISION_METRICS_TASK_TAG   = 4,
} vision_metrics_task_t;

typedef struct {
    vision_metrics_task_t task;
    bool     msroi_enabled;
    float    fps;
    float    coarse_ms;
    float    refine_ms;
    float    roi_hit_rate;      // percent 0..100
    float    roi_area_pct;      // percent 0..100
    float    latency_ms;       // end-to-end latency ms
    float    cpu_pct;          // task busy ratio (approx)
    uint32_t fallback_full_scan;
    uint32_t queue_depth;
    int64_t  ts_us;             // set by publish
} vision_metrics_t;

void vision_metrics_publish(const vision_metrics_t *m);
bool vision_metrics_get(vision_metrics_t *out);
bool vision_metrics_get_text(char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
