#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "vision_metrics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     enabled;          // enable multi-scale + ROI tracking
    bool     module_coarse_mode;     // module1: coarse mode decision (skip/local/full)
    bool     module_valid_fallback;  // module2: validity check + graded fallback
    bool     module_roi_refine;      // module3: ROI crop/cache + ROI refine
    bool     dynamic_tune;     // update knobs from runtime metrics
    uint8_t  coarse_stride;    // coarse sampling step (>=1)
    uint8_t  coarse_interval;  // run coarse scan every N frames (>=1)
    uint8_t  refine_interval;  // run refine every N frames (>=1)
    uint8_t  full_refresh_interval; // force a full refresh every N frames
    uint8_t  max_reuse_frames; // max frames to reuse previous result
    uint8_t  roi_pad;          // ROI padding in pixels
    uint8_t  roi_hold_frames;  // keep ROI this many frames after miss
    uint16_t roi_min_hits;     // minimum sampled hits to accept ROI
    float    reuse_conf_hi;    // high confidence threshold for reuse/skip
    float    reuse_conf_lo;    // low confidence threshold for full scan
    float    roi_max_area_pct; // percent threshold to reject oversized ROI
} vision_msroi_config_t;

// Get current config snapshot.
void vision_msroi_get_config(vision_msroi_config_t *out_cfg);

// Enable/disable MSROI feature.
void vision_msroi_set_enabled(bool enable);

// Fast check for enabled.
bool vision_msroi_is_enabled(void);

// Control module switches.
void vision_msroi_set_modules(bool coarse_mode, bool valid_fallback, bool roi_refine);
void vision_msroi_get_modules(bool *coarse_mode, bool *valid_fallback, bool *roi_refine);

// Closed-loop tuning switch.
void vision_msroi_set_dynamic_tune(bool enable);
bool vision_msroi_dynamic_tune_is_enabled(void);

// Feed runtime metrics to update coarse/refine/ROI knobs.
void vision_msroi_feedback(const vision_metrics_t *m);

// Build readable route label for logs, e.g. "旧路线"/"开启模块1、模块3".
void vision_msroi_format_route_label(bool msroi_enabled,
                                     bool module1_enabled,
                                     bool module2_enabled,
                                     bool module3_enabled,
                                     char *out,
                                     size_t out_len);

#ifdef __cplusplus
}
#endif
