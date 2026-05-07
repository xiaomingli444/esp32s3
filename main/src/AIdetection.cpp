#include "AIdetection.h"



// for vTaskDelay
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// metrics/msroi
#include "vision_msroi.h"
#include "vision_metrics.h"
#include "esp_timer.h"
// for file existence & directory listing on ESP-IDF
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <cctype>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include "esp_heap_caps.h"
#include "uartp.h"

static const char* TAG = "AIdetection";

// 全局/静态资源（简单起见，做成单例风格）
static dl::Model*                            g_model = nullptr;
static dl::TensorBase*                       g_input = nullptr;
static dl::image::ImagePreprocessor*         g_pre   = nullptr;
static dl::detect::ESPDetPostProcessor*      g_post  = nullptr;
// 推理任务句柄（避免在 main 任务里长时间阻塞，触发 WDT）
static TaskHandle_t                          g_y11_task = nullptr;
static TaskHandle_t                          g_ai_task  = nullptr;
static SemaphoreHandle_t                     g_ai_exit_sem = nullptr;
static std::string                           g_model_path;
static float                                 g_conf_thresh = 0.0f;
static volatile bool                         g_ai_stop_requested = false;

// 队列句柄（由 register_AIdetection 提供）
static QueueHandle_t s_in = nullptr;
static QueueHandle_t s_out = nullptr;

// 模型输入尺寸可由 g_input->shape 查询，这里不再保留冗余的固定网格尺寸变量

typedef struct {
    int x1, y1, x2, y2;
    uint8_t lost;
    bool valid;
} roi_state_t;

static const uint8_t AI_ROI_STABLE_MIN = 1;
static const int     AI_ROI_MIN_SIDE = 8;
static const int64_t AI_REUSE_US_PER_FRAME = 200000; // ~5fps worth of reuse window

static uint8_t   s_roi_stable = 0;
static y11_box_t s_last_boxes[20] = {};
static int       s_last_num = 0;
static int64_t   s_last_det_us = 0;

static roi_state_t s_roi = {0};
static uint8_t *s_roi_buf = nullptr;
static size_t s_roi_buf_cap = 0;

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

static inline int clamp_i32(int v, int lo, int hi)
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

static inline float roi_area_pct(const roi_state_t *r, int w, int h)
{
    if (!r || !r->valid || w <= 0 || h <= 0) return 0.0f;
    const int rw = r->x2 - r->x1 + 1;
    const int rh = r->y2 - r->y1 + 1;
    if (rw <= 0 || rh <= 0) return 0.0f;
    return (float)(rw * rh) * 100.0f / (float)(w * h);
}

static inline float roi_iou(const roi_state_t *a, const roi_state_t *b)
{
    if (!a || !b || !a->valid || !b->valid) return 0.0f;
    const int ix1 = std::max(a->x1, b->x1);
    const int iy1 = std::max(a->y1, b->y1);
    const int ix2 = std::min(a->x2, b->x2);
    const int iy2 = std::min(a->y2, b->y2);
    if (ix1 > ix2 || iy1 > iy2) return 0.0f;
    const int iw = ix2 - ix1 + 1;
    const int ih = iy2 - iy1 + 1;
    const float inter = (float)(iw * ih);
    const float area_a = (float)((a->x2 - a->x1 + 1) * (a->y2 - a->y1 + 1));
    const float area_b = (float)((b->x2 - b->x1 + 1) * (b->y2 - b->y1 + 1));
    const float uni = area_a + area_b - inter;
    if (uni <= 1e-6f) return 0.0f;
    return inter / uni;
}

typedef enum {
    AI_COARSE_SKIP = 0,
    AI_COARSE_LOCAL = 1,
    AI_COARSE_FULL = 2,
} ai_coarse_mode_t;

static inline float ai_prev_det_conf(void)
{
    if (s_last_num <= 0) return 0.0f;
    float best = 0.0f;
    for (int i = 0; i < s_last_num; ++i) {
        if (s_last_boxes[i].score > best) best = s_last_boxes[i].score;
    }
    return clamp_f32(best, 0.0f, 1.0f);
}

static inline float ai_prior_reliability(float prev_conf, float track_iou, uint8_t stable_hits, uint8_t lost_cnt)
{
    const float hit_term = clamp_f32((float)stable_hits / 6.0f, 0.0f, 1.0f);
    const float lost_term = clamp_f32((float)lost_cnt / 5.0f, 0.0f, 1.0f);
    float rel = 0.46f * prev_conf + 0.28f * clamp_f32(track_iou, 0.0f, 1.0f) +
                0.26f * hit_term - 0.28f * lost_term;
    return clamp_f32(rel, 0.0f, 1.0f);
}

static inline uint8_t up5_to_8(uint8_t v5) { return (uint8_t)((v5 * 255 + 15) / 31); }
static inline uint8_t up6_to_8(uint8_t v6) { return (uint8_t)((v6 * 255 + 31) / 63); }

static inline uint8_t rgb565_to_luma(uint8_t hi, uint8_t lo)
{
    uint16_t pix = ((uint16_t)hi << 8) | lo;
    uint8_t r5 = (pix >> 11) & 0x1F;
    uint8_t g6 = (pix >> 5) & 0x3F;
    uint8_t b5 = pix & 0x1F;
    uint8_t r = up5_to_8(r5);
    uint8_t g = up6_to_8(g6);
    uint8_t b = up5_to_8(b5);
    return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

// coarse scan: sparse edge energy on RGB565 to propose ROI
static bool coarse_scan_edge_rgb565_region(const camera_fb_t *fb,
                                    int step, int edge_th,
                                    int rx0, int ry0, int rx1, int ry1,
                                    int &x1, int &y1, int &x2, int &y2, uint32_t &hits)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) return false;
    const int W = fb->width;
    const int H = fb->height;
    if (step < 1) step = 1;
    rx0 = clamp_i32(rx0, 0, W - 1);
    ry0 = clamp_i32(ry0, 0, H - 1);
    rx1 = clamp_i32(rx1, 0, W - 1);
    ry1 = clamp_i32(ry1, 0, H - 1);
    if (rx0 > rx1 || ry0 > ry1) return false;

    const uint8_t *base = (const uint8_t *)fb->buf;
    int minx = W, miny = H, maxx = -1, maxy = -1;
    uint32_t cnt = 0;

    for (int y = ry0; y < ry1; y += step) {
        const uint8_t *row = base + (size_t)y * W * 2;
        const uint8_t *row_down = base + (size_t)(y + 1) * W * 2;
        for (int x = rx0; x < rx1; x += step) {
            uint8_t l0 = rgb565_to_luma(row[2 * x], row[2 * x + 1]);
            uint8_t l1 = rgb565_to_luma(row[2 * (x + 1)], row[2 * (x + 1) + 1]);
            uint8_t l2 = rgb565_to_luma(row_down[2 * x], row_down[2 * x + 1]);
            int diff = std::abs((int)l0 - (int)l1) + std::abs((int)l0 - (int)l2);
            if (diff >= edge_th) {
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

static bool ensure_roi_buf(size_t bytes)
{
    if (s_roi_buf && s_roi_buf_cap >= bytes) return true;
    if (s_roi_buf) {
        heap_caps_free(s_roi_buf);
        s_roi_buf = nullptr;
        s_roi_buf_cap = 0;
    }
    s_roi_buf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
    if (!s_roi_buf) {
        return false;
    }
    s_roi_buf_cap = bytes;
    return true;
}

static void free_roi_buf(void)
{
    if (s_roi_buf) {
        heap_caps_free(s_roi_buf);
        s_roi_buf = nullptr;
    }
    s_roi_buf_cap = 0;
}

static bool copy_roi_rgb565(const camera_fb_t *fb,
                            int x0, int y0, int x1, int y1,
                            uint8_t **out_buf, int *out_w, int *out_h)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) return false;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= (int)fb->width) x1 = fb->width - 1;
    if (y1 >= (int)fb->height) y1 = fb->height - 1;
    if (x0 > x1 || y0 > y1) return false;

    const int w = x1 - x0 + 1;
    const int h = y1 - y0 + 1;
    const size_t bytes = (size_t)w * (size_t)h * 2;
    if (!ensure_roi_buf(bytes)) return false;

    const uint8_t *base = (const uint8_t *)fb->buf;
    uint8_t *dst = s_roi_buf;
    for (int y = y0; y <= y1; ++y) {
        const uint8_t *row = base + (size_t)y * fb->width * 2 + (size_t)x0 * 2;
        memcpy(dst, row, (size_t)w * 2);
        dst += (size_t)w * 2;
    }
    if (out_buf) *out_buf = s_roi_buf;
    if (out_w) *out_w = w;
    if (out_h) *out_h = h;
    return true;
}

// 辅助：检查文件是否存在
static bool file_exists(const char* path) {
    struct stat st;
    return (path && stat(path, &st) == 0 && (st.st_mode & S_IFREG));
}

// 辅助：打印目录内容，便于调试 SD 卡路径问题
static void log_dir(const char* dir) {
    DIR* d = opendir(dir);
    if (!d) {
        ESP_LOGW(TAG, "opendir(%s) fail", dir);
        return;
    }
    ESP_LOGI(TAG, "list %s:", dir);
    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        ESP_LOGI(TAG, " - %s", e->d_name);
    }
    closedir(d);
}

// 大小写不敏感比较后缀
static bool ends_with_casei(const std::string &name, const char *sfx)
{
    size_t n = name.size();
    size_t m = strlen(sfx);
    if (m > n) return false;
    for (size_t i = 0; i < m; ++i) {
        char a = (char)std::toupper((unsigned char)name[n - m + i]);
        char b = (char)std::toupper((unsigned char)sfx[i]);
        if (a != b) return false;
    }
    return true;
}

// 从路径提取目录部分（不含最后一个'/'之后的文件名）
static std::string get_dir_part(const char *path)
{
    if (!path) return std::string();
    const char *slash = strrchr(path, '/');
    if (!slash) return std::string();
    return std::string(path, slash - path);
}

// 兼容 FAT 无 LFN 情况：
// - 优先使用原路径
// - 若扩展名为 .espdl，尝试 .esp（8.3）
// - 扫描目录，选取首个 .ESPDL 或 .ESP 文件
static bool resolve_model_path(const char *preferred, std::string &out_full)
{
    if (preferred && file_exists(preferred)) {
        out_full = preferred;
        return true;
    }
    if (preferred) {
        std::string p(preferred);
        // 替换 .espdl -> .esp（大小写不敏感）
        if (ends_with_casei(p, ".espdl")) {
            p.resize(p.size() - 6);
            p += ".esp";
            if (file_exists(p.c_str())) {
                out_full = p;
                return true;
            }
        }
        // 扫描所在目录
        std::string dir = get_dir_part(preferred);
        if (!dir.empty()) {
            DIR *d = opendir(dir.c_str());
            if (d) {
                struct dirent *e;
                while ((e = readdir(d)) != nullptr) {
                    std::string name = e->d_name;
                    if (ends_with_casei(name, ".espdl") || ends_with_casei(name, ".esp")) {
                        std::string full = dir + "/" + name;
                        if (file_exists(full.c_str())) {
                            closedir(d);
                            out_full = full;
                            return true;
                        }
                    }
                }
                closedir(d);
            }
        }
    }
    return false;
}

int y11_init_from_sd(const char* espdl_path, float conf, float iou)
{
    if (g_model) return ESP_OK;                // 已初始化
    if (!espdl_path) return ESP_ERR_INVALID_ARG;

    // 0) 解析实际可用的模型路径（兼容 FATFS 无长文件名导致的 8.3 别名）
    std::string real_path;
    if (!resolve_model_path(espdl_path, real_path)) {
        ESP_LOGE(TAG, "Model file not found: %s", espdl_path);
        log_dir("/sdcard");
        log_dir("/sdcard/models");
        log_dir("/sdcard/model");
        return ESP_FAIL;
    }

    // 1) 从 SD 卡加载 .espdl/.esp (8.3) 模型
    // 为避免可用内部块在运行中波动导致 root_alloc 失败，这里将 internal 预算固定为 0（完全不强求内部 RAM）
    // 如需提速，可改回动态预算，但务必确保分配成功后再继续创建预处理器
    int max_internal = 0;
    size_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "max_internal picked: %d KB (largest=%d KB)", max_internal / 1024, (int)(largest_blk / 1024));

    g_model = new dl::Model(real_path.c_str(),
                            fbs::MODEL_LOCATION_IN_SDCARD,
                            max_internal,
                            dl::MEMORY_MANAGER_GREEDY,
                            nullptr,
                            true /*param_copy*/);
    if (!g_model) {
        ESP_LOGE(TAG, "new Model failed");
        return ESP_FAIL;
    }
    g_model->minimize();
    if (!g_model->get_output("box0") || !g_model->get_output("score0") ||
        !g_model->get_output("box1") || !g_model->get_output("score1") ||
        !g_model->get_output("box2") || !g_model->get_output("score2")) {
        ESP_LOGE(TAG, "model outputs mismatch, need: box0/score0/box1/score1/box2/score2");
        y11_deinit();
        return ESP_FAIL;
    }

    // 2) 获取输入张量：优先检查 inputs 容器，避免直接调用 get_input 触发 size==1 的断言
    auto &inputs = g_model->get_inputs();
    if (inputs.empty()) {
        ESP_LOGE(TAG, "model has no input tensor (load may have failed)");
        y11_deinit();
        return ESP_FAIL;
    }
    if (inputs.size() == 1) {
        g_input = g_model->get_input(); // 此时 size==1，满足断言
    } else {
        // 多输入模型：先拿第一个作为图像输入（如需要可按名称筛选 'input'/'images'）
        g_input = inputs.begin()->second;
    }

    // 3) 预处理：使用 esp-dl 的 ImagePreprocessor（依据模型输入形状自动设置 dst 尺寸与量化）
    //    归一化采用 [0,255] -> [0,1]，并启用 letterbox(114)
    std::vector<float> mean(3, 0.0f);
    std::vector<float> stdv(3, 255.0f);
    // 如果摄像头帧缓冲是标准 RGB565（大端），可以避免颜色交换以减少预处理成本
    // 如你的摄像头数据需要通道调换/字节调换，可按需 OR 上 DL_IMAGE_CAP_RGB_SWAP / DL_IMAGE_CAP_RGB565_BYTE_SWAP
    uint32_t caps = dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN;
    g_pre = new dl::image::ImagePreprocessor(g_model, mean, stdv, caps);
    if (!g_pre) {
        ESP_LOGE(TAG, "new ImagePreprocessor failed");
        y11_deinit();
        return ESP_FAIL;
    }
    g_pre->enable_letterbox({114, 114, 114});

    // 4) ESPDet 后处理（AnchorPoint 风格，需要提供三个尺度的 stride/offset 配置）
    std::vector<dl::detect::anchor_point_stage_t> stages = {
        { .stride_y = 8,  .stride_x = 8,  .offset_y = 4,  .offset_x = 4  },
        { .stride_y = 16, .stride_x = 16, .offset_y = 8,  .offset_x = 8  },
        { .stride_y = 32, .stride_x = 32, .offset_y = 16, .offset_x = 16 },
    };
    float conf_thr = (conf > 0.f) ? conf : 0.25f;
    float iou_thr  = (iou  > 0.f) ? iou  : 0.70f;
    int   top_k    = 10;  // 减少候选框数量，降低 NMS/排序开销
    g_post = new dl::detect::ESPDetPostProcessor(
        g_model,
        g_pre,
        conf_thr,
        iou_thr,
        top_k,
        stages
    );
    if (!g_post) {
        ESP_LOGE(TAG, "new ESPDetPostProcessor failed");
        y11_deinit();
        return ESP_FAIL;
    }

    // 记录模型输入尺寸（便于调试）
    int in_h = g_input->shape[1];
    int in_w = g_input->shape[2];
    ESP_LOGI(TAG, "ESPDet init ok: %s (input=%dx%d, conf=%.2f, iou=%.2f)", real_path.c_str(), in_w, in_h, conf_thr, iou_thr);
    return ESP_OK;
}

int y11_detect_rgb565(const uint8_t* rgb565, int w, int h, int stride_bytes,
                      y11_box_t* out, int max_out)
{
    if (!g_model || !g_input || !g_pre || !g_post) 
    {
        return -ESP_ERR_INVALID_STATE;
    }
    if (!rgb565 || w <= 0 || h <= 0 || !out || max_out <= 0)
    {
        return -ESP_ERR_INVALID_ARG;
    }

    (void)stride_bytes;

    // A) 打包成 esp-dl 的图像描述（注意：该 img_t 结构无 stride 字段）
    dl::image::img_t src{};
    src.data     = const_cast<uint8_t*>(rgb565);
    src.width    = (uint16_t)w;
    src.height   = (uint16_t)h;
    src.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

    // B) 预处理 -> 写入模型输入
    g_pre->preprocess(src);

    // C) 推理
    // 启用自动运行模式，允许算子根据任务划分在双核并行
    //g_model->run(dl::RUNTIME_MODE_AUTO);
    g_model->run(dl::RUNTIME_MODE_SINGLE_CORE);

    // D) 后处理（坐标映射到原图尺寸）
    g_post->clear_result();
    g_post->postprocess();
    auto &list = g_post->get_result(w, h);

    // E) 拷贝到用户缓冲（最多 max_out 个）
    int n = 0;
    for (const auto &o : list) {
        if (n >= max_out) break;
        y11_box_t b;
        b.cls   = o.category;
        b.score = o.score;
        // o.box 是 std::vector<int>，顺序: [x1, y1, x2, y2]
        if (o.box.size() >= 4) {
            b.x1 = o.box[0]; b.y1 = o.box[1];
            b.x2 = o.box[2]; b.y2 = o.box[3];
        } else {
            b.x1 = b.y1 = b.x2 = b.y2 = 0;
        }
        out[n++] = b;
    }
    return n; // 返回写入的框数
}

void y11_deinit(void)
{
    if (g_post)  { delete g_post;  g_post  = nullptr; }
    if (g_pre)   { delete g_pre;   g_pre   = nullptr; }
    if (g_model) { delete g_model; g_model = nullptr; }
    g_input = nullptr;
    roi_reset(&s_roi);
    s_roi_stable = 0;
    s_last_num = 0;
    s_last_det_us = 0;
    free_roi_buf();
}

void y11_test(void)
{
    // 若任务已在运行，直接返回
    if (g_y11_task) {
        ESP_LOGI(TAG, "y11_test: task already running");
        return;
    }

    // 在 APP CPU(核心1) 启动独立任务，避免占用 PRO CPU 的 IDLE0
    auto y11_task = [](void*) {
        // 1) 确保模型已加载（如已加载则直接返回 OK）
    const char *model_path = "/sdcard/models/1.espdl";
    // 提高置信度阈值可显著减少候选框，降低后处理开销
    esp_err_t err = y11_init_from_sd(model_path, 0.40f, 0.70f);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "y11_task: init model failed (%d)", (int)err);
            vTaskDelete(nullptr);
            return;
        }

        // 2) 循环检测
    // 缩短 delay，避免人为限速；实际 FPS 由单次推理时长决定
    const TickType_t interval = pdMS_TO_TICKS(5);
        y11_box_t boxes[20];
        for (;;) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "y11_task: esp_camera_fb_get() failed");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            int stride_bytes = (int)fb->width * 2; // RGB565 每像素 2 字节
            int num = y11_detect_rgb565(fb->buf,
                                        (int)fb->width,
                                        (int)fb->height,
                                        stride_bytes,
                                        boxes,
                                        (int)(sizeof(boxes) / sizeof(boxes[0])));

            esp_camera_fb_return(fb);

            if (num < 0) {
                ESP_LOGE(TAG, "y11_task: detect failed (%d)", num);
                vTaskDelay(interval);
                continue;
            }

            ESP_LOGI(TAG, "y11_test: detected %d objects", num);
            for (int i = 0; i < num; ++i) {
                ESP_LOGI(TAG, " #%02d  cls=%d  score=%.3f  box=[%d,%d,%d,%d]",
                         i, boxes[i].cls, boxes[i].score, boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2);
            }

            // 关键：让出 CPU，确保 Idle 任务能喂狗
            vTaskDelay(interval);
        }
    };

    BaseType_t ok = xTaskCreatePinnedToCore(
        y11_task,
        "y11_task",
        12288,           // 栈大小（字节数与平台相关；此处 12KB 经验值）
        nullptr,
        4,               // 适中优先级，低于系统/驱动关键任务
        &g_y11_task,
        1                // 绑定到 APP CPU (核心1)
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "y11_test: create task failed (%ld)", (long)ok);
        g_y11_task = nullptr;
    } else {
        ESP_LOGI(TAG, "y11_test: task started on core 1");
    }
}

// ====================== 在线检测任务（对接 main 的队列） ======================
static void ai_detect_task(void *arg)
{
    (void)arg;
    if (g_ai_stop_requested) {
        if (g_ai_exit_sem) {
            xSemaphoreGive(g_ai_exit_sem);
        }
        g_ai_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    // 1) 确保模型已加载一次
    if (!g_model) {
        if (g_model_path.empty()) {
            ESP_LOGE(TAG, "ai_detect_task: no model path provided");
            if (g_ai_exit_sem) {
                xSemaphoreGive(g_ai_exit_sem);
            }
            g_ai_task = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        if (g_conf_thresh <= 0.0f || g_conf_thresh > 1.0f) {
            ESP_LOGE(TAG, "ai_detect_task: invalid score %.2f", (double)g_conf_thresh);
            if (g_ai_exit_sem) {
                xSemaphoreGive(g_ai_exit_sem);
            }
            g_ai_task = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        esp_err_t err = y11_init_from_sd(g_model_path.c_str(), g_conf_thresh, 0.70f);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ai_detect_task: init model failed (%d)", (int)err);
            if (g_ai_exit_sem) {
                xSemaphoreGive(g_ai_exit_sem);
            }
            g_ai_task = nullptr;
            vTaskDelete(nullptr);
            return;
        }
    }

    camera_fb_t *frame = nullptr;
    y11_box_t boxes[16];

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
    uint32_t seq = 0;

    roi_reset(&s_roi);
    s_roi_stable = 0;
    s_last_num = 0;
    s_last_det_us = 0;
    while (!g_ai_stop_requested) {
        if (!s_in) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xQueueReceive(s_in, &frame, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;
        }
        if (!frame) continue;

        bool handed = false;
        int src_w = (int)frame->width;
        int src_h = (int)frame->height;
        do {
            if (frame->format != PIXFORMAT_RGB565) {
                ESP_LOGW(TAG, "AI needs RGB565, got %d", frame->format);
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
                if (cfg.refine_interval < 1) cfg.refine_interval = 1;
                if (cfg.full_refresh_interval < 1) cfg.full_refresh_interval = 1;
                if (cfg.max_reuse_frames < 1) cfg.max_reuse_frames = 1;
                if (cfg.roi_max_area_pct <= 0.0f) cfg.roi_max_area_pct = 45.0f;
                if (cfg.reuse_conf_hi <= 0.0f) cfg.reuse_conf_hi = 0.78f;
                if (cfg.reuse_conf_lo <= 0.0f) cfg.reuse_conf_lo = 0.38f;
                if (cfg.reuse_conf_lo > cfg.reuse_conf_hi - 0.08f) {
                    cfg.reuse_conf_lo = cfg.reuse_conf_hi - 0.08f;
                }
            }
            const bool mod_m1 = (msroi && cfg.module_coarse_mode);
            const bool mod_m2 = (msroi && cfg.module_valid_fallback);
            const bool mod_m3 = (msroi && cfg.module_roi_refine);

            int roi_x0 = 0, roi_y0 = 0, roi_x1 = src_w - 1, roi_y1 = src_h - 1;
            bool roi_used = false;
            bool coarse_ran = false;
            int fallback_level = 0;
            float prior_rel = 0.0f;
            float coarse_conf = 0.0f;

            int64_t t0 = esp_timer_get_time();
            if (msroi && src_w > 0 && src_h > 0) {
                seq++;
                const float prev_conf = ai_prev_det_conf();
                prior_rel = ai_prior_reliability(prev_conf,
                                                 s_roi.valid ? 1.0f : 0.0f,
                                                 s_roi_stable,
                                                 s_roi.lost);

                ai_coarse_mode_t coarse_mode = AI_COARSE_FULL;
                if (mod_m1) {
                    const bool force_full = (cfg.full_refresh_interval > 0) &&
                                            ((seq % cfg.full_refresh_interval) == 0);
                    if (!force_full && s_roi.valid &&
                        s_roi_stable >= AI_ROI_STABLE_MIN &&
                        prior_rel >= cfg.reuse_conf_hi) {
                        coarse_mode = AI_COARSE_SKIP;
                    } else if (s_roi.valid && prior_rel >= cfg.reuse_conf_lo) {
                        coarse_mode = AI_COARSE_LOCAL;
                    }
                }

                const bool need_coarse_by_period =
                    (!s_roi.valid) || (cfg.coarse_interval <= 1) || ((seq % cfg.coarse_interval) == 0);
                const bool run_coarse = (!mod_m1) ? true : (coarse_mode != AI_COARSE_SKIP && need_coarse_by_period);
                roi_state_t coarse_roi = {0};
                uint32_t coarse_hits = 0;
                bool coarse_ok = false;

                if (run_coarse) {
                    coarse_ran = true;
                    int cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0;
                    const int edge_th = 28; // coarse edge threshold
                    int sx0 = 0, sy0 = 0, sx1 = src_w - 1, sy1 = src_h - 1;
                    if (coarse_mode == AI_COARSE_LOCAL && s_roi.valid) {
                        const int expand = std::max(6, (int)cfg.roi_pad * 2);
                        sx0 = s_roi.x1 - expand;
                        sy0 = s_roi.y1 - expand;
                        sx1 = s_roi.x2 + expand;
                        sy1 = s_roi.y2 + expand;
                    }
                    coarse_ok = coarse_scan_edge_rgb565_region(frame,
                                                               cfg.coarse_stride,
                                                               edge_th,
                                                               sx0, sy0, sx1, sy1,
                                                               cx1, cy1, cx2, cy2, coarse_hits);
                    if (coarse_ok && coarse_hits >= cfg.roi_min_hits) {
                        coarse_roi.x1 = cx1; coarse_roi.y1 = cy1;
                        coarse_roi.x2 = cx2; coarse_roi.y2 = cy2;
                        coarse_roi.valid = true;
                        coarse_roi.lost = 0;
                        coarse_conf = clamp_f32((float)coarse_hits / (float)(cfg.roi_min_hits * 3), 0.0f, 1.0f);
                    }
                }

                roi_state_t pred_roi = s_roi;
                if (pred_roi.valid && mod_m1) {
                    const int pred_pad = std::max(2, (int)cfg.roi_pad / 2);
                    roi_apply_pad(&pred_roi, pred_pad, src_w, src_h);
                }

                roi_state_t selected = {0};
                if (mod_m1) {
                    float best_score = -1.0f;
                    if (pred_roi.valid) {
                        float score = 0.58f * prior_rel +
                                      0.22f * clamp_f32((float)s_roi_stable / 6.0f, 0.0f, 1.0f) -
                                      0.20f * clamp_f32(roi_area_pct(&pred_roi, src_w, src_h) / 100.0f, 0.0f, 1.0f);
                        selected = pred_roi;
                        best_score = score;
                    }
                    if (coarse_roi.valid) {
                        float iou_pc = pred_roi.valid ? roi_iou(&pred_roi, &coarse_roi) : 0.0f;
                        float score = 0.44f * coarse_conf +
                                      0.24f * iou_pc +
                                      0.20f * clamp_f32((float)s_roi_stable / 6.0f, 0.0f, 1.0f) -
                                      0.18f * clamp_f32(roi_area_pct(&coarse_roi, src_w, src_h) / 100.0f, 0.0f, 1.0f);
                        if (score > best_score) {
                            selected = coarse_roi;
                            best_score = score;
                        }
                    }
                } else if (coarse_roi.valid) {
                    selected = coarse_roi;
                }

                if (selected.valid) {
                    int bw = selected.x2 - selected.x1 + 1;
                    int bh = selected.y2 - selected.y1 + 1;
                    int min_side = (bw < bh) ? bw : bh;
                    int dyn_pad = cfg.roi_pad;
                    if (min_side > 0) {
                        int cap = min_side / 6;
                        if (cap < 2) cap = 2;
                        if (dyn_pad > cap) dyn_pad = cap;
                    }
                    roi_apply_pad(&selected, dyn_pad, src_w, src_h);
                    const float area_pct = roi_area_pct(&selected, src_w, src_h);
                    if (selected.valid && area_pct > cfg.roi_max_area_pct) {
                        selected.valid = false;
                    }
                }

                if (selected.valid) {
                    s_roi = selected;
                    s_roi.lost = 0;
                } else {
                    if (s_roi.valid) s_roi.lost++;
                    if (s_roi.lost > cfg.roi_hold_frames) {
                        roi_reset(&s_roi);
                    }
                }
                if (s_roi.valid) {
                    roi_x0 = s_roi.x1; roi_y0 = s_roi.y1;
                    roi_x1 = s_roi.x2; roi_y1 = s_roi.y2;
                }
            }
            int64_t t1 = esp_timer_get_time();

            float roi_area_pct_now = 100.0f;
            if (msroi && s_roi.valid && src_w > 0 && src_h > 0) {
                roi_area_pct_now = roi_area_pct(&s_roi, src_w, src_h);
            }
            const bool roi_small = (roi_area_pct_now > 0.0f && roi_area_pct_now <= cfg.roi_max_area_pct);
            const bool force_full = (msroi && cfg.full_refresh_interval > 0 && ((seq % cfg.full_refresh_interval) == 0));
            const bool roi_stable = (s_roi_stable >= AI_ROI_STABLE_MIN);
            bool allow_roi = (msroi && mod_m3 && s_roi.valid && roi_small && roi_stable && !force_full);

            if (allow_roi && mod_m2) {
                const int rw = roi_x1 - roi_x0 + 1;
                const int rh = roi_y1 - roi_y0 + 1;
                const bool geom_ok = (rw >= AI_ROI_MIN_SIDE && rh >= AI_ROI_MIN_SIDE);
                const bool score_ok = (!mod_m1) || (prior_rel >= (cfg.reuse_conf_lo * 0.9f));
                const bool budget_ok = (roi_area_pct_now > 0.0f && roi_area_pct_now <= cfg.roi_max_area_pct);
                if (!(geom_ok && score_ok && budget_ok)) {
                    if (!geom_ok) fallback_level = 3;
                    else if (!score_ok) fallback_level = 2;
                    else fallback_level = 1;
                    allow_roi = false;
                }
            }

            int num = -1;
            const uint8_t *det_buf = frame->buf;
            int det_w = src_w;
            int det_h = src_h;
            if (allow_roi) {
                roi_used = true;
                uint8_t *roi_buf = nullptr;
                if (copy_roi_rgb565(frame, roi_x0, roi_y0, roi_x1, roi_y1, &roi_buf, &det_w, &det_h)) {
                    det_buf = roi_buf;
                } else {
                    roi_used = false;
                    if (msroi) fallback_cnt++;
                }
            } else {
                if (msroi) fallback_cnt++;
                if (mod_m2 && fallback_level == 1 && s_roi.valid) {
                    int pad = cfg.roi_pad + 2;
                    roi_apply_pad(&s_roi, pad, src_w, src_h);
                    roi_x0 = s_roi.x1; roi_y0 = s_roi.y1;
                    roi_x1 = s_roi.x2; roi_y1 = s_roi.y2;
                }
            }

            bool run_refine = true;
            if (allow_roi && cfg.refine_interval > 1) {
                if ((seq % cfg.refine_interval) != 0) {
                    run_refine = false;
                }
            }

            const int64_t reuse_check_us = esp_timer_get_time();
            if (!run_refine) {
                int64_t reuse_max_us = (int64_t)cfg.max_reuse_frames * AI_REUSE_US_PER_FRAME;
                if (!mod_m1) {
                    reuse_max_us = 0;
                }
                const bool can_reuse = (s_last_num > 0) &&
                                       (reuse_max_us > 0) &&
                                       (reuse_check_us - s_last_det_us <= reuse_max_us) &&
                                       (prior_rel >= cfg.reuse_conf_hi);
                if (can_reuse) {
                    num = s_last_num;
                    if (num > 0) {
                        memcpy(boxes, s_last_boxes, sizeof(y11_box_t) * (size_t)num);
                    }
                } else {
                    run_refine = true;
                }
            }

            if (run_refine) {
                num = y11_detect_rgb565(det_buf, det_w, det_h,
                                        det_w * 2, boxes, (int)(sizeof(boxes)/sizeof(boxes[0])));

                if (num >= 0 && (msroi && roi_used)) {
                    for (int i = 0; i < num; ++i) {
                        boxes[i].x1 += roi_x0;
                        boxes[i].y1 += roi_y0;
                        boxes[i].x2 += roi_x0;
                        boxes[i].y2 += roi_y0;
                    }
                }

                if (num >= 0) {
                    s_last_num = num;
                    if (num > 0) {
                        memcpy(s_last_boxes, boxes, sizeof(y11_box_t) * (size_t)num);
                    }
                    s_last_det_us = reuse_check_us;
                }
            }
            if (msroi) {
                if (num > 0) {
                    int bx1 = src_w, by1 = src_h, bx2 = -1, by2 = -1;
                    for (int i = 0; i < num; ++i) {
                        const y11_box_t &b = boxes[i];
                        int x1 = std::max(0, std::min(b.x1, src_w - 1));
                        int y1 = std::max(0, std::min(b.y1, src_h - 1));
                        int x2 = std::max(0, std::min(b.x2, src_w - 1));
                        int y2 = std::max(0, std::min(b.y2, src_h - 1));
                        if (x1 < bx1) bx1 = x1;
                        if (y1 < by1) by1 = y1;
                        if (x2 > bx2) bx2 = x2;
                        if (y2 > by2) by2 = y2;
                    }
                    if (bx1 <= bx2 && by1 <= by2) {
                        s_roi.x1 = bx1; s_roi.y1 = by1; s_roi.x2 = bx2; s_roi.y2 = by2;
                        s_roi.valid = true;
                        s_roi.lost = 0;
                        int bw = bx2 - bx1 + 1;
                        int bh = by2 - by1 + 1;
                        int min_side = (bw < bh) ? bw : bh;
                        int dyn_pad = cfg.roi_pad;
                        if (min_side > 0) {
                            int cap = min_side / 6;
                            if (cap < 2) cap = 2;
                            if (dyn_pad > cap) dyn_pad = cap;
                        }
                        roi_apply_pad(&s_roi, dyn_pad, src_w, src_h);
                        if (s_roi.valid && roi_area_pct(&s_roi, src_w, src_h) > cfg.roi_max_area_pct) {
                            s_roi.valid = false;
                            s_roi.lost = 0;
                        }
                    }
                    if (s_roi.valid) {
                        if (s_roi_stable < 255) s_roi_stable++;
                    } else {
                        s_roi_stable = 0;
                    }
                } else if (s_roi.valid) {
                    s_roi.lost++;
                    if (s_roi.lost > cfg.roi_hold_frames) {
                        roi_reset(&s_roi);
                    }
                    s_roi_stable = 0;
                } else {
                    s_roi_stable = 0;
                }
            }

            if (num < 0) {
                ESP_LOGW(TAG, "detect failed %d", num);
                uartp_post_ai_detect(0, 0, nullptr);
            } else {
                ESP_LOGI(TAG, "num_dets=%d", num);
                for (int i = 0; i < num; ++i) {
                    const y11_box_t &b = boxes[i];
                    ESP_LOGI(TAG, " det#%02d class_id=%d score=%.2f bbox=[%d,%d,%d,%d]",
                             i, b.cls, b.score, b.x1, b.y1, b.x2, b.y2);
                }
                if (num > 0) {
                    uartp_d5h_det_t dets[UARTP_D5H_MAX_DETS];
                    int out_n = num;
                    if (out_n > UARTP_D5H_MAX_DETS) out_n = UARTP_D5H_MAX_DETS;
                    int max_x = (src_w > 0) ? (src_w - 1) : 0;
                    int max_y = (src_h > 0) ? (src_h - 1) : 0;
                    auto clamp_u16 = [](int v, int lo, int hi) -> uint16_t {
                        if (v < lo) v = lo;
                        if (v > hi) v = hi;
                        return (uint16_t)v;
                    };
                    for (int i = 0; i < out_n; ++i) {
                        const y11_box_t &b = boxes[i];
                        dets[i].class_id = (uint8_t)b.cls;
                        dets[i].score = b.score;
                        dets[i].x1 = clamp_u16(b.x1, 0, max_x);
                        dets[i].y1 = clamp_u16(b.y1, 0, max_y);
                        dets[i].x2 = clamp_u16(b.x2, 0, max_x);
                        dets[i].y2 = clamp_u16(b.y2, 0, max_y);
                    }
                    uartp_post_ai_detect(1, (uint8_t)out_n, dets);
                } else {
                    uartp_post_ai_detect(0, 0, nullptr);
                }
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
                if (num > 0) roi_ok_cnt++;
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
            if (num > 0) det_ok_cnt++;
            last_q_depth = s_in ? (uint32_t)uxQueueMessagesWaiting(s_in) : 0;

            if (t2 - last_report_us >= 1000000) {
                const float elapsed = (float)(t2 - last_report_us) / 1000000.0f;
                vision_metrics_t m{};
                m.task = VISION_METRICS_TASK_AI;
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
    
            // 先把原始帧交给 LCD 显示（确保 LCD 任务取到帧后再收集叠加命令）
            if (s_out) {
                if (xQueueSend(s_out, &frame, 0) == pdPASS) { 
                    handed = true; frame = nullptr; 
                }
            }

            // 再把边框投递到 LCD 叠加队列（由 LCD 任务统一绘制，避免并发访问面板）
            if (lcd_is_overlay_enabled()) {
                for (int i = 0; i < num; ++i) {
                    const y11_box_t &b = boxes[i];
                    if (b.x1 >= b.x2 || b.y1 >= b.y2) continue;
                    lcd_overlay_post_bbox_from_src(src_w, src_h, b.x1, b.y1, b.x2, b.y2, RED);
                }
            }
        } while (0);

        if (!handed && frame) { esp_camera_fb_return(frame); frame = nullptr; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (frame) {
        esp_camera_fb_return(frame);
        frame = nullptr;
    }
    if (g_ai_exit_sem) {
        xSemaphoreGive(g_ai_exit_sem);
    }
    g_ai_task = nullptr;
    vTaskDelete(nullptr);
}

void register_AIdetection(QueueHandle_t frame_in, QueueHandle_t frame_out, const char *model_path, float conf_thresh)
{
    if (g_ai_task) {
        unregister_AIdetection();
    }

    if (!g_ai_exit_sem) {
        g_ai_exit_sem = xSemaphoreCreateBinary();
        if (!g_ai_exit_sem) {
            ESP_LOGW(TAG, "create ai exit sem failed");
        }
    }
    if (g_ai_exit_sem) {
        (void)xSemaphoreTake(g_ai_exit_sem, 0);
    }

    g_ai_stop_requested = false;
    s_in = frame_in;
    s_out = frame_out;
    g_model_path = model_path ? model_path : "";
    g_conf_thresh = conf_thresh;

    BaseType_t ok = xTaskCreatePinnedToCore(ai_detect_task, TAG, 10*1024, nullptr, 5, &g_ai_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create ai_detect_task failed (%ld)", (long)ok);
        g_ai_task = nullptr;
    }
}

void unregister_AIdetection(void)
{
    if (g_ai_task) {
        if (!g_ai_exit_sem) {
            g_ai_exit_sem = xSemaphoreCreateBinary();
            if (!g_ai_exit_sem) {
                ESP_LOGW(TAG, "create ai exit sem failed");
            }
        }
        g_ai_stop_requested = true;

        TickType_t wait_ticks = pdMS_TO_TICKS(g_model ? 800 : 3000);
        if (g_ai_exit_sem) {
            if (xSemaphoreTake(g_ai_exit_sem, wait_ticks) != pdTRUE) {
                ESP_LOGW(TAG, "ai task stop timeout, force delete");
                vTaskDelete(g_ai_task);
            }
        } else {
            vTaskDelete(g_ai_task);
        }
        g_ai_task = nullptr;
    }

    g_ai_stop_requested = false;
    s_in = nullptr;
    s_out = nullptr;
    g_model_path.clear();
    g_conf_thresh = 0.0f;
    y11_deinit();
}
