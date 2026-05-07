#include "find_color.h"
using namespace cv;
using namespace std;

static const char *TAG = "find_color";
static QueueHandle_t xQueueFrameIn = NULL;
static QueueHandle_t xQueueFrameOut = NULL;
static uint8_t rgb_index = 0;
static TaskHandle_t s_find_color_task = NULL;
uint8_t R[7] = {0}, G[7] = {0}, B[7] = {0};
static volatile bool s_learn_pending = false;
static volatile uint8_t s_learn_color_id = 0;

// 5/6bit 到 8bit 更线性映射（比位复制更准）
static inline uint8_t up5_to_8(uint8_t v5) { return (uint8_t)((v5 * 255 + 15) / 31); }
static inline uint8_t up6_to_8(uint8_t v6) { return (uint8_t)((v6 * 255 + 31) / 63); }

// 高字节在前的 RGB565 两字节转 8bit RGB
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


static void Find_color(camera_fb_t *fb)
{
    if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) {
        return;
    }
    const int w = fb->width;
    const int h = fb->height;
    const uint8_t *base = (const uint8_t *)fb->buf;

    const int cx = w / 2;
    const int cy = h / 2;

    int x0 = cx - HALF_WIN; if (x0 < 0)     x0 = 0;
    int y0 = cy - HALF_WIN; if (y0 < 0)     y0 = 0;
    int x1 = cx + HALF_WIN; if (x1 > w - 1) x1 = w - 1;
    int y1 = cy + HALF_WIN; if (y1 > h - 1) y1 = h - 1;

    uint32_t rs = 0, gs = 0, bs = 0;
    uint32_t cnt = 0;

    for (int y = y0; y <= y1; ++y) {
        size_t row_off = (size_t)y * w * 2;
        const uint8_t *p = base + row_off + (size_t)x0 * 2; // 指向第一个像素的高字�?
        for (int x = x0; x <= x1; ++x) {
            uint8_t r8, g8, b8;
            // 高字节在前：p[0]=HI, p[1]=LO
            rgb565_hi_first_to_rgb8(p[0], p[1], &r8, &g8, &b8);
            rs += r8; gs += g8; bs += b8; ++cnt;
            p += 2;
        }
    }
    if (cnt == 0) return;
    R[rgb_index] = (uint8_t)(rs / cnt);
    G[rgb_index] = (uint8_t)(gs / cnt);
    B[rgb_index] = (uint8_t)(bs / cnt);

    // 将RGB值保存到NVS（键名：red{color_id}/green{color_id}/blue{color_id}�?
    nvs_handle_t my_handle;
    char red_key[16], green_key[16], blue_key[16];
    snprintf(red_key, sizeof(red_key), "red%u", rgb_index+1);
    snprintf(green_key, sizeof(green_key), "green%u", rgb_index+1);
    snprintf(blue_key, sizeof(blue_key), "blue%u", rgb_index+1);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);  // 打开NVS存储区域
    if (err == ESP_OK) {
        esp_err_t er = nvs_set_u8(my_handle, red_key, R[rgb_index]);    // 保存红色分量
        esp_err_t eg = nvs_set_u8(my_handle, green_key, G[rgb_index]);  // 保存绿色分量
        esp_err_t eb = nvs_set_u8(my_handle, blue_key, B[rgb_index]);   // 保存蓝色分量
        // 提交更改
        err = nvs_commit(my_handle);
        if (er != ESP_OK || eg != ESP_OK || eb != ESP_OK || err != ESP_OK) {
            ESP_LOGE(TAG, "save RGB failed(id=%u) er=%d eg=%d eb=%d commit=%d", rgb_index+1, (int)er, (int)eg, (int)eb, (int)err);
        } else {
            ESP_LOGI(TAG, "save RGB ok: id=%u keys=(%s,%s,%s) values=(R=%u,G=%u,B=%u)",
                     rgb_index+1, red_key, green_key, blue_key, R[rgb_index], G[rgb_index], B[rgb_index]);
        }
        nvs_close(my_handle);  // 关闭NVS
    } else {
        ESP_LOGE(TAG, "open NVS failed err=%d", (int)err);
    }
    uartp_post_color_learn(R[rgb_index], G[rgb_index], B[rgb_index]);
}

void find_color_request_learn(uint8_t color_id)
{
    if (color_id < 1 || color_id > 7) {
        s_learn_pending = false;
        s_learn_color_id = 0;
        return;
    }
    s_learn_color_id = color_id;
    s_learn_pending = true;
}



static void find_color_task_handler(void *arg)
{
    camera_fb_t *frame = NULL;

    while (1) {
        if (xQueueReceive(xQueueFrameIn, &frame, portMAX_DELAY)) {

            bool do_learn = false;
            if (lvgl_ui_consume_web_clicked()) {
                ESP_LOGI(TAG, "start color learn via Web button");
                do_learn = true;
            }

            uint8_t req_id = s_learn_color_id;
            if (s_learn_pending && req_id != 0 && req_id == (uint8_t)(rgb_index + 1)) {
                s_learn_pending = false;
                ESP_LOGI(TAG, "start color learn via UART request (id=%u)", (unsigned)req_id);
                do_learn = true;
            }

            if (do_learn) {
                Find_color(frame);
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
        }
    }
}

void register_find_color(const QueueHandle_t frame_in, const QueueHandle_t frame_out, const uint8_t color_id)
{
    xQueueFrameIn = frame_in;
    xQueueFrameOut = frame_out;
    rgb_index = (color_id > 0) ? (color_id - 1) : 0; 
    if (s_find_color_task) {
        vTaskDelete(s_find_color_task);
        s_find_color_task = NULL;
    }
    xTaskCreatePinnedToCore(find_color_task_handler, TAG, 4 * 1024, NULL, 4, &s_find_color_task, 0);
}

void unregister_find_color(void)
{
    if (s_find_color_task) {
        vTaskDelete(s_find_color_task);
        s_find_color_task = NULL;
    }
    xQueueFrameIn = NULL;
    xQueueFrameOut = NULL;
}
