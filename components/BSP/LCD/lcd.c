#include "lcd.h"
#include "wifi.h"
#include <string.h>

static const char *TAG = "LCD";

esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t xQueueFrameIn = NULL;
static QueueHandle_t xQueueFrameOut = NULL;
static TaskHandle_t s_lcd_task_handle = NULL;
static uartp_task_t current_task;
static ledc_channel_config_t ledc_channel;
static SemaphoreHandle_t s_panel_mutex = NULL;
static TickType_t s_frame_period_ticks = pdMS_TO_TICKS(33); // 默认 ~30fps
// 叠加绘制命令队列（由其他任务投递，LCD 任务统一绘制）
typedef struct {
    int src_w, src_h;
    int x1, y1, x2, y2;
    uint16_t color;
} lcd_overlay_cmd_t;

static QueueHandle_t xQueueOverlay = NULL;
static bool s_overlay_enabled = true;

static uint16_t *s_line = NULL;  // 单行 DMA 缓冲（172 像素）

/* LCD 帧率统计：最近 1 秒内的帧数 */
static volatile uint32_t s_lcd_fps = 0;
static uint32_t s_lcd_frame_cnt = 0;
static int64_t s_lcd_last_ts_us = 0;
static bool s_fps_overlay_dirty = false;

/* Minimal 5x7 font for FPS overlay (scaled after lookup). */
#define FPS_FONT_W 5
#define FPS_FONT_H 7
#define FPS_FONT_SPACING 1
#define FPS_FONT_SCALE 2
#define FPS_CHAR_W (FPS_FONT_W * FPS_FONT_SCALE)
#define FPS_CHAR_H (FPS_FONT_H * FPS_FONT_SCALE)
#define FPS_SPACING_PX (FPS_FONT_SPACING * FPS_FONT_SCALE)
#define FPS_MARGIN 6
/* RGB565 close to LVGL's UI blue (0x2A7FFF). */
#define FPS_FONT_COLOR 0x2BFF

static const uint8_t s_font_5x7[][FPS_FONT_H] = {
    /* 0 - 9 */
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, /* 0 */
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, /* 1 */
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, /* 2 */
    {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E}, /* 3 */
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, /* 4 */
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}, /* 5 */
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, /* 6 */
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, /* 7 */
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, /* 8 */
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}, /* 9 */
    /* F, P, S, space */
    {0x1F, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10}, /* F */
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}, /* P */
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, /* S */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
};

static const uint8_t *font5x7_get_rows(char c)
{
    if (c >= '0' && c <= '9') {
        return s_font_5x7[c - '0'];
    }
    switch (c) {
    case 'F': return s_font_5x7[10];
    case 'P': return s_font_5x7[11];
    case 'S': return s_font_5x7[12];
    case ' ': return s_font_5x7[13];
    default:  return NULL;
    }
}

static void lcd_draw_char_5x7(const uint8_t *rows,
                              int x, int y,
                              uint16_t color,
                              uint16_t *line_buf)
{
    if (!rows) return;
    for (int r = 0; r < FPS_FONT_H; ++r) {
        uint8_t bits = rows[r];
        int col = 0;
        while (col < FPS_FONT_W) {
            while (col < FPS_FONT_W &&
                   ((bits & (1U << (FPS_FONT_W - 1 - col))) == 0)) {
                ++col;
            }
            if (col >= FPS_FONT_W) break;
            int start = col;
            while (col < FPS_FONT_W &&
                   (bits & (1U << (FPS_FONT_W - 1 - col)))) {
                ++col;
            }
            int run = col - start;
            int run_px = run * FPS_FONT_SCALE;
            for (int i = 0; i < run_px; ++i) {
                line_buf[i] = color;
            }
            int x0 = x + start * FPS_FONT_SCALE;
            int y0 = y + r * FPS_FONT_SCALE;
            esp_lcd_panel_draw_bitmap(panel_handle,
                                      x0, y0,
                                      x0 + run_px, y0 + FPS_FONT_SCALE,
                                      line_buf);
        }
    }
}

static void lcd_draw_text_5x7(const char *text, int x, int y, uint16_t color)
{
    if (!text || !*text) return;
    if (!lcd_panel_lock(pdMS_TO_TICKS(20))) {
        return;
    }

    uint16_t line_buf[FPS_FONT_W * FPS_FONT_SCALE];
    for (int i = 0; i < FPS_FONT_W * FPS_FONT_SCALE; ++i) line_buf[i] = color;

    int cx = x;
    for (const char *p = text; *p; ++p) {
        const uint8_t *rows = font5x7_get_rows(*p);
        lcd_draw_char_5x7(rows, cx, y, color, line_buf);
        cx += FPS_CHAR_W + FPS_SPACING_PX;
    }

    lcd_panel_unlock();
}


static void lcd_draw_fps_overlay(uint32_t fps)
{
    char num_buf[8];
    char text[16];
    snprintf(num_buf, sizeof(num_buf), "%lu", (unsigned long)fps);
    snprintf(text, sizeof(text), "FPS %s", num_buf);

    size_t len = strlen(text);
    int text_w = (int)len * FPS_CHAR_W + (int)(len - 1) * FPS_SPACING_PX;
    int text_h = FPS_CHAR_H;
    int x = LCD_W - text_w - FPS_MARGIN;
    int y = LCD_H - text_h - FPS_MARGIN;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    lcd_draw_text_5x7(text, x, y, FPS_FONT_COLOR);
}

static void ensure_panel_mutex(void)
{
    if (s_panel_mutex == NULL) {
        s_panel_mutex = xSemaphoreCreateRecursiveMutex();
    }
}

bool lcd_panel_lock(TickType_t timeout)
{
    ensure_panel_mutex();
    if (!s_panel_mutex) {
        return false;
    }
    return xSemaphoreTakeRecursive(s_panel_mutex, timeout) == pdTRUE;
}

void lcd_panel_unlock(void)
{
    if (s_panel_mutex) {
        xSemaphoreGiveRecursive(s_panel_mutex);
    }
}

// ===================== 便捷绘制：屏幕十字 =====================
// 在 (cx,cy) 画一个十字，臂长为 half_len（左右/上下各延伸 half_len 像素）
// 注意：坐标基于当前屏幕方向（0..LCD_W-1, 0..LCD_H-1）
void lcd_draw_cross(int cx, int cy, int half_len, uint16_t color)
{
    if (!lcd_panel_lock(pdMS_TO_TICKS(50))) {
        return;
    }

    if (half_len < 0) half_len = 0;

    // 限制中心点范围
    if (cx < 0) { cx = 0; }
    if (cx >= LCD_W) { cx = LCD_W - 1; }
    if (cy < 0) { cy = 0; }
    if (cy >= LCD_H) { cy = LCD_H - 1; }

    int x0 = cx - half_len; if (x0 < 0) { x0 = 0; }
    int x1 = cx + half_len; if (x1 >= LCD_W) { x1 = LCD_W - 1; }
    int y0 = cy - half_len; if (y0 < 0) { y0 = 0; }
    int y1 = cy + half_len; if (y1 >= LCD_H) { y1 = LCD_H - 1; }

    const int thickness = 3; // 加粗中心十字

    // 水平线：y 方向画 thickness 条，x in [x0,x1]
    if (x1 >= x0) {
        int w = x1 - x0 + 1;
        uint16_t buf_h[LCD_W];
        for (int i = 0; i < w; ++i) buf_h[i] = color;
        int half_t = thickness / 2;
        for (int dy = -half_t; dy <= half_t; ++dy) {
            int yy = cy + dy;
            if (yy < y0 || yy > y1) continue;
            esp_lcd_panel_draw_bitmap(panel_handle, x0, yy, x0 + w, yy + 1, buf_h);
        }
    }

    // 垂直线：x 方向画 thickness 条，y in [y0,y1]
    if (y1 >= y0) {
        int h = y1 - y0 + 1;
        uint16_t buf_v[LCD_H];
        for (int i = 0; i < h; ++i) buf_v[i] = color;
        int half_t = thickness / 2;
        for (int dx = -half_t; dx <= half_t; ++dx) {
            int xx = cx + dx;
            if (xx < x0 || xx > x1) continue;
            esp_lcd_panel_draw_bitmap(panel_handle, xx, y0, xx + 1, y0 + h, buf_v);
        }
    }

    lcd_panel_unlock();
}

// 根据原图坐标在屏幕上画矩形框（匹配旋转后的视频显示区域，贴底显示）
void lcd_draw_bbox_from_src(int src_w, int src_h, int x1, int y1, int x2, int y2, uint16_t color)
{
    if (!s_overlay_enabled) return;
    if (src_w <= 0 || src_h <= 0) return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }

    // 与 lcd_draw_frame_blocking 相同的显示窗口：172x230，贴底
    const int view_w = LCD_W; // 172
    const int view_h_req = 230;
    int view_y = LCD_H - view_h_req;
    if (view_y < 0) view_y = 0;
    int view_h = view_h_req;
    if (view_y + view_h > LCD_H) view_h = LCD_H - view_y;

    // screen_x = ((src_h - 1 - y_s) * view_w) / src_h
    // screen_y = view_y + (x_s * view_w) / src_h
    if (x1 < 0) x1 = 0;
    if (x1 >= src_w) x1 = src_w - 1;
    if (x2 < 0) x2 = 0;
    if (x2 >= src_w) x2 = src_w - 1;
    if (y1 < 0) y1 = 0;
    if (y1 >= src_h) y1 = src_h - 1;
    if (y2 < 0) y2 = 0;
    if (y2 >= src_h) y2 = src_h - 1;
    if (x2 > x1) x2 -= 1;
    if (y2 > y1) y2 -= 1;

    const int sx_top   = ((src_h - 1 - y1) * view_w) / src_h;
    const int sx_bot   = ((src_h - 1 - y2) * view_w) / src_h;
    const int sy_left  = view_y + (x1 * view_w) / src_h;
    const int sy_right = view_y + (x2 * view_w) / src_h;

    int x_min = sx_bot < sx_top ? sx_bot : sx_top;
    int x_max = sx_bot > sx_top ? sx_bot : sx_top;
    int y_min = sy_left < sy_right ? sy_left : sy_right;
    int y_max = sy_left > sy_right ? sy_left : sy_right;

    if (x_min < 0) x_min = 0;
    if (x_min >= view_w) x_min = view_w - 1;
    if (x_max < 0) x_max = 0;
    if (x_max >= view_w) x_max = view_w - 1;
    if (y_min < 0) y_min = 0;
    if (y_min >= LCD_H) y_min = LCD_H - 1;
    if (y_max < 0) y_max = 0;
    if (y_max >= LCD_H) y_max = LCD_H - 1;

    uint16_t buf_h[LCD_W];
    uint16_t buf_v[LCD_H];

    if (!lcd_panel_lock(pdMS_TO_TICKS(50))) {
        return;
    }

    if (x_max >= x_min) {
        int w = x_max - x_min + 1;
        for (int i = 0; i < w; ++i) buf_h[i] = color;
        esp_lcd_panel_draw_bitmap(panel_handle, x_min, y_min, x_min + w, y_min + 1, buf_h);
        esp_lcd_panel_draw_bitmap(panel_handle, x_min, y_max, x_min + w, y_max + 1, buf_h);
    }

    if (y_max >= y_min) {
        int h = y_max - y_min + 1;
        for (int i = 0; i < h; ++i) buf_v[i] = color;
        int col_top = sx_top; if (col_top < 0) col_top = 0; if (col_top >= view_w) col_top = view_w - 1;
        esp_lcd_panel_draw_bitmap(panel_handle, col_top, y_min, col_top + 1, y_min + h, buf_v);
        int col_bot = sx_bot; if (col_bot < 0) col_bot = 0; if (col_bot >= view_w) col_bot = view_w - 1;
        esp_lcd_panel_draw_bitmap(panel_handle, col_bot, y_min, col_bot + 1, y_min + h, buf_v);
    }

    lcd_panel_unlock();
}

// 对外：非 LCD 任务调用，投递一个叠加框命令（非阻塞）
void lcd_overlay_post_bbox_from_src(int src_w, int src_h, int x1, int y1, int x2, int y2, uint16_t color)
{
    if (!s_overlay_enabled) return;
    if (!xQueueOverlay) return;
    lcd_overlay_cmd_t cmd = { src_w, src_h, x1, y1, x2, y2, color };
    (void)xQueueSend(xQueueOverlay, &cmd, 0);
}

void BK_Init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    
    // 配置LEDC
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel    = LEDC_HS_CH0_CHANNEL;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = EXAMPLE_PIN_NUM_BK_LIGHT;
    ledc_channel.speed_mode = LEDC_LS_MODE;
    ledc_channel.timer_sel  = LEDC_HS_TIMER;
    ledc_channel_config(&ledc_channel);
    ledc_fade_func_install(0);
}

void BK_Light(uint8_t Light)
{   
    if(Light > 100) Light = 100;
    uint16_t Duty = LEDC_MAX_Duty-(81*(100-Light));
    if(Light == 0) Duty = 0;
    // 设置PWM占空比
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, Duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void lcd_init(void)
{
    ensure_panel_mutex();

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {                                                         
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,                                            
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,                                            
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,                                            
        .quadwp_io_num = -1,                                                            
        .quadhd_io_num = -1,                                                            
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t),    
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));            

    ESP_LOGI(TAG, "Install panel IO");                                              
    esp_lcd_panel_io_handle_t io_handle = NULL;                                         
    esp_lcd_panel_io_spi_config_t io_config = {                                             
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 1,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,       
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));


    esp_lcd_panel_dev_st7789t_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ST7789T panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789t(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    /* 兼容“芯片重启但 LCD 未掉电”的场景：额外发送一次 SWRESET，避免面板状态机异常导致花屏 */
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x01, NULL, 0)); // SWRESET
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    esp_lcd_panel_set_gap(panel_handle, 34, 0); // 关键：X 偏移
    
    BK_Init();                                                                                          // Initialize the backlight
    BK_Light(75);                                                                                       // Set the backlight brightness to 75%
}

static inline uint16_t bswap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

static inline uint16_t gray_to_rgb565(uint8_t g)
{
    uint16_t r5 = (uint16_t)(g >> 3);
    uint16_t g6 = (uint16_t)(g >> 2);
    uint16_t b5 = (uint16_t)(g >> 3);
    uint16_t v  = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
    return v;
}

static uint16_t s_gray565_lut[256];
static bool s_gray565_lut_ready = false;

static inline void ensure_gray565_lut(void)
{
    if (s_gray565_lut_ready) return;
    for (int g = 0; g < 256; ++g) {
        s_gray565_lut[g] = gray_to_rgb565((uint8_t)g);
    }
    s_gray565_lut_ready = true;
}

static void lcd_draw_frame_blocking(const camera_fb_t *fb)
{
    bool locked = lcd_panel_lock(pdMS_TO_TICKS(100));
    if (!locked) {
        ESP_LOGW(TAG, "panel lock timeout, drop frame");
        return;
    }

    if (!s_line) {
        s_line = (uint16_t *)heap_caps_malloc(LCD_W * sizeof(uint16_t),
                                              MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!s_line) { 
            ESP_LOGE(TAG, "DMA 行缓冲分配失败"); 
            lcd_panel_unlock();
            return; 
        }
    }

    const int SRC_W_ = (int)fb->width;    // 原宽（如 320）
    const int SRC_H_ = (int)fb->height;   // 原高（如 240）

    // 旋转90°（顺时针）后的宽高
    const int Wr = SRC_H_;                // 旋后宽 = 原高
    const int Hr = SRC_W_;                // 旋后高 = 原宽

    // 只在底部 172x230 区域显示，保留上方 UI
    const int view_w = LCD_W;             // 172
    const int view_h_req = 230;
    int view_y = LCD_H - view_h_req;
    if (view_y < 0) view_y = 0;
    int view_h = view_h_req;
    if (view_y + view_h > LCD_H) view_h = LCD_H - view_y;

    // 按宽等比缩放到视窗高度（覆盖整个视窗，不留白边）
    const int scaled_h = view_h;          // 全高填充

    // x 方向端点覆盖映射（确保右缘不漏 1 列）
    static int s_map_x[LCD_W];
    static int cache_wr = -1, cache_wd = -1;
    if (cache_wr != Wr || cache_wd != view_w) {
        if (view_w <= 1) s_map_x[0] = 0;
        else {
            for (int x = 0; x < view_w; ++x)
                s_map_x[x] = (x * (Wr - 1)) / (view_w - 1);  // 0..Wr-1
        }
        cache_wr = Wr; cache_wd = view_w;
    }

    const uint8_t  *src8  = (const uint8_t *)fb->buf;    // GRAY: 1B/px
    const uint16_t *src16 = (const uint16_t *)fb->buf;   // RGB565: 2B/px

    const bool is_gray = (fb->format == PIXFORMAT_GRAYSCALE);
    if (is_gray) ensure_gray565_lut();

    for (int y = 0; y < view_h; ++y) {
        const int y_r = (scaled_h <= 1) ? 0
                       : (y * (Hr - 1)) / (scaled_h - 1); // 0..Hr-1

        if (!is_gray && fb->format == PIXFORMAT_RGB565) {
            // RGB565 路径：与现状相同，做一次字节交换
            for (int x = 0; x < view_w; ++x) {
                const int x_r = s_map_x[x];                 // 0..Wr-1
                const int x_s = y_r;
                const int y_s = SRC_H_ - 1 - x_r;
                const int idx = y_s * SRC_W_ + x_s;
                s_line[x] = bswap16(src16[idx]);
            }
        } else {
            // GRAYSCALE 路径：用已交换字节序的 LUT，保证 r=g=b
            for (int x = 0; x < view_w; ++x) {
                const int x_r = s_map_x[x];
                const int x_s = y_r;
                const int y_s = SRC_H_ - 1 - x_r;
                const int idx = y_s * SRC_W_ + x_s;
                s_line[x] = s_gray565_lut[src8[idx]];
            }
        }
        const int dest_y = view_y + y;
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, dest_y, view_w, dest_y + 1, s_line));
    }

    lcd_panel_unlock();
}


void lcd_test()
{
    camera_fb_t *fb = NULL;
    lcd_init();
    camera_init(PIXFORMAT_GRAYSCALE,FRAMESIZE_QVGA, 2);
    while (1)
    {
        fb = esp_camera_fb_get();
        if (!fb) {
            printf("进入lcd_test出了问题\n");
        }
        if (!fb) {
            ESP_LOGE(TAG, "摄像头获取帧失败");
            return;
        }
        lcd_draw_frame_blocking(fb);
        esp_camera_fb_return(fb);
    }    
}

static void lcd_task_handler(void *arg)
{
    camera_fb_t *frame = NULL;
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        // 阻塞等待帧，避免空转占满 CPU 触发 WDT
        if (xQueueReceive(xQueueFrameIn, &frame, portMAX_DELAY)) {
            /* FPS 统计：累加帧数，每秒更新一次当前帧率 */
            s_lcd_frame_cnt++;
            int64_t now = esp_timer_get_time();
            if (s_lcd_last_ts_us == 0) {
                s_lcd_last_ts_us = now;
            } else if (now - s_lcd_last_ts_us >= 1000000) {
                s_lcd_fps = s_lcd_frame_cnt;
                s_lcd_frame_cnt = 0;
                s_lcd_last_ts_us = now;
                s_fps_overlay_dirty = true; // 每秒刷新一次 FPS 文本，降低额外开销
            }

            lcd_draw_frame_blocking(frame);  
            /* 仅 Find-Color 模式显示中心十字 */
            if (current_task == UARTP_TASK_COLOR_LEARN) {
                const int view_h_req = 230;
                int view_y = LCD_H - view_h_req;
                if (view_y < 0) view_y = 0;
                int view_h = view_h_req;
                if (view_y + view_h > LCD_H) view_h = LCD_H - view_y;
                const int cx = LCD_W / 2;
                const int cy = view_y + (view_h / 2);
                lcd_draw_cross(cx, cy, 10, BLACK); // 居中十字
            }

            // TaskNone 默认只做本地预览；若有单帧/连续图传请求，则也走推流路径
            bool single = wifi_stream_consume_single_shot();
            bool streaming = wifi_stream_is_running();
            const bool overlay_pending = s_overlay_enabled && xQueueOverlay && (uxQueueMessagesWaiting(xQueueOverlay) > 0);
            const bool preview_mode = (current_task == UARTP_TASK_NONE) && !single && !streaming && !overlay_pending;

            if (!preview_mode) {
                // 统一在 LCD 任务里绘制所有叠加框，避免跨任务同时访问面板；收集供图传叠加
                lcd_overlay_cmd_t collected[8];
                int collected_cnt = 0;
                if (xQueueOverlay) {
                    lcd_overlay_cmd_t cmd;
                    // 给 AI 任务一个很短的时间把本帧的叠加命令投递进来
                    // 然后尽可能把叠加命令全部取出并绘制
                    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2);
                    do {
                        while (xQueueReceive(xQueueOverlay, &cmd, 0) == pdTRUE) {
                            if (s_overlay_enabled) {
                                lcd_draw_bbox_from_src(cmd.src_w, cmd.src_h, cmd.x1, cmd.y1, cmd.x2, cmd.y2, cmd.color);
                                if (collected_cnt < (int)(sizeof(collected)/sizeof(collected[0]))) {
                                    collected[collected_cnt++] = cmd;
                                }
                            }
                        }
                    } while (xTaskGetTickCount() < deadline);
                }

                // WebSocket 图传：取色任务之外才推送，Frame-ON 时叠加检测框
                if (current_task != UARTP_TASK_COLOR_LEARN) {
                    bool want_overlay = s_overlay_enabled;
                    if (want_overlay && frame && frame->format == PIXFORMAT_RGB565) {
                        uint16_t *p = (uint16_t *)frame->buf;
                        const int w = frame->width;
                        const int h = frame->height;
                        for (int i = 0; i < collected_cnt; ++i) {
                            const lcd_overlay_cmd_t *c = &collected[i];
                            int x1 = c->x1; int y1 = c->y1; int x2 = c->x2; int y2 = c->y2;
                            if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
                            if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
                            if (x1 < 0) x1 = 0;
                            if (y1 < 0) y1 = 0;
                            if (x2 >= w) x2 = w - 1;
                            if (y2 >= h) y2 = h - 1;
                            const uint16_t color = c->color;
                            // top/bottom
                            for (int x = x1; x <= x2; ++x) {
                                p[y1 * w + x] = color;
                                p[y2 * w + x] = color;
                            }
                            // left/right
                            for (int y = y1; y <= y2; ++y) {
                                p[y * w + x1] = color;
                                p[y * w + x2] = color;
                            }
                        }
                    }
                    if (single) {
                        /* STA 模式下单帧也走 JPEG，避免 RGB565 大包导致前端卡顿 */
                        if (wifi_get_run_mode() == WIFI_RUN_MODE_AP) {
                            if (!wifi_stream_send_single_rgb565(frame)) {
                                wifi_stream_send_single_jpeg(frame);
                            }
                        } else {
                            wifi_stream_send_single_jpeg(frame);
                        }
                    } else {
                        wifi_stream_push_if_running(frame);
                    }
                }
            } else {
                // 纯预览：跳过叠加/FPS绘制/图传，减轻 CPU 和总线负担
                if (xQueueOverlay) {
                    xQueueReset(xQueueOverlay);
                }
            }
            // FPS 文字仅在数值更新时绘制一次，避免对帧率的持续影响；所有任务均显示
            if (s_fps_overlay_dirty) {
                lcd_draw_fps_overlay(s_lcd_fps);
                s_fps_overlay_dirty = false;
            }

            // 归还帧给摄像头驱动，避免帧池耗尽导致 cam_hal 溢出
            esp_camera_fb_return(frame);

            /* 控制刷新节奏，把时间片让给 LVGL/按键等任务 */
            vTaskDelayUntil(&last_wake, s_frame_period_ticks);
        }
        // 理论上不会走到这里；保险留一丝时间片
        
        vTaskDelay(1);
    }
}

uint32_t lcd_get_fps(void)
{
    return s_lcd_fps;
}

void lcd_set_frame_interval_ms(uint32_t interval_ms)
{
    if (interval_ms < 10) interval_ms = 10;   // 防止过快挤占 CPU
    if (interval_ms > 100) interval_ms = 100; // 无需过慢导致拖影
    s_frame_period_ticks = pdMS_TO_TICKS(interval_ms);
}


void register_lcd(const QueueHandle_t frame_in, 
    const QueueHandle_t frame_out, 
    uartp_task_t task)
{
    xQueueFrameIn = frame_in;
    xQueueFrameOut = frame_out;
    current_task = task;
    if (s_lcd_task_handle) {
        lcd_task_delete();
    }
    if (!xQueueOverlay) {
        xQueueOverlay = xQueueCreate(32, sizeof(lcd_overlay_cmd_t));
    }
    s_frame_period_ticks = pdMS_TO_TICKS(33); // 默认恢复 30fps
    /* 降低优先级，避免长期占用 CPU 影响 LVGL 和按键 */
    s_lcd_task_handle = NULL;
    const int max_retry = 3;
    for (int i = 0; i < max_retry; ++i) {
        BaseType_t ok = xTaskCreatePinnedToCore(lcd_task_handler, TAG, 4 * 1024, NULL, 2, &s_lcd_task_handle, 0);
        if (ok == pdPASS && s_lcd_task_handle) {
            break;
        }
        s_lcd_task_handle = NULL;
        ESP_LOGW(TAG, "create lcd task failed, retry %d/%d", i + 1, max_retry);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_lcd_task_handle) {
        ESP_LOGE(TAG, "create lcd task failed after retries");
    }
    
}

QueueHandle_t lcd_get_overlay_queue(void)
{
    if (!xQueueOverlay) {
        xQueueOverlay = xQueueCreate(32, sizeof(lcd_overlay_cmd_t));
    }
    return xQueueOverlay;
}

void lcd_set_overlay_enabled(bool enabled)
{
    s_overlay_enabled = enabled;
}

bool lcd_is_overlay_enabled(void)
{
    return s_overlay_enabled;
}

void lcd_task_suspend(void)
{
    if (s_lcd_task_handle) {
        /* 等待当前面板操作完成再挂起，避免持锁被暂停导致 UI 刷新阻塞 */
        if (!lcd_panel_lock(pdMS_TO_TICKS(1000))) {
            ESP_LOGW(TAG, "panel busy too long, skip suspending LCD task");
            return;
        }
        vTaskSuspend(s_lcd_task_handle);
        lcd_panel_unlock();
    }
}

void lcd_task_resume(void)
{
    if (s_lcd_task_handle) {
        if (eTaskGetState(s_lcd_task_handle) == eSuspended) {
            vTaskResume(s_lcd_task_handle);
        }
    }
}

bool lcd_task_is_created(void)
{
    return s_lcd_task_handle != NULL;
}

bool lcd_task_is_suspended(void)
{
    return s_lcd_task_handle && eTaskGetState(s_lcd_task_handle) == eSuspended;
}

void lcd_task_delete(void)
{
    if (s_lcd_task_handle) {
        if (!lcd_panel_lock(portMAX_DELAY)) {
            ESP_LOGE(TAG, "panel lock unavailable, skip deleting LCD task");
            return;
        }
        vTaskDelete(s_lcd_task_handle);
        s_lcd_task_handle = NULL;
        lcd_panel_unlock();
    }
}
