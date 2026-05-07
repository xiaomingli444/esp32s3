#include "uartp.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG          "UARTP"
#define LOCAL_RX_TMP 2048

// ===== 全局状态 =====
static uart_port_t   s_port    = UART_NUM_MAX;
static TaskHandle_t  s_rx_task = NULL;
static TaskHandle_t  s_tx_task = NULL;
static QueueHandle_t s_tx_q    = NULL;
static uartp_config_t s_cfg    = {0};

// 当前任务状态
static uartp_task_state_t s_task_state = { 
    .current_task = UARTP_TASK_NONE, 
    .current_color_id = 0,
    .tag_dict = none,
};

// 回调
static uartp_on_color_learn_req_cb s_cb_d1 = NULL;
static uartp_on_color_detect_req_cb s_cb_d2 = NULL;
static uartp_on_apriltag_req_cb    s_cb_d3 = NULL;
static uartp_on_line_follow_req_cb s_cb_d4 = NULL;
static uartp_on_ai_detect_req_cb   s_cb_d5 = NULL;
static uartp_on_frame_overlay_req_cb s_cb_d6 = NULL;
static uartp_on_fill_light_req_cb  s_cb_d7 = NULL;
static uartp_on_stop_req_cb        s_cb_d8 = NULL;
static uartp_on_empty_task_req_cb  s_cb_d9 = NULL;
static uartp_on_msroi_ctrl_req_cb  s_cb_da = NULL;

void uartp_set_handlers(uartp_on_color_learn_req_cb d1_cb,
                        uartp_on_color_detect_req_cb d2_cb,
                        uartp_on_apriltag_req_cb    d3_cb,
                        uartp_on_line_follow_req_cb d4_cb,
                        uartp_on_ai_detect_req_cb   d5_cb,
                        uartp_on_frame_overlay_req_cb d6_cb,
                        uartp_on_fill_light_req_cb  d7_cb,
                        uartp_on_stop_req_cb        d8_cb,
                        uartp_on_empty_task_req_cb  d9_cb,
                        uartp_on_msroi_ctrl_req_cb  da_cb)
{
    s_cb_d1 = d1_cb; s_cb_d2 = d2_cb; s_cb_d3 = d3_cb; s_cb_d4 = d4_cb;
    s_cb_d5 = d5_cb; s_cb_d6 = d6_cb; s_cb_d7 = d7_cb; s_cb_d8 = d8_cb; s_cb_d9 = d9_cb; s_cb_da = da_cb;
}

void uartp_get_current_task(uartp_task_state_t *out_state){
    if (out_state) *out_state = s_task_state;
}

void uartp_set_task_state(uartp_task_t task, uint8_t color_id, tagformat_t tag)
{
    s_task_state.current_task    = task;
    s_task_state.current_color_id = color_id;
    s_task_state.tag_dict        = tag;
}

// ===== 大端工具 =====
static inline void be16_write(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static inline void be32_write(uint8_t *p, uint32_t v){ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static inline void befloat_write(uint8_t *p, float f){ uint32_t v; memcpy(&v,&f,4); be32_write(p,v); }
static inline float befloat_read(const uint8_t *p){
    uint32_t v=((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|(uint32_t)p[3];
    float f; memcpy(&f,&v,4); return f;
}

// ===== Fletcher-16（CKA/CKB）=====
static void fletcher16(const uint8_t *data, size_t len, uint8_t *cka, uint8_t *ckb){
    uint16_t s1=0,s2=0; for (size_t i=0;i<len;++i){ s1=(s1+data[i])&0xFF; s2=(s2+s1)&0xFF; }
    *cka=(uint8_t)s1; *ckb=(uint8_t)s2;
}

// ===== 发送打包 =====
static esp_err_t proto_send(uint8_t dev, uint8_t msg, const uint8_t *payload, uint8_t plen){
    if (s_port == UART_NUM_MAX) return ESP_FAIL;
    uint8_t buf[4 + 255 + 2];
    size_t  w = 0;
    buf[w++] = UARTP_STX;
    buf[w++] = dev;
    buf[w++] = msg;
    buf[w++] = plen;
    if (plen && payload){ memcpy(&buf[w], payload, plen); w += plen; }
    uint8_t cka=0,ckb=0; fletcher16(buf, w, &cka, &ckb);
    buf[w++] = cka; buf[w++] = ckb;
    int written = uart_write_bytes(s_port, (const char*)buf, w);
    return (written == (int)w) ? ESP_OK : ESP_FAIL;
}

// ===== TX 事件（发包队列） =====
typedef enum {
    EV_D1H_LEARN_RSP = 0,
    EV_D2H_DETECT_RSP,
    EV_D3H_TAG_RSP,
    EV_D4H_LINE_RSP,
    EV_D5H_AI_RSP
} tx_evt_type_t;

typedef struct {
    tx_evt_type_t type;
    union {
        uartp_d1h_color_learn_rsp_t d1;
        uartp_d2h_color_detect_rsp_t d2;
        uartp_d3h_apriltag_rsp_t    d3;
        uartp_d4h_line_follow_rsp_t d4;
        uartp_d5h_ai_detect_rsp_t   d5;
    } u;
} tx_evt_t;

// ===== 三类既有应答 =====
static esp_err_t send_d1_rsp(const uartp_d1h_color_learn_rsp_t *p){
    uint8_t pay[3] = { p->R, p->G, p->B };
    return proto_send(UARTP_DEV_DEVICE, UARTP_MSG_D1H, pay, sizeof(pay));
}
static esp_err_t send_d2_rsp(const uartp_d2h_color_detect_rsp_t *p){
    uint8_t pay[1+4+4+4]; size_t w=0;
    pay[w++] = p->valid ? 1 : 0;
    befloat_write(&pay[w], p->cx);       w+=4;
    befloat_write(&pay[w], p->cy);       w+=4;
    befloat_write(&pay[w], p->area_pct); w+=4;
    return proto_send(UARTP_DEV_DEVICE, UARTP_MSG_D2H, pay, w);
}
static esp_err_t send_d3_rsp(const uartp_d3h_apriltag_rsp_t *p){
    uint8_t pay[1+2+4+4+4+4+4]; size_t w=0;
    pay[w++] = p->valid ? 1 : 0;
    be16_write(&pay[w], (uint16_t)p->tag_id); w+=2; // 覆盖 0..586
    befloat_write(&pay[w], p->cx);        w+=4;
    befloat_write(&pay[w], p->cy);        w+=4;
    befloat_write(&pay[w], p->area_pct);  w+=4;
    befloat_write(&pay[w], p->yaw_deg);   w+=4;
    befloat_write(&pay[w], p->dist_cm);   w+=4;
    return proto_send(UARTP_DEV_DEVICE, UARTP_MSG_D3H, pay, w);
}

// ===== D4H 巡线应答（含交叉口类型&置信度）=====
static esp_err_t send_d4_rsp(const uartp_d4h_line_follow_rsp_t *p){
    uint8_t pay[1 + 4 + 4 + 4 + 1 + 4];
    size_t  w = 0;
    pay[w++] = p->valid ? 1 : 0;
    befloat_write(&pay[w], p->offset_x);        w += 4;
    befloat_write(&pay[w], p->heading_err_deg); w += 4;
    befloat_write(&pay[w], p->line_width_pct);  w += 4;
    pay[w++] = (uint8_t)p->intersect_type;      // 0/1/2
    befloat_write(&pay[w], p->intersect_conf);  w += 4;
    return proto_send(UARTP_DEV_DEVICE, UARTP_MSG_D4H, pay, w);
}

// ===== D5H AI检测应答 =====
static esp_err_t send_d5_rsp(const uartp_d5h_ai_detect_rsp_t *p){
    uint8_t pay[2 + 13 * UARTP_D5H_MAX_DETS];
    size_t  w = 0;
    uint8_t n = p->num_dets;
    if (n > UARTP_D5H_MAX_DETS) n = UARTP_D5H_MAX_DETS;
    pay[w++] = p->valid ? 1 : 0;
    pay[w++] = n;
    for (uint8_t i = 0; i < n; ++i) {
        const uartp_d5h_det_t *d = &p->dets[i];
        pay[w++] = d->class_id;
        befloat_write(&pay[w], d->score); w += 4;
        be16_write(&pay[w], d->x1);       w += 2;
        be16_write(&pay[w], d->y1);       w += 2;
        be16_write(&pay[w], d->x2);       w += 2;
        be16_write(&pay[w], d->y2);       w += 2;
    }
    return proto_send(UARTP_DEV_DEVICE, UARTP_MSG_D5H, pay, (uint8_t)w);
}

// ===== D5H AI检测应答 =====
// ===== Host->Device 分发 & 任务切换 =====
static inline void set_task(uartp_task_t t, uint8_t color_id){
    s_task_state.current_task   = t;
    s_task_state.current_color_id = color_id;
    ESP_LOGI(TAG, "Task switched to %d (color_id=%u)", t, color_id);
}

static void handle_host_request(uint8_t msg, const uint8_t *p, uint8_t n){
    switch(msg){
    case UARTP_MSG_D1H: // 颜色学习 [color_id]
        if (n<1){ ESP_LOGW(TAG,"D1H len err"); break; }
        set_task(UARTP_TASK_COLOR_LEARN, p[0]);
        if (s_cb_d1) s_cb_d1(p[0]);
        break;
    case UARTP_MSG_D2H: // 颜色检测 [color_id]
        if (n<1){ ESP_LOGW(TAG,"D2H len err"); break; }
        set_task(UARTP_TASK_COLOR_DETECT, p[0]);
        if (s_cb_d2) s_cb_d2(p[0]);
        break;
    case UARTP_MSG_D3H: // AprilTag 无载荷
        tagformat_t dict = none;
        if (p[0] == 0x01)      dict = tag16h5;
        else if (p[0] == 0x02) dict = tag36h11;
        set_task(UARTP_TASK_APRILTAG, 0);
        s_task_state.tag_dict = dict;
        if (s_cb_d3) s_cb_d3();
        break;
    case UARTP_MSG_D4H: // 智能巡线 [color_id]
        if (n<1){ ESP_LOGW(TAG,"D4H len err"); break; }
        set_task(UARTP_TASK_LINE_FOLLOW, p[0]);
        if (s_cb_d4) s_cb_d4(p[0]);
        break;
    case UARTP_MSG_D5H: // AI检测 [model_id, score_id]
        if (n<2){ ESP_LOGW(TAG,"D5H len err"); break; }
        set_task(UARTP_TASK_AI_DETECT, 0);
        if (s_cb_d5) s_cb_d5(p[0], p[1]);
        break;
    case UARTP_MSG_D6H: // frame overlay [state]
        if (n<1){ ESP_LOGW(TAG,"D6H len err"); break; }
        if (s_cb_d6) s_cb_d6(p[0] != 0);
        break;
    case UARTP_MSG_D7H: // fill light [state]
        if (n<1){ ESP_LOGW(TAG,"D7H len err"); break; }
        if (s_cb_d7) s_cb_d7(p[0] != 0);
        break;
    case UARTP_MSG_D8H: // stop current task [reason]
        if (n<1){ ESP_LOGW(TAG,"D8H len err"); break; }
        if (s_cb_d8) s_cb_d8(p[0]);
        break;
    case UARTP_MSG_D9H: // empty task (camera preview) [state]
        if (n<1){ ESP_LOGW(TAG,"D9H len err"); break; }
        if (p[0] != 0) {
            set_task(UARTP_TASK_NONE, 0);
        }
        if (s_cb_d9) s_cb_d9(p[0] != 0);
        break;
    case UARTP_MSG_DAH: // msroi/module switch [msroi, module1, module2, module3]
        if (n != 4){ ESP_LOGW(TAG,"DAH len err"); break; }
        if (s_cb_da) s_cb_da(p[0] != 0, p[1] != 0, p[2] != 0, p[3] != 0);
        break;
    default:
        ESP_LOGW(TAG,"unknown msg=0x%02X", msg);
        break;
    }
}

// ===== RX 状态机 =====
typedef enum { RX_STX=0,RX_DEV,RX_MSG,RX_LEN,RX_PAY,RX_CKA,RX_CKB } rx_state_t;
typedef struct{
    rx_state_t st;
    uint8_t dev,msg,len;
    uint8_t pay[255], idx;
    uint8_t cka,ckb;
    uint8_t sum_buf[4+255];
    uint16_t sum_n;
} rx_ctx_t;

static rx_ctx_t s_rx;

static void rx_reset(void){ memset(&s_rx,0,sizeof(s_rx)); s_rx.st=RX_STX; }
static void rx_sum_push(uint8_t b){ if (s_rx.sum_n<sizeof(s_rx.sum_buf)) s_rx.sum_buf[s_rx.sum_n++]=b; }

static void rx_feed(uint8_t b){
    switch(s_rx.st){
    case RX_STX:
        if (b==UARTP_STX){ rx_reset(); s_rx.st=RX_DEV; rx_sum_push(b); }
        break;
    case RX_DEV: s_rx.dev=b; s_rx.st=RX_MSG; rx_sum_push(b); break;
    case RX_MSG: s_rx.msg=b; s_rx.st=RX_LEN; rx_sum_push(b); break;
    case RX_LEN:
        s_rx.len=b; s_rx.idx=0; rx_sum_push(b);
        s_rx.st = (s_rx.len==0)?RX_CKA:RX_PAY; break;
    case RX_PAY:
        s_rx.pay[s_rx.idx++]=b; rx_sum_push(b);
        if (s_rx.idx>=s_rx.len) {s_rx.st=RX_CKA; break;}
        break;
    case RX_CKA: s_rx.cka=b; s_rx.st=RX_CKB; break;
    case RX_CKB: {
        s_rx.ckb=b;
        uint8_t a=0,k=0; fletcher16(s_rx.sum_buf, s_rx.sum_n, &a, &k);
        bool ok=(a==s_rx.cka)&&(k==s_rx.ckb);
        if (!ok){ ESP_LOGW(TAG,"ck fail (%02X,%02X)!=exp(%02X,%02X)",s_rx.cka,s_rx.ckb,a,k); rx_reset(); break; }
        if (s_rx.dev==UARTP_DEV_HOST) handle_host_request(s_rx.msg, s_rx.pay, s_rx.len);
        rx_reset(); break;
    }
    default: rx_reset(); break;
    }
}

// ===== 任务：RX & TX =====
static void rx_task(void *arg){
    uint8_t *buf = (uint8_t*)malloc(LOCAL_RX_TMP);
    rx_reset();
    while(1){
        int n = uart_read_bytes(s_port, buf, LOCAL_RX_TMP, pdMS_TO_TICKS(50));
        if (n>0) for(int i=0;i<n;++i) rx_feed(buf[i]);
    }
}

static esp_err_t send_by_evt(const tx_evt_t *e){
    switch(e->type){
    case EV_D1H_LEARN_RSP: return send_d1_rsp(&e->u.d1);
    case EV_D2H_DETECT_RSP:return send_d2_rsp(&e->u.d2);
    case EV_D3H_TAG_RSP:   return send_d3_rsp(&e->u.d3);
    case EV_D4H_LINE_RSP:  return send_d4_rsp(&e->u.d4);
    case EV_D5H_AI_RSP:    return send_d5_rsp(&e->u.d5);
    default: return ESP_OK;
    }
}

static void tx_task(void *arg){
    tx_evt_t e;
    while(1){
        if (xQueueReceive(s_tx_q, &e, portMAX_DELAY)){
            (void)send_by_evt(&e);
        }
    }
}

// ===== 启动 / 停止（含 UART 初始化） =====
esp_err_t uartp_start(const uartp_config_t *cfg){
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_rx_task || s_tx_task) return ESP_ERR_INVALID_STATE;

    s_cfg = *cfg;
    s_port = cfg->port;

    const uart_config_t ucfg = {
        .baud_rate  = cfg->baud > 0 ? cfg->baud : 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(s_port,
                        cfg->rx_buf > 0 ? cfg->rx_buf : 2048,
                        cfg->tx_buf > 0 ? cfg->tx_buf : 2048,
                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_port, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(s_port, cfg->tx_gpio, cfg->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_tx_q = xQueueCreate(16, sizeof(tx_evt_t));
    if (!s_tx_q) return ESP_ERR_NO_MEM;

    BaseType_t ok;
    ok = xTaskCreatePinnedToCore(rx_task, "uartp_rx",
                                 4096, NULL,
                                 cfg->rx_task_prio>0?cfg->rx_task_prio:10,
                                 &s_rx_task,
                                 (cfg->rx_task_core>=0)?cfg->rx_task_core:tskNO_AFFINITY);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ok = xTaskCreatePinnedToCore(tx_task, "uartp_tx",
                                 4096, NULL,
                                 cfg->tx_task_prio>0?cfg->tx_task_prio:9,
                                 &s_tx_task,
                                 (cfg->tx_task_core>=0)?cfg->tx_task_core:tskNO_AFFINITY);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "started: UART%u TXD=%d RXD=%d %d baud",
             (unsigned)s_port, cfg->tx_gpio, cfg->rx_gpio, ucfg.baud_rate);
    return ESP_OK;
}

void uartp_stop(void){
    if (s_rx_task){ vTaskDelete(s_rx_task); s_rx_task=NULL; }
    if (s_tx_task){ vTaskDelete(s_tx_task); s_tx_task=NULL; }
    if (s_tx_q){ vQueueDelete(s_tx_q); s_tx_q=NULL; }
    if (s_port != UART_NUM_MAX){
        uart_driver_delete(s_port);
        s_port = UART_NUM_MAX;
    }
}

// ===== 对外：上报接口 =====
bool uartp_post_color_learn(uint8_t R, uint8_t G, uint8_t B){
    if (!s_tx_q) return false;
    tx_evt_t e = { .type = EV_D1H_LEARN_RSP };
    e.u.d1.R=R; e.u.d1.G=G; e.u.d1.B=B;
    return xQueueSend(s_tx_q, &e, 0) == pdTRUE;
}
bool uartp_post_color_detect(bool valid, float cx, float cy, float area_pct){
    if (!s_tx_q) return false;
    tx_evt_t e = { .type = EV_D2H_DETECT_RSP };
    e.u.d2.valid = valid?1:0; e.u.d2.cx=cx; e.u.d2.cy=cy; e.u.d2.area_pct=area_pct;
    return xQueueSend(s_tx_q, &e, 0) == pdTRUE;
}
bool uartp_post_apriltag(bool valid, int tag_id, float cx, float cy,
                         float area_pct, float yaw_deg, float dist_cm){
    if (!s_tx_q) return false;
    tx_evt_t e = { .type = EV_D3H_TAG_RSP };
    e.u.d3.valid = valid?1:0;
    e.u.d3.tag_id = (int16_t)tag_id;
    e.u.d3.cx=cx; e.u.d3.cy=cy; e.u.d3.area_pct=area_pct;
    e.u.d3.yaw_deg=yaw_deg; e.u.d3.dist_cm=dist_cm;
    return xQueueSend(s_tx_q, &e, 0) == pdTRUE;
}
bool uartp_post_line_follow(bool valid, float offset_x, float heading_err_deg,
                            float line_width_pct, uint8_t intersect_type, float intersect_conf){
    if (!s_tx_q) return false;
    tx_evt_t e = { .type = EV_D4H_LINE_RSP };
    e.u.d4.valid = valid?1:0;
    e.u.d4.offset_x = offset_x;
    e.u.d4.heading_err_deg = heading_err_deg;
    e.u.d4.line_width_pct = line_width_pct;
    e.u.d4.intersect_type = intersect_type;   // 0/1/2
    e.u.d4.intersect_conf = intersect_conf;   // 0..1
    return xQueueSend(s_tx_q, &e, 0) == pdTRUE;
}

bool uartp_post_ai_detect(uint8_t valid, uint8_t num_dets, const uartp_d5h_det_t *dets){
    if (!s_tx_q) return false;
    tx_evt_t e = { .type = EV_D5H_AI_RSP };
    if (!dets || num_dets == 0) {
        e.u.d5.valid = 0;
        e.u.d5.num_dets = 0;
    } else {
        if (num_dets > UARTP_D5H_MAX_DETS) num_dets = UARTP_D5H_MAX_DETS;
        e.u.d5.valid = valid ? 1 : 0;
        e.u.d5.num_dets = num_dets;
        memcpy(e.u.d5.dets, dets, sizeof(uartp_d5h_det_t) * num_dets);
    }
    return xQueueSend(s_tx_q, &e, 0) == pdTRUE;
}
