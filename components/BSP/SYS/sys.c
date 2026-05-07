#include "sys.h"

static const char* TAG = "NVS_init";


void NVS_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

bool nvs_read_rgb(uint8_t color_id, uint8_t *r, uint8_t *g, uint8_t *b)
{
    // 入参合法性检查（避免空指针访问，C语言需手动处理）
    if (r == NULL || g == NULL || b == NULL) {
        return false;
    }

    nvs_handle_t h;
    // 打开 NVS 分区（"storage" 为分区名，只读模式）
    if (nvs_open("storage", NVS_READONLY, &h) != ESP_OK) {
        return false;
    }

    char rk[16], gk[16], bk[16];
    // 生成 NVS 键名（如 "red0", "green1"）
    snprintf(rk, sizeof(rk), "red%u", color_id);
    snprintf(gk, sizeof(gk), "green%u", color_id);
    snprintf(bk, sizeof(bk), "blue%u", color_id);

    uint8_t rv = 0, gv = 0, bv = 0;
    esp_err_t er = nvs_get_u8(h, rk, &rv);
    esp_err_t eg = nvs_get_u8(h, gk, &gv);
    esp_err_t eb = nvs_get_u8(h, bk, &bv);
    // 关闭 NVS 句柄（必须执行，避免资源泄漏）
    nvs_close(h);

    // 若 R/G/B 均读取成功，赋值并返回 true
    if (er == ESP_OK && eg == ESP_OK && eb == ESP_OK) {
        *r = (uint8_t)rv;  // 指针解引用赋值（C语言无引用，用指针间接访问）
        *g = (uint8_t)gv;
        *b = (uint8_t)bv;
        return true;
    }

    // 任一值读取失败，返回 false（原默认值不变）
    return false;
}






