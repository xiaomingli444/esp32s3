#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

void NVS_init(void);
bool nvs_read_rgb(uint8_t color_id, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif

