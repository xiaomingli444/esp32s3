#ifndef __SD_H_
#define __SD_H_
#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include "string.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_flash.h"  
#include "driver/sdmmc_host.h"



#define SD_PIN_CMD  GPIO_NUM_1   // SD_CMD
#define SD_PIN_CLK  GPIO_NUM_2   // SD_CLK
#define SD_PIN_D0   GPIO_NUM_3   // SD_DATA0

#define SDIO_FILELINE_MAX_CHAR_SIZE         64

// 全局变量只声明
extern uint32_t Flash_Size;
extern uint32_t SDCard_Size;


void sd_init(void);
esp_err_t sdio_write_file(const char *path, char *data);
esp_err_t sdio_read_file(const char *path);
void sdcard_test(void);
void sd_list_models(char *buf, size_t len);
char *sd_list_models_alloc(void);
void sd_cleanup_model_temp_files(void);



#ifdef __cplusplus
}
#endif

#endif /* __SD_H_ */













