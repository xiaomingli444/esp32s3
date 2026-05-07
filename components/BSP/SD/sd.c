#include "sd.h"

#include <stdlib.h>
#include <ctype.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static sdmmc_card_t *card = NULL;
static bool s_sd_mounted_once = false;

static char TAG[]="SD";

uint32_t Flash_Size = 0;
uint32_t SDCard_Size = 0;

static bool sd_mount_point_ready(void)
{
    struct stat st;
    return stat("/sdcard", &st) == 0;
}

void sd_init(void)
{
    if (card != NULL) {
        if (sd_mount_point_ready()) {
            return;
        }
        card = NULL;
    }

    esp_err_t ret;
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,                         //若挂载不成功是否需要格式化SD卡
        .max_files = 5,                                         //最大打开文件数
        .allocation_unit_size = 16 * 1024                       //分配单元为14k
    };
    const char mount_point[] = "/sdcard";                       //定义挂载点

    mount_config.format_if_mount_failed = !s_sd_mounted_once;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripgeral");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();                   //SDMMC主机接口配置，直接使用默认配置
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();  //SDMMC插槽配置
    slot_config.width = 1;                                          //设置为1线SD模式
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;           //打开内部上拉电阻

    ESP_LOGI(TAG, "mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);


    if(ret != ESP_OK)
    {
        if(ret == ESP_FAIL)
        {
            ESP_LOGI(TAG, "Failed to mount filesystem");
        }else
        {
            ESP_LOGI(TAG, "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        card = NULL;
        return;
    }


    ESP_LOGI(TAG, "Filesystem mounted");                            //提示挂载成功
    sdmmc_card_print_info(stdout, card);                            //终端打印SD卡的一些信息
    sd_cleanup_model_temp_files();
    s_sd_mounted_once = true;
    SDCard_Size = ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024);
    if(esp_flash_get_physical_size(NULL, &Flash_Size) == ESP_OK)
    {
        Flash_Size = Flash_Size / (uint32_t)(1024 * 1024);
        printf("Flash size: %ld MB\n", Flash_Size);
    }
    else{
        printf("Get flash size failed\n");
    }
    
}

esp_err_t sdio_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");                                     //以写的方式打开文件
    if(f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);                                                      //关闭文件
    ESP_LOGI(TAG, "File written");
    return ESP_OK;
}


esp_err_t sdio_read_file(const char *path)
{
    char flieline[SDIO_FILELINE_MAX_CHAR_SIZE];
    char *pos =NULL;

    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if(f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    fgets(flieline, sizeof(flieline), f);
    fclose(f);
    pos = strchr(flieline, '\n');
    if(pos)
    {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file : '%s'", flieline);
    return ESP_OK;
}


void sdcard_test(void)
{
    esp_err_t ret;
    const char *tfile = "/sdcard/test01.txt";
    char data[SDIO_FILELINE_MAX_CHAR_SIZE];
    
    snprintf(data, SDIO_FILELINE_MAX_CHAR_SIZE, "%s %s!\n", "sdcard_test", card->cid.name);
    ret = sdio_write_file(tfile, data);                     //写sdcard 测试
    if(ret != ESP_OK)
    {
        return;
    }

    ret = sdio_read_file(tfile);                            //读sdcard 测试
    if(ret != ESP_OK)
    {
        return;
    }

    esp_vfs_fat_sdcard_unmount("/sdcard", card);            //卸载sd 卡
    ESP_LOGI(TAG, "Card unmounted");

}

static bool ends_with_casei(const char *name, const char *sfx)
{
    if (!name || !sfx) return false;
    size_t n = strlen(name);
    size_t m = strlen(sfx);
    if (m > n) return false;
    for (size_t i = 0; i < m; ++i) {
        char a = (char)toupper((unsigned char)name[n - m + i]);
        char b = (char)toupper((unsigned char)sfx[i]);
        if (a != b) return false;
    }
    return true;
}

static bool is_model_file(const char *name)
{
    /* FATFS may be built without LFN (CONFIG_FATFS_LFN_NONE=y), so
     * ".espdl" becomes the short 8.3 alias ".esp". Accept both. */
    return ends_with_casei(name, ".espdl") || ends_with_casei(name, ".esp");
}

static bool is_temp_file(const char *name)
{
    return ends_with_casei(name, ".part") || ends_with_casei(name, ".tmp");
}

static bool append_text(char **buf, size_t *cap, size_t *pos, const char *text)
{
    if (!buf || !cap || !pos || !text) return false;
    const size_t add = strlen(text);
    if (add == 0) return true;

    const size_t need = *pos + add + 1;
    if (need > *cap) {
        size_t new_cap = (*cap == 0) ? 256 : *cap;
        while (new_cap < need) {
            if (new_cap >= 8192) {
                return false;
            }
            new_cap *= 2;
        }
        char *p = (char *)realloc(*buf, new_cap);
        if (!p) return false;
        *buf = p;
        *cap = new_cap;
    }

    memcpy(*buf + *pos, text, add);
    *pos += add;
    (*buf)[*pos] = '\0';
    return true;
}

void sd_cleanup_model_temp_files(void)
{
    if (card == NULL || !sd_mount_point_ready()) {
        sd_init();
    }
    if (card == NULL || !sd_mount_point_ready()) {
        return;
    }

    mkdir("/sdcard/models", 0775);

    DIR *d = opendir("/sdcard/models");
    if (!d) {
        return;
    }

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        const char *name = e->d_name;
        if (!name || name[0] == '\0') continue;
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) continue;
        if (!is_temp_file(name)) continue;

        char full[384];
        int n = snprintf(full, sizeof(full), "/sdcard/models/%s", name);
        if (n > 0 && (size_t)n < sizeof(full)) {
            unlink(full);
        }
    }
    closedir(d);
}

char *sd_list_models_alloc(void)
{
    char *buf = (char *)malloc(256);
    if (!buf) return NULL;
    size_t cap = 256;
    size_t pos = 0;
    buf[0] = '\0';

    (void)append_text(&buf, &cap, &pos, "未设置AI模型");

    if (card == NULL || !sd_mount_point_ready()) {
        sd_init();
    }
    if (card == NULL || !sd_mount_point_ready()) {
        return buf;
    }

    sd_cleanup_model_temp_files();
    mkdir("/sdcard/models", 0775);

    DIR *d = opendir("/sdcard/models");
    if (!d) {
        return buf;
    }

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        const char *name = e->d_name;
        if (!name || name[0] == '\0') continue;
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) continue;
        if (is_temp_file(name)) continue;
        if (!is_model_file(name)) continue;

        if (!append_text(&buf, &cap, &pos, "\n")) break;
        if (!append_text(&buf, &cap, &pos, name)) break;
    }
    closedir(d);

    return buf;
}

void sd_list_models(char *buf, size_t len)
{
    if (!buf || len == 0) return;

    char *opts = sd_list_models_alloc();
    if (!opts) {
        buf[0] = '\0';
        return;
    }
    snprintf(buf, len, "%s", opts);
    free(opts);
}
