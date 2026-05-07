#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEVICE_IDENTITY_STATUS_OK = 0,
    DEVICE_IDENTITY_STATUS_UNPROVISIONED = 1,
    DEVICE_IDENTITY_STATUS_CORRUPT = 2,
    DEVICE_IDENTITY_STATUS_MISMATCH = 3,
    DEVICE_IDENTITY_STATUS_ERROR = 4,
} device_identity_status_t;

esp_err_t device_identity_init(void);

device_identity_status_t device_identity_get_status(void);

esp_err_t device_identity_get_sn(char *out, size_t out_len);

const char *device_identity_get_oem(void);
const char *device_identity_get_fw_version(void);
esp_err_t device_identity_set_fw_version_label(const char *label);
esp_err_t device_identity_set_fw_version_label_from_filename(const char *filename);

esp_err_t device_identity_get_base_mac(uint8_t mac_out[6]);
esp_err_t device_identity_get_base_mac_str(char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
