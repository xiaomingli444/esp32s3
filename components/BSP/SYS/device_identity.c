#include "device_identity.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs.h"

#define DEVICE_IDENTITY_NVS_NS  "device"
#define DEVICE_IDENTITY_NVS_KEY "identity"
#define DEVICE_IDENTITY_NVS_KEY_FW_LABEL "fw_label"

static const char *TAG = "device_identity";

typedef struct {
    uint8_t magic[4];     /* "IDV1" */
    uint8_t ver;          /* 1 */
    uint8_t reserved0[3];
    uint8_t mac_bind[6];  /* eFuse Base MAC */
    uint8_t reserved1[2];
    char sn[48];          /* "XXXX-XXXX-XX" */
    uint32_t crc32;
} device_identity_blob_v1_t;

static device_identity_status_t s_status = DEVICE_IDENTITY_STATUS_UNPROVISIONED;
static device_identity_blob_v1_t s_identity;
static bool s_loaded = false;
static bool s_fw_label_loaded = false;
static char s_fw_label[32] = {0}; /* "YYYY-MM-Vx.y" */

static bool ends_with_casei(const char *s, const char *suffix)
{
    if (!s || !suffix) return false;
    size_t slen = strlen(s);
    size_t tlen = strlen(suffix);
    if (tlen == 0 || tlen > slen) return false;
    const char *p = s + slen - tlen;
    for (size_t i = 0; i < tlen; ++i) {
        char a = p[i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

static bool is_digit_ascii(char c)
{
    return c >= '0' && c <= '9';
}

static bool is_fw_label_char(char c)
{
    return (is_digit_ascii(c) ||
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            c == '-' || c == '_' || c == '.');
}

static bool fw_label_format_valid(const char *label)
{
    if (!label) return false;
    const size_t len = strlen(label);
    if (len < 10) return false;

    if (!is_digit_ascii(label[0]) || !is_digit_ascii(label[1]) ||
        !is_digit_ascii(label[2]) || !is_digit_ascii(label[3])) {
        return false;
    }
    if (label[4] != '-') return false;
    if (!is_digit_ascii(label[5]) || !is_digit_ascii(label[6])) return false;
    if (label[7] != '-') return false;
    if (!(label[8] == 'V' || label[8] == 'v')) return false;

    int month = (label[5] - '0') * 10 + (label[6] - '0');
    if (month < 1 || month > 12) return false;

    for (size_t i = 0; i < len; ++i) {
        if (!is_fw_label_char(label[i])) return false;
    }
    return true;
}

static bool extract_fw_label_from_filename(const char *filename, char *out, size_t out_len)
{
    if (!out || out_len == 0) return false;
    out[0] = '\0';
    if (!filename || filename[0] == '\0') return false;

    const char *base = filename;
    const char *slash = strrchr(filename, '/');
    const char *bslash = strrchr(filename, '\\');
    if (slash && bslash) {
        base = (slash > bslash) ? (slash + 1) : (bslash + 1);
    } else if (slash) {
        base = slash + 1;
    } else if (bslash) {
        base = bslash + 1;
    }
    if (!base || base[0] == '\0') {
        base = filename;
    }

    const size_t len = strlen(base);
    if (len <= 4 || !ends_with_casei(base, ".bin")) {
        return false;
    }

    size_t label_len = len - 4;
    if (label_len >= out_len) {
        return false;
    }

    for (size_t i = 0; i < label_len; ++i) {
        if (!is_fw_label_char(base[i])) {
            return false;
        }
        out[i] = base[i];
    }
    out[label_len] = '\0';

    if (!fw_label_format_valid(out)) {
        out[0] = '\0';
        return false;
    }
    if (out[8] == 'v') {
        out[8] = 'V';
    }
    return true;
}

static void fw_label_load_once(void)
{
    if (s_fw_label_loaded) return;
    s_fw_label[0] = '\0';

    nvs_handle_t h;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return;
    }
    s_fw_label_loaded = true;

    size_t sz = sizeof(s_fw_label);
    err = nvs_get_str(h, DEVICE_IDENTITY_NVS_KEY_FW_LABEL, s_fw_label, &sz);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_fw_label[0] = '\0';
        return;
    }
    if (err != ESP_OK) {
        s_fw_label[0] = '\0';
        return;
    }
    if (!fw_label_format_valid(s_fw_label)) {
        s_fw_label[0] = '\0';
    }
    if (s_fw_label[8] == 'v') {
        s_fw_label[8] = 'V';
    }
}

static esp_err_t fw_label_save(const char *label)
{
    if (!label) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, DEVICE_IDENTITY_NVS_KEY_FW_LABEL, label);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static uint32_t crc32_ieee(const void *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)p[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static const char *base32_alphabet(void)
{
    /* Crockford Base32: 0-9, A-Z (no I,L,O,U) */
    return "0123456789ABCDEFGHJKMNPQRSTVWXYZ";
}

static esp_err_t base32_encode(const uint8_t *in, size_t in_len, char *out, size_t out_len)
{
    if (!in || !out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t needed = (in_len * 8 + 4) / 5;
    if (out_len < needed + 1) {
        return ESP_ERR_NO_MEM;
    }

    const char *alpha = base32_alphabet();
    uint32_t buffer = 0;
    int bits_left = 0;
    size_t pos = 0;

    for (size_t i = 0; i < in_len; ++i) {
        buffer = (buffer << 8) | in[i];
        bits_left += 8;
        while (bits_left >= 5) {
            uint8_t idx = (buffer >> (bits_left - 5)) & 0x1F;
            out[pos++] = alpha[idx];
            bits_left -= 5;
        }
    }

    if (bits_left > 0) {
        uint8_t idx = (buffer << (5 - bits_left)) & 0x1F;
        out[pos++] = alpha[idx];
    }

    out[pos] = '\0';
    return ESP_OK;
}

static esp_err_t generate_sn(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 4-4-2: "XXXX-XXXX-XX" */
    if (out_len < 13) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t rnd[8];
    esp_fill_random(rnd, sizeof(rnd));

    char enc[16];
    esp_err_t err = base32_encode(rnd, sizeof(rnd), enc, sizeof(enc));
    if (err != ESP_OK) {
        return err;
    }

    if (strlen(enc) < 10) {
        return ESP_FAIL;
    }
    enc[10] = '\0';

    int n = snprintf(out, out_len, "%.4s-%.4s-%.2s", enc, enc + 4, enc + 8);
    if (n < 0 || (size_t)n >= out_len) {
        out[0] = '\0';
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static bool identity_blob_valid(const device_identity_blob_v1_t *id)
{
    if (!id) return false;
    if (memcmp(id->magic, "IDV1", 4) != 0) return false;
    if (id->ver != 1) return false;
    if (memchr(id->sn, '\0', sizeof(id->sn)) == NULL) return false;
    if (id->sn[0] == '\0') return false;

    uint32_t calc = crc32_ieee(id, offsetof(device_identity_blob_v1_t, crc32));
    return calc == id->crc32;
}

static esp_err_t identity_load_from_nvs(device_identity_blob_v1_t *out, bool *found)
{
    if (found) *found = false;
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    size_t len = 0;
    err = nvs_get_blob(h, DEVICE_IDENTITY_NVS_KEY, NULL, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(h);
        return err;
    }
    if (len != sizeof(*out)) {
        nvs_close(h);
        return ESP_ERR_INVALID_SIZE;
    }

    err = nvs_get_blob(h, DEVICE_IDENTITY_NVS_KEY, out, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return err;
    }

    if (found) *found = true;
    return ESP_OK;
}

static esp_err_t identity_save_to_nvs(const device_identity_blob_v1_t *id)
{
    if (!id) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(DEVICE_IDENTITY_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(h, DEVICE_IDENTITY_NVS_KEY, id, sizeof(*id));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t device_identity_get_base_mac(uint8_t mac_out[6])
{
    if (!mac_out) return ESP_ERR_INVALID_ARG;
    return esp_efuse_mac_get_default(mac_out);
}

esp_err_t device_identity_get_base_mac_str(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;

    uint8_t mac[6];
    esp_err_t err = device_identity_get_base_mac(mac);
    if (err != ESP_OK) {
        out[0] = '\0';
        return err;
    }

    int n = snprintf(out, out_len,
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2],
                     (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
    if (n < 0 || (size_t)n >= out_len) {
        out[0] = '\0';
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

const char *device_identity_get_oem(void)
{
    return "KPUAV";
}

const char *device_identity_get_fw_version(void)
{
    fw_label_load_once();
    if (s_fw_label[0] != '\0') {
        return s_fw_label;
    }
    const esp_app_desc_t *desc = esp_app_get_description();
    if (!desc || desc->version[0] == '\0') {
        return "-";
    }
    return desc->version;
}

esp_err_t device_identity_set_fw_version_label_from_filename(const char *filename)
{
    char label[sizeof(s_fw_label)] = {0};
    if (!extract_fw_label_from_filename(filename, label, sizeof(label))) {
        return ESP_ERR_INVALID_ARG;
    }
    return device_identity_set_fw_version_label(label);
}

esp_err_t device_identity_set_fw_version_label(const char *label)
{
    if (!label || label[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (!fw_label_format_valid(label)) {
        return ESP_ERR_INVALID_ARG;
    }

    char norm[sizeof(s_fw_label)] = {0};
    size_t len = strnlen(label, sizeof(norm));
    if (len == 0 || len >= sizeof(norm)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(norm, label, len);
    norm[len] = '\0';
    if (norm[8] == 'v') {
        norm[8] = 'V';
    }

    esp_err_t err = fw_label_save(norm);
    if (err != ESP_OK) {
        return err;
    }

    s_fw_label_loaded = true;
    snprintf(s_fw_label, sizeof(s_fw_label), "%s", norm);
    return ESP_OK;
}

esp_err_t device_identity_init(void)
{
    if (s_loaded) {
        return ESP_OK;
    }

    uint8_t mac_now[6];
    esp_err_t err = device_identity_get_base_mac(mac_now);
    if (err != ESP_OK) {
        s_status = DEVICE_IDENTITY_STATUS_ERROR;
        return err;
    }

    device_identity_blob_v1_t id;
    memset(&id, 0, sizeof(id));

    bool found = false;
    err = identity_load_from_nvs(&id, &found);
    if (err != ESP_OK) {
        s_status = DEVICE_IDENTITY_STATUS_ERROR;
        ESP_LOGW(TAG, "nvs load failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!found) {
        memcpy(id.magic, "IDV1", 4);
        id.ver = 1;
        memcpy(id.mac_bind, mac_now, sizeof(id.mac_bind));
        err = generate_sn(id.sn, sizeof(id.sn));
        if (err != ESP_OK) {
            s_status = DEVICE_IDENTITY_STATUS_ERROR;
            ESP_LOGW(TAG, "sn generate failed: %s", esp_err_to_name(err));
            return err;
        }
        id.crc32 = crc32_ieee(&id, offsetof(device_identity_blob_v1_t, crc32));

        err = identity_save_to_nvs(&id);
        if (err != ESP_OK) {
            s_status = DEVICE_IDENTITY_STATUS_ERROR;
            ESP_LOGW(TAG, "nvs save failed: %s", esp_err_to_name(err));
            return err;
        }

        s_identity = id;
        s_status = DEVICE_IDENTITY_STATUS_OK;
        s_loaded = true;
        ESP_LOGI(TAG, "SN created: %s", s_identity.sn);
        return ESP_OK;
    }

    s_identity = id;
    s_loaded = true;

    if (!identity_blob_valid(&s_identity)) {
        s_status = DEVICE_IDENTITY_STATUS_CORRUPT;
        ESP_LOGW(TAG, "identity blob corrupt");
        return ESP_OK;
    }

    if (memcmp(s_identity.mac_bind, mac_now, sizeof(s_identity.mac_bind)) != 0) {
        s_status = DEVICE_IDENTITY_STATUS_MISMATCH;
        ESP_LOGW(TAG, "identity mac mismatch");
        return ESP_OK;
    }

    s_status = DEVICE_IDENTITY_STATUS_OK;
    return ESP_OK;
}

device_identity_status_t device_identity_get_status(void)
{
    (void)device_identity_init();
    return s_status;
}

esp_err_t device_identity_get_sn(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out[0] = '\0';

    esp_err_t err = device_identity_init();
    if (err != ESP_OK) {
        return err;
    }
    if (s_status != DEVICE_IDENTITY_STATUS_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t sn_len = strnlen(s_identity.sn, sizeof(s_identity.sn));
    if (sn_len == 0 || sn_len >= sizeof(s_identity.sn) || sn_len + 1 > out_len) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(out, s_identity.sn, sn_len + 1);
    return ESP_OK;
}
