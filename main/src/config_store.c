#include "config_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG       = "config_store";
static const char *NVS_NS    = "aatgo";
static const char *KEY_CALIB = "servo_calib";

void config_reset(servo_calib_t *calib)
{
    memset(calib, 0, sizeof(*calib));
}

esp_err_t config_load(servo_calib_t *calib)
{
    config_reset(calib);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved calibration, using defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    size_t sz = sizeof(*calib);
    err = nvs_get_blob(h, KEY_CALIB, calib, &sz);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config_reset(calib);
        err = ESP_OK;
    } else if (sz != sizeof(*calib)) {
        /* Struct size mismatch (firmware upgrade) — reset to safe defaults */
        config_reset(calib);
        err = ESP_OK;
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded calib: az_off=%.2f el_off=%.2f az_rev=%d el_rev=%d",
                 calib->az_offset_deg, calib->el_offset_deg,
                 calib->az_reversed, calib->el_reversed);
    }
    return err;
}

esp_err_t config_save(const servo_calib_t *calib)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, KEY_CALIB, calib, sizeof(*calib));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved calib: az_off=%.2f el_off=%.2f az_rev=%d el_rev=%d",
                 calib->az_offset_deg, calib->el_offset_deg,
                 calib->az_reversed, calib->el_reversed);
    }
    return err;
}
