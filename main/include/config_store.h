#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float az_offset_deg;
    float el_offset_deg;
    bool  az_reversed;
    bool  el_reversed;
} servo_calib_t;

esp_err_t config_load(servo_calib_t *calib);
esp_err_t config_save(const servo_calib_t *calib);
void      config_reset(servo_calib_t *calib);
