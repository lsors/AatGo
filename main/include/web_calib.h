#pragma once

#include "esp_err.h"

/* Start the HTTP calibration server (call after wifi_config_init). */
esp_err_t web_calib_init(void);
