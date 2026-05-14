#pragma once

#include "esp_err.h"

#define AATGO_WIFI_SSID     "AatGo-AP"
#define AATGO_WIFI_PASS     "aatgo1234"
#define AATGO_WIFI_CHANNEL  1
#define AATGO_WIFI_MAX_CONN 4

esp_err_t wifi_config_init(void);
void      wifi_config_deinit(void);
