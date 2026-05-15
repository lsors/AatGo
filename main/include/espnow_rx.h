#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Must be called after wifi_config_init(). */
esp_err_t     espnow_rx_init(void);

/* Returns the queue that receives crsf_gps_frame_t items. */
QueueHandle_t espnow_rx_get_queue(void);
