#include "espnow_rx.h"
#include "crsf_parser.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"

#define GPS_QUEUE_DEPTH 4

static const char   *TAG       = "espnow_rx";
static QueueHandle_t s_gps_queue;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int data_len)
{
    if (!data || data_len <= 0) return;

    crsf_gps_frame_t frame;
    if (crsf_parse_gps(data, (uint16_t)data_len, &frame)) {
        if (xQueueSend(s_gps_queue, &frame, 0) != pdTRUE) {
            ESP_LOGW(TAG, "GPS queue full, frame dropped");
        }
    }
}

esp_err_t espnow_rx_init(void)
{
    s_gps_queue = xQueueCreate(GPS_QUEUE_DEPTH, sizeof(crsf_gps_frame_t));
    if (!s_gps_queue) return ESP_ERR_NO_MEM;

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register recv_cb failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register broadcast peer so we receive from the ELRS TX module */
    esp_now_peer_info_t peer = {
        .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .channel   = 0,        /* 0 = use current WiFi channel */
        .ifidx     = WIFI_IF_AP,
        .encrypt   = false,
    };
    esp_now_add_peer(&peer);

    ESP_LOGI(TAG, "ESP-NOW receiver ready");
    return ESP_OK;
}

QueueHandle_t espnow_rx_get_queue(void)
{
    return s_gps_queue;
}
