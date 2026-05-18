#include "espnow_rx.h"
#include "crsf_parser.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <stdio.h>

#define GPS_QUEUE_DEPTH 4

static const char   *TAG       = "espnow_rx";
static QueueHandle_t s_gps_queue;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int data_len)
{
    if (!data || data_len <= 0) return;

    /* ---- 打印每一个收到的 ESP-NOW 包 ---- */
    const uint8_t *mac = recv_info->src_addr;
    ESP_LOGI(TAG, "PKT from %02X:%02X:%02X:%02X:%02X:%02X  len=%d  rssi=%d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             data_len,
             recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0);

    /* 打印原始字节（十六进制），最多显示 32 字节防止刷屏 */
    int print_len = data_len > 32 ? 32 : data_len;
    char hex[32 * 3 + 1];
    for (int i = 0; i < print_len; i++) {
        snprintf(hex + i * 3, 4, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "RAW [%.*s%s]", print_len * 3, hex,
             data_len > 32 ? "..." : "");

    /* ---- 解析 CRSF GPS 帧 ---- */
    crsf_gps_frame_t frame;
    if (crsf_parse_gps(data, (uint16_t)data_len, &frame)) {
        ESP_LOGI(TAG, "GPS lat=%.7f lon=%.7f alt=%.1fm spd=%.1fkm/h hdg=%.1f° sats=%d",
                 frame.latitude  / 1e7,
                 frame.longitude / 1e7,
                 (float)frame.altitude - 1000.0f,
                 frame.groundspeed / 10.0f,
                 frame.heading / 100.0f,
                 frame.satellites);

        if (xQueueSend(s_gps_queue, &frame, 0) != pdTRUE) {
            ESP_LOGW(TAG, "GPS queue full, frame dropped");
        }
    } else {
        ESP_LOGW(TAG, "No valid CRSF GPS frame in packet");
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
