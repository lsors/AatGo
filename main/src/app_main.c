#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "version.h"
#include "log_service.h"
#include "nvs_store.h"
#include "wifi_config.h"
#include "buzzer.h"
#include "gps_tracker.h"
#include "espnow_rx.h"
#include "servo_driver.h"
#include "antenna_control.h"
#include "web_calib.h"

static const char *TAG = "aatgo";

void app_main(void)
{
    log_service_init();
    ESP_LOGI(TAG, "AatGo firmware v%s", AATGO_FW_VERSION_STR);

    ESP_ERROR_CHECK(nvs_store_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    buzzer_init();
    gps_tracker_init();

    /* WiFi SoftAP must start before ESP-NOW */
    ESP_ERROR_CHECK(wifi_config_init());
    ESP_ERROR_CHECK(espnow_rx_init());

    ESP_ERROR_CHECK(servo_driver_init());
    ESP_ERROR_CHECK(web_calib_init());
    ESP_ERROR_CHECK(antenna_control_start());

    ESP_LOGI(TAG, "System ready — connect to '%s' for calibration", AATGO_WIFI_SSID);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
