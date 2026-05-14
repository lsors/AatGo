#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "version.h"
#include "log_service.h"
#include "nvs_store.h"
#include "wifi_config.h"

static const char *TAG = "aatgo";

void app_main(void)
{
    log_service_init();
    ESP_LOGI(TAG, "AatGo firmware v%s", AATGO_FW_VERSION_STR);

    ESP_ERROR_CHECK(nvs_store_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(wifi_config_init());

    ESP_LOGI(TAG, "System ready");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
