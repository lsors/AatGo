#include "log_service.h"
#include "esp_log.h"

void log_service_init(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("aatgo", ESP_LOG_DEBUG);
}
