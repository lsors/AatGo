#include "antenna_control.h"
#include "espnow_rx.h"
#include "gps_tracker.h"
#include "pointing_calc.h"
#include "servo_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TASK_STACK    8192
#define TASK_PRIORITY 5
#define SIGNAL_TIMEOUT_MS 3000
#define TRANSITION_MARGIN_MS 20

static const char *TAG = "antenna_ctrl";

static volatile bool s_paused = false;

static void tracking_task(void *arg)
{
    QueueHandle_t q    = espnow_rx_get_queue();
    float         cur_az = 150.0f;   /* start at mechanical centre */
    float         cur_el =   0.0f;

    ESP_LOGI(TAG, "Tracking task started, waiting for home calibration...");

    /* Wait until GPS home coordinate is ready */
    while (!gps_tracker_home_ready()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Home ready — tracking active");
    pointing_calc_reset();

    for (;;) {
        if (s_paused) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        crsf_gps_frame_t frame;
        BaseType_t got = xQueueReceive(q, &frame, pdMS_TO_TICKS(SIGNAL_TIMEOUT_MS));

        if (got != pdTRUE) {
            /* Signal loss — hold position */
            ESP_LOGW(TAG, "GPS signal lost, holding position");
            continue;
        }

        if (s_paused) continue;  /* Became paused while waiting */

        gps_tracker_update(&frame);

        if (!gps_tracker_home_ready()) continue;  /* Still calibrating */

        gps_coord_t home, target;
        gps_tracker_get_home(&home);
        gps_tracker_get_target(&target);

        servo_solution_t sol;
        calc_pointing(&home, &target, cur_az, cur_el, &sol);

        float trans_ms = pointing_transition_ms(cur_az, cur_el,
                                                sol.az_servo, sol.el_servo);

        ESP_LOGD(TAG, "az=%.1f el=%.1f flip=%d trans=%.0fms",
                 sol.az_servo, sol.el_servo, sol.is_flip, trans_ms);

        servo_set_angle(SERVO_AZ, sol.az_servo);
        servo_set_angle(SERVO_EL, sol.el_servo);

        cur_az = sol.az_servo;
        cur_el = sol.el_servo;

        /* During large transitions, pause new commands until servos arrive */
        if (trans_ms > 80.0f) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(trans_ms + TRANSITION_MARGIN_MS)));
        }
    }
}

esp_err_t antenna_control_start(void)
{
    BaseType_t ret = xTaskCreate(tracking_task, "ant_ctrl",
                                 TASK_STACK, NULL, TASK_PRIORITY, NULL);
    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void antenna_control_pause(bool pause)
{
    if (pause && !s_paused) {
        s_paused = true;
        ESP_LOGI(TAG, "Tracking paused (calibration mode)");
    } else if (!pause && s_paused) {
        s_paused = false;
        pointing_calc_reset();   /* Reset Kalman so first update is clean */
        ESP_LOGI(TAG, "Tracking resumed");
    }
}

bool antenna_control_is_paused(void)
{
    return s_paused;
}
