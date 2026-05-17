#pragma once

#include "esp_err.h"
#include "config_store.h"
#include "board.h"

#define SERVO_AZ_RANGE_DEG  320.0f
#define SERVO_EL_RANGE_DEG  180.0f

typedef enum {
    SERVO_AZ = 0,
    SERVO_EL = 1,
} servo_id_t;

esp_err_t servo_driver_init(void);

/* Set servo to ideal angle with calibration compensation applied. */
void servo_set_angle(servo_id_t id, float ideal_deg);

/* Set servo to raw angle bypassing calibration (for web calibration page). */
void servo_set_raw(servo_id_t id, float raw_deg);

/* Read last commanded raw angle (after calibration). */
float servo_get_raw(servo_id_t id);

/* Apply new calibration (takes effect immediately). */
void servo_calib_update(const servo_calib_t *calib);
