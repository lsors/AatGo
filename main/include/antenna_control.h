#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* Create and start the tracking FreeRTOS task. */
esp_err_t antenna_control_start(void);

/* Pause / resume tracking (used by web calibration page). */
void antenna_control_pause(bool pause);
bool antenna_control_is_paused(void);
