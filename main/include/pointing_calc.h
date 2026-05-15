#pragma once

#include "gps_tracker.h"

typedef struct {
    float az_servo;  /* azimuth servo angle  [0°, 300°] */
    float el_servo;  /* elevation servo angle [0°, 180°] */
    bool  is_flip;   /* true = over-the-top flip solution */
} servo_solution_t;

/* Reset Kalman filter state (call when resuming after calibration pause). */
void pointing_calc_reset(void);

/* Compute optimal servo solution.
   cur_az / cur_el: current servo positions used for cost comparison. */
void calc_pointing(const gps_coord_t *home,
                   const gps_coord_t *target,
                   float cur_az_servo,
                   float cur_el_servo,
                   servo_solution_t  *out);

/* Transition time (ms) from current to target servo position. */
float pointing_transition_ms(float cur_az, float cur_el,
                              float tgt_az, float tgt_el);
