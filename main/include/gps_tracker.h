#pragma once

#include "crsf_parser.h"
#include <stdbool.h>

typedef struct {
    double lat;   /* degrees */
    double lon;   /* degrees */
    float  alt;   /* metres */
} gps_coord_t;

void gps_tracker_init(void);

/* Feed a parsed CRSF GPS frame.
   While home is not ready, accumulates samples and averages on the 10th. */
void gps_tracker_update(const crsf_gps_frame_t *frame);

bool            gps_tracker_home_ready(void);
void            gps_tracker_get_home(gps_coord_t *out);
void            gps_tracker_get_target(gps_coord_t *out);

/* Force re-calibration (discard accumulated samples). */
void            gps_tracker_reset_home(void);
