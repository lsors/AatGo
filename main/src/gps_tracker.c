#include "gps_tracker.h"
#include "buzzer.h"
#include "esp_log.h"
#include <string.h>

#define HOME_SAMPLE_COUNT 10

static const char *TAG = "gps_tracker";

static struct {
    /* Home calibration accumulator */
    double lat_sum;
    double lon_sum;
    double alt_sum;
    int    sample_count;
    bool   home_ready;

    gps_coord_t home;
    gps_coord_t target;
} s;

void gps_tracker_init(void)
{
    memset(&s, 0, sizeof(s));
}

void gps_tracker_update(const crsf_gps_frame_t *frame)
{
    /* Ignore frames without a valid GPS fix (< 4 satellites) */
    if (frame->satellites < 4) return;

    double lat = (double)frame->latitude  / 1e7;
    double lon = (double)frame->longitude / 1e7;
    float  alt = (float)frame->altitude - 1000.0f;

    /* Always update the target (live drone position) */
    s.target.lat = lat;
    s.target.lon = lon;
    s.target.alt = alt;

    /* Home calibration: accumulate first HOME_SAMPLE_COUNT valid frames */
    if (!s.home_ready) {
        s.lat_sum    += lat;
        s.lon_sum    += lon;
        s.alt_sum    += alt;
        s.sample_count++;

        ESP_LOGI(TAG, "Home sample %d/%d  lat=%.7f lon=%.7f alt=%.1f sats=%d",
                 s.sample_count, HOME_SAMPLE_COUNT, lat, lon, alt, frame->satellites);

        if (s.sample_count >= HOME_SAMPLE_COUNT) {
            s.home.lat = s.lat_sum / HOME_SAMPLE_COUNT;
            s.home.lon = s.lon_sum / HOME_SAMPLE_COUNT;
            s.home.alt = (float)(s.alt_sum / HOME_SAMPLE_COUNT);
            s.home_ready = true;

            ESP_LOGI(TAG, "Home calibrated: lat=%.7f lon=%.7f alt=%.1f",
                     s.home.lat, s.home.lon, s.home.alt);

            buzzer_play(BUZZER_PATTERN_CALIBRATED);
        }
    }
}

bool gps_tracker_home_ready(void)
{
    return s.home_ready;
}

void gps_tracker_get_home(gps_coord_t *out)
{
    *out = s.home;
}

void gps_tracker_get_target(gps_coord_t *out)
{
    *out = s.target;
}

void gps_tracker_reset_home(void)
{
    s.lat_sum      = 0;
    s.lon_sum      = 0;
    s.alt_sum      = 0;
    s.sample_count = 0;
    s.home_ready   = false;
    ESP_LOGI(TAG, "Home reset, re-calibration required");
}
