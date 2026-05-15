#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_GPS      0x02
#define CRSF_GPS_PAYLOAD_LEN    15

typedef struct {
    int32_t  latitude;     /* degrees × 1e7 */
    int32_t  longitude;    /* degrees × 1e7 */
    uint16_t groundspeed;  /* km/h × 10 */
    uint16_t heading;      /* degrees × 100 */
    uint16_t altitude;     /* metres + 1000 offset */
    uint8_t  satellites;
} crsf_gps_frame_t;

/* Scan buf for a valid CRSF GPS frame; returns true and fills *out on success. */
bool crsf_parse_gps(const uint8_t *buf, uint16_t len, crsf_gps_frame_t *out);
