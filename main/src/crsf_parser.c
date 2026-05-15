#include "crsf_parser.h"
#include <string.h>

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    return crc;
}

bool crsf_parse_gps(const uint8_t *buf, uint16_t len, crsf_gps_frame_t *out)
{
    /* Scan buffer for a valid GPS frame */
    for (uint16_t i = 0; i + 4 <= len; i++) {
        if (buf[i] != CRSF_SYNC_BYTE) continue;

        uint8_t frame_len = buf[i + 1];
        /* frame_len = type(1) + payload(N) + crc(1); minimum meaningful = 3 */
        if (frame_len < 3) continue;
        if ((uint16_t)(i + 1 + frame_len) >= len) continue;

        uint8_t type = buf[i + 2];
        if (type != CRSF_FRAMETYPE_GPS) continue;

        uint8_t payload_len = frame_len - 2; /* subtract type and crc */
        if (payload_len != CRSF_GPS_PAYLOAD_LEN) continue;

        /* Verify CRC (covers type + payload, i.e. frame_len-1 bytes) */
        uint8_t crc_calc = crsf_crc8(&buf[i + 2], frame_len - 1);
        uint8_t crc_recv = buf[i + 1 + frame_len];
        if (crc_calc != crc_recv) continue;

        /* Extract big-endian fields */
        const uint8_t *p = &buf[i + 3];
        out->latitude    = (int32_t) (((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                                      ((uint32_t)p[2] <<  8) |  (uint32_t)p[3]);
        out->longitude   = (int32_t) (((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) |
                                      ((uint32_t)p[6] <<  8) |  (uint32_t)p[7]);
        out->groundspeed = (uint16_t)(((uint16_t)p[8]  << 8) |  p[9]);
        out->heading     = (uint16_t)(((uint16_t)p[10] << 8) |  p[11]);
        out->altitude    = (uint16_t)(((uint16_t)p[12] << 8) |  p[13]);
        out->satellites  = p[14];
        return true;
    }
    return false;
}
