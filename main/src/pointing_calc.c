#include "pointing_calc.h"
#include "servo_driver.h"
#include <math.h>
#include <stddef.h>

#define DEG2RAD  (M_PI / 180.0)
#define RAD2DEG  (180.0 / M_PI)
#define EARTH_R  6371000.0   /* metres */

/* Servo speed: 60° in 160 ms */
#define SERVO_DEG_PER_MS  (60.0f / 160.0f)

/* ---- 1-D Kalman filter ---- */
typedef struct {
    float x;  /* state estimate */
    float p;  /* estimate covariance */
    float q;  /* process noise */
    float r;  /* measurement noise */
} kalman1d_t;

static kalman1d_t s_kf_az = {.q = 0.1f, .r = 1.0f, .p = 1000.0f};
static kalman1d_t s_kf_el = {.q = 0.1f, .r = 1.0f, .p = 1000.0f};
static bool       s_kf_init = false;

static float kalman_update_az(kalman1d_t *k, float meas)
{
    k->p += k->q;
    float innov = meas - k->x;
    /* Wrap innovation to [-180, 180] to handle 0/360 boundary */
    while (innov >  180.0f) innov -= 360.0f;
    while (innov < -180.0f) innov += 360.0f;
    float gain = k->p / (k->p + k->r);
    k->x += gain * innov;
    k->p  = (1.0f - gain) * k->p;
    while (k->x <    0.0f) k->x += 360.0f;
    while (k->x >= 360.0f) k->x -= 360.0f;
    return k->x;
}

static float kalman_update_el(kalman1d_t *k, float meas)
{
    k->p += k->q;
    float gain = k->p / (k->p + k->r);
    k->x += gain * (meas - k->x);
    k->p  = (1.0f - gain) * k->p;
    return k->x;
}

void pointing_calc_reset(void)
{
    s_kf_az.p = 1000.0f;
    s_kf_el.p = 1000.0f;
    s_kf_init = false;
}

static void compute_az_el(const gps_coord_t *home, const gps_coord_t *tgt,
                          double *az_out, double *el_out)
{
    double lat1 = home->lat * DEG2RAD;
    double lat2 = tgt->lat  * DEG2RAD;
    double dlon = (tgt->lon - home->lon) * DEG2RAD;

    /* Azimuth (0=North, 90=East) */
    double x  = sin(dlon) * cos(lat2);
    double y  = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    double az = atan2(x, y) * RAD2DEG;
    if (az < 0) az += 360.0;
    *az_out = az;

    /* Haversine horizontal distance */
    double dlat = (tgt->lat - home->lat) * DEG2RAD;
    double a    = sin(dlat / 2) * sin(dlat / 2) +
                  cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double dist = EARTH_R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    /* Elevation */
    double alt_diff = (double)(tgt->alt - home->alt);
    double el = atan2(alt_diff, dist) * RAD2DEG;
    if (el < 0.0) el = 0.0;   /* clamp: don't track below horizon */
    if (el > 90.0) el = 90.0;
    *el_out = el;
}

void calc_pointing(const gps_coord_t *home,
                   const gps_coord_t *target,
                   float cur_az_servo,
                   float cur_el_servo,
                   servo_solution_t  *out)
{
    double az_geo, el_deg;
    compute_az_el(home, target, &az_geo, &el_deg);

    /* First-run: seed Kalman state with raw measurement */
    if (!s_kf_init) {
        s_kf_az.x = (float)az_geo;
        s_kf_el.x = (float)el_deg;
        s_kf_init = true;
    }

    float az_filt = kalman_update_az(&s_kf_az, (float)az_geo);
    float el_filt = kalman_update_el(&s_kf_el, (float)el_deg);

    /* ---- Solution A: direct (1:1 — servo angle equals geographic azimuth) ----
     * The 320° servo covers geo 0°..320° directly; the remaining 40° is the blind zone. */
    float az_A = az_filt;
    float el_A = el_filt;                          /* [0, 90] */

    /* ---- Solution B: flip (az ±180°, el = 180-el) ---- */
    float az_geo_B = (az_filt >= 180.0f) ? (az_filt - 180.0f) : (az_filt + 180.0f);
    float az_B     = az_geo_B;
    float el_B     = 180.0f - el_filt;             /* [90, 180] */

    /* Clamp both solutions to servo ranges */
    if (az_A < 0.0f) az_A = 0.0f;
    if (az_A > SERVO_AZ_RANGE_DEG) az_A = SERVO_AZ_RANGE_DEG;
    if (az_B < 0.0f) az_B = 0.0f;
    if (az_B > SERVO_AZ_RANGE_DEG) az_B = SERVO_AZ_RANGE_DEG;

    /* Select solution with minimum total servo movement */
    float cost_A = fabsf(cur_az_servo - az_A) + fabsf(cur_el_servo - el_A);
    float cost_B = fabsf(cur_az_servo - az_B) + fabsf(cur_el_servo - el_B);

    if (cost_B < cost_A) {
        out->az_servo = az_B;
        out->el_servo = el_B;
        out->is_flip  = true;
    } else {
        out->az_servo = az_A;
        out->el_servo = el_A;
        out->is_flip  = false;
    }
}

float pointing_transition_ms(float cur_az, float cur_el,
                              float tgt_az, float tgt_el)
{
    float t_az = fabsf(tgt_az - cur_az) / SERVO_DEG_PER_MS;
    float t_el = fabsf(tgt_el - cur_el) / SERVO_DEG_PER_MS;
    return (t_az > t_el) ? t_az : t_el;
}
