#include "servo_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "servo_driver";

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_14_BIT  /* 16384 ticks */
#define LEDC_FREQ_HZ    333                /* digital servos support high refresh rate */

/* Common pulse floor */
#define PULSE_MIN_US    500

/* AZ: 320° digital servo, full range used.  500 µs = 0°,  2500 µs = 320°
 * EL: 180° servo, full 500..2500 µs range. */
#define AZ_PULSE_MAX_US 2500
#define EL_PULSE_MAX_US 2500

static const int          CH_GPIO[2]      = {SERVO_AZ_GPIO, SERVO_EL_GPIO};
static const float        CH_RANGE[2]     = {SERVO_AZ_RANGE_DEG, SERVO_EL_RANGE_DEG};
static const uint32_t     CH_PULSE_MAX[2] = {AZ_PULSE_MAX_US, EL_PULSE_MAX_US};
static const ledc_channel_t CH_ID[2]      = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};

static float         s_raw_deg[2] = {160.0f, 90.0f}; /* matches startup neutral */
static servo_calib_t s_calib      = {0};

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    /* duty = pulse_us / period_us × 2^14 = pulse_us × freq × 16384 / 1e6 */
    return (uint32_t)(((uint64_t)pulse_us * LEDC_FREQ_HZ * 16384u) / 1000000ULL);
}

static uint32_t angle_to_duty(servo_id_t id, float angle_deg)
{
    float range = CH_RANGE[id];
    if (angle_deg < 0.0f)    angle_deg = 0.0f;
    if (angle_deg > range)   angle_deg = range;
    uint32_t pulse = (uint32_t)(PULSE_MIN_US +
                     (angle_deg / range) * (CH_PULSE_MAX[id] - PULSE_MIN_US));
    return pulse_to_duty(pulse);
}

static void servo_write_raw(servo_id_t id, float raw_deg)
{
    float range = CH_RANGE[id];
    if (raw_deg < 0.0f)    raw_deg = 0.0f;
    if (raw_deg > range)   raw_deg = range;
    s_raw_deg[id] = raw_deg;
    ledc_set_duty(LEDC_MODE, CH_ID[id], angle_to_duty(id, raw_deg));
    ledc_update_duty(LEDC_MODE, CH_ID[id]);
}

esp_err_t servo_driver_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;

    /* Initialize each channel with the center duty directly so the very first
     * PWM pulse is already at neutral — avoids the twitch from duty=0. */
    static const float INIT_DEG[2] = {160.0f, 90.0f};
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_MODE,
            .channel    = CH_ID[i],
            .timer_sel  = LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = CH_GPIO[i],
            .duty       = angle_to_duty((servo_id_t)i, INIT_DEG[i]),
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) return err;
        s_raw_deg[i] = INIT_DEG[i];
    }

    ESP_LOGI(TAG, "Servo driver ready — AZ GPIO%d (320°)  EL GPIO%d (180°)  @%d Hz",
             SERVO_AZ_GPIO, SERVO_EL_GPIO, LEDC_FREQ_HZ);
    return ESP_OK;
}

static float apply_calib_az(float ideal)
{
    float out = s_calib.az_reversed ? (SERVO_AZ_RANGE_DEG - ideal) : ideal;
    out += s_calib.az_offset_deg;
    if (out < 0.0f)               out = 0.0f;
    if (out > SERVO_AZ_RANGE_DEG) out = SERVO_AZ_RANGE_DEG;
    return out;
}

static float apply_calib_el(float ideal)
{
    float out = s_calib.el_reversed ? (SERVO_EL_RANGE_DEG - ideal) : ideal;
    out += s_calib.el_offset_deg;
    if (out < 0.0f)               out = 0.0f;
    if (out > SERVO_EL_RANGE_DEG) out = SERVO_EL_RANGE_DEG;
    return out;
}

void servo_set_angle(servo_id_t id, float ideal_deg)
{
    float raw = (id == SERVO_AZ) ? apply_calib_az(ideal_deg)
                                 : apply_calib_el(ideal_deg);
    servo_write_raw(id, raw);
}

void servo_set_raw(servo_id_t id, float raw_deg)
{
    servo_write_raw(id, raw_deg);
}

float servo_get_raw(servo_id_t id)
{
    return s_raw_deg[id];
}

void servo_calib_update(const servo_calib_t *calib)
{
    s_calib = *calib;
}
