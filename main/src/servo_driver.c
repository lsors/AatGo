#include "servo_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "servo_driver";

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_14_BIT   /* 16384 ticks */
#define LEDC_FREQ_HZ    333

#define PULSE_MIN_US    500
#define PULSE_MAX_US    2500

static const int    CH_GPIO[2]  = {SERVO_AZ_GPIO, SERVO_EL_GPIO};
static const float  CH_RANGE[2] = {SERVO_AZ_RANGE_DEG, SERVO_EL_RANGE_DEG};
static const ledc_channel_t CH_ID[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};

static float       s_raw_deg[2] = {150.0f, 90.0f};  /* last commanded raw angle */
static servo_calib_t s_calib    = {0};

/* Convert pulse width in µs to LEDC duty value */
static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    /* duty = pulse_us / period_us * 2^resolution
              = pulse_us * freq * 2^14 / 1e6            */
    return (uint32_t)(((uint64_t)pulse_us * LEDC_FREQ_HZ * (1 << LEDC_RESOLUTION)) / 1000000ULL);
}

static uint32_t angle_to_duty(float angle_deg, float range_deg)
{
    if (angle_deg < 0.0f)         angle_deg = 0.0f;
    if (angle_deg > range_deg)    angle_deg = range_deg;
    uint32_t pulse = (uint32_t)(PULSE_MIN_US +
                     (angle_deg / range_deg) * (PULSE_MAX_US - PULSE_MIN_US));
    return pulse_to_duty(pulse);
}

static void servo_write_raw(servo_id_t id, float raw_deg)
{
    if (raw_deg < 0.0f)            raw_deg = 0.0f;
    if (raw_deg > CH_RANGE[id])    raw_deg = CH_RANGE[id];
    s_raw_deg[id] = raw_deg;
    uint32_t duty = angle_to_duty(raw_deg, CH_RANGE[id]);
    ledc_set_duty(LEDC_MODE, CH_ID[id], duty);
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

    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_MODE,
            .channel    = CH_ID[i],
            .timer_sel  = LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = CH_GPIO[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) return err;
    }

    /* Move to neutral: AZ centre (150°), EL horizontal (0°) */
    servo_write_raw(SERVO_AZ, 150.0f);
    servo_write_raw(SERVO_EL,   0.0f);

    ESP_LOGI(TAG, "Servo driver ready — AZ GPIO%d  EL GPIO%d  @%d Hz",
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
