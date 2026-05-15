#include "buzzer.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "buzzer";

typedef struct {
    uint32_t on_ms;
    uint32_t off_ms;
    uint8_t  repeat;
} buzzer_beat_t;

static const buzzer_beat_t PATTERNS[BUZZER_PATTERN_MAX] = {
    [BUZZER_PATTERN_CALIBRATED] = {.on_ms = 120, .off_ms = 120, .repeat = 2},
    [BUZZER_PATTERN_WARN]       = {.on_ms = 600, .off_ms =   0, .repeat = 1},
};

typedef struct {
    const buzzer_beat_t *beat;
    uint8_t  remaining;
    bool     pin_on;
} buzzer_state_t;

static esp_timer_handle_t s_timer;
static buzzer_state_t     s_state;

static void buzzer_set(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? 1 : 0);
}

static void timer_cb(void *arg)
{
    buzzer_state_t *st = (buzzer_state_t *)arg;

    if (st->pin_on) {
        buzzer_set(false);
        st->pin_on = false;
        if (--st->remaining == 0) return;
        if (st->beat->off_ms > 0) {
            esp_timer_start_once(s_timer, (uint64_t)st->beat->off_ms * 1000);
        } else {
            /* No off gap — done */
        }
    } else {
        buzzer_set(true);
        st->pin_on = true;
        esp_timer_start_once(s_timer, (uint64_t)st->beat->on_ms * 1000);
    }
}

void buzzer_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    buzzer_set(false);

    esp_timer_create_args_t ta = {
        .callback = timer_cb,
        .arg      = &s_state,
        .name     = "buzzer",
    };
    esp_timer_create(&ta, &s_timer);
    ESP_LOGI(TAG, "Buzzer ready on GPIO %d", BUZZER_GPIO);
}

void buzzer_play(buzzer_pattern_t pattern)
{
    if (pattern >= BUZZER_PATTERN_MAX) return;

    esp_timer_stop(s_timer);
    buzzer_set(false);

    s_state.beat      = &PATTERNS[pattern];
    s_state.remaining = PATTERNS[pattern].repeat;
    s_state.pin_on    = false;

    /* Kick off first ON pulse */
    buzzer_set(true);
    s_state.pin_on = true;
    esp_timer_start_once(s_timer, (uint64_t)s_state.beat->on_ms * 1000);
}
