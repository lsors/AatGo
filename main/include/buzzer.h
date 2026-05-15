#pragma once

/* GPIO pin — assign after PCB is finalised */
#define BUZZER_GPIO  2

typedef enum {
    BUZZER_PATTERN_CALIBRATED = 0,  /* 滴滴两短声：坐标标定完成 */
    BUZZER_PATTERN_WARN,            /* 单长声：预留警告 */
    BUZZER_PATTERN_MAX
} buzzer_pattern_t;

void buzzer_init(void);
void buzzer_play(buzzer_pattern_t pattern);
