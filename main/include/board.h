#pragma once

/*
 * board.h — AatGo 硬件引脚与板级常量
 *
 * PCB 定板后在此统一修改，其他模块不再各自定义 GPIO。
 */

/* ---- 蜂鸣器 ---- */
#define BUZZER_GPIO         4

/* ---- 舵机 PWM ---- */
#define SERVO_AZ_GPIO       5      /* 方位舵机（底部，数字舵机，320°全量程使用，中位 160°） */
#define SERVO_EL_GPIO       6      /* 仰角舵机（顶部，180°） */
