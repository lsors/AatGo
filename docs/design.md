# AatGo 天线自动跟踪系统设计文档

> ESP32-C6 · ESP-IDF · 双轴舵机 · ESP-NOW + CRSF

---

## 1. 项目背景

通过控制两个舵机驱动平板天线，使天线始终对准穿越机方向，实现自动跟踪。

- 穿越机 GPS 遥测数据由 ELRS 高频头（TX 模块）通过 **ESP-NOW** 协议广播出来
- 地面站 **ESP32-C6**（AatGo）接收 ESP-NOW 数据包，解析 CRSF 协议获取飞机实时 GPS 坐标
- 根据地面站自身坐标与飞机坐标，计算方位角和仰角，驱动两个舵机转动

---

## 2. 硬件组成

| 部件 | 说明 |
|------|------|
| ESP32-C6 开发板 | 主控，运行 AatGo 固件 |
| ELRS 高频头（TX 模块） | 接收穿越机遥测，通过 ESP-NOW 广播 |
| 底部舵机（300°） | 绕垂直轴旋转，控制方位角（Azimuth） |
| 顶部舵机（180°） | 绕水平轴俯仰，控制仰角（Elevation） |
| 平板天线 | 安装在顶部舵机上，指向飞机 |

**机械结构：**
```
       [ 平板天线 ]
            │
       [ 仰角舵机 180° ]   ← 顶部，控制上下俯仰
            │
       [ 方位舵机 300° ]   ← 底部，控制左右旋转
            │
          底座
```

---

## 3. 系统整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                         数据流                               │
│                                                             │
│  穿越机 GPS                                                  │
│     │  (ELRS 遥测, CRSF 协议)                               │
│     ▼                                                       │
│  ELRS 高频头 (TX 模块)                                       │
│     │  (ESP-NOW 广播)                                       │
│     ▼                                                       │
│  ESP32-C6 (AatGo)                                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  espnow_rx  →  crsf_parser  →  gps_tracker           │  │
│  │                    pointing_calc (方位角 + 仰角)       │  │
│  │                    antenna_control (跟踪主循环)        │  │
│  │                    servo_driver (PWM 输出)            │  │
│  └──────────────────────────────────────────────────────┘  │
│         │                             │                     │
│   方位舵机 (300°)               仰角舵机 (180°)             │
│   底部 / 水平转台                 顶部 / 俯仰轴              │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 软件模块划分

```
main/
├── include/
│   ├── espnow_rx.h          # ESP-NOW 接收层
│   ├── crsf_parser.h        # CRSF 遥测帧解析
│   ├── gps_tracker.h        # 飞机 GPS 状态维护 + 本机坐标标定
│   ├── pointing_calc.h      # 方位角 / 仰角计算 + 双解选优
│   ├── servo_driver.h       # PWM 舵机驱动 + 标定补偿
│   ├── antenna_control.h    # 跟踪主循环 + 过渡时间管理
│   ├── buzzer.h             # 蜂鸣器驱动（GPIO + 软件定时器）
│   ├── web_calib.h          # HTTP 标定页面服务
│   ├── config_store.h       # NVS 配置持久化
│   ├── wifi_config.h        # WiFi AP
│   ├── nvs_store.h          # NVS 初始化
│   └── log_service.h        # 日志
└── src/
    ├── app_main.c
    ├── espnow_rx.c
    ├── crsf_parser.c
    ├── gps_tracker.c
    ├── pointing_calc.c
    ├── servo_driver.c
    ├── antenna_control.c
    ├── buzzer.c
    ├── web_calib.c
    └── ...（已有模块）
```

---

## 5. 各模块职责

| 模块 | 职责 | 关键接口 |
|------|------|---------|
| `espnow_rx` | 初始化 ESP-NOW，注册接收回调，将原始包推入 FreeRTOS 队列 | `espnow_rx_init()` |
| `crsf_parser` | 从 ESP-NOW payload 中识别并解析裸 CRSF 帧，提取 GPS 帧字段 | `crsf_parse(buf, len, &frame)` |
| `gps_tracker` | 维护飞机实时坐标；本机坐标标定（前 10 次有效帧均值）；丢失检测 | `gps_tracker_update()` / `gps_tracker_get_target()` / `gps_tracker_home_ready()` |
| `pointing_calc` | Haversine 方位角 + 仰角；双解计算；选最优解；卡尔曼滤波 | `calc_pointing(home, target, cur, &sol)` |
| `servo_driver` | LEDC PWM 输出（333 Hz），角度→脉宽映射，行程限位，施加标定补偿 | `servo_set_angle(id, deg)` / `servo_set_raw(id, deg)` |
| `antenna_control` | 跟踪主循环；过渡时间管理；信号丢失保持；标定模式暂停 | `antenna_control_task()` / `antenna_control_pause()` |
| `buzzer` | GPIO 驱动蜂鸣器，软件定时器实现节拍模式 | `buzzer_init()` / `buzzer_play(pattern)` |
| `web_calib` | HTTP 服务；提供标定页面；接收手动控制和标定保存指令 | `web_calib_init()` |
| `config_store` | NVS 读写：本机坐标、舵机标定偏移、方向反转标志 | `config_load()` / `config_save()` |

---

## 6. 关键算法

### 6.1 方位角计算（→ 底部 300° 舵机）

```c
// 输入：本机坐标 (home_lat, home_lon)，目标坐标 (tgt_lat, tgt_lon)，单位：度
double dlon = tgt_lon - home_lon;
double dlat = tgt_lat - home_lat;
double x = sin(dlon * DEG2RAD) * cos(tgt_lat * DEG2RAD);
double y = cos(home_lat * DEG2RAD) * sin(tgt_lat * DEG2RAD)
         - sin(home_lat * DEG2RAD) * cos(tgt_lat * DEG2RAD) * cos(dlon * DEG2RAD);
double azimuth = atan2(x, y) * RAD2DEG;  // [-180, 180]
if (azimuth < 0) azimuth += 360.0;       // 转换为 [0, 360]
```

### 6.2 仰角计算（→ 顶部 180° 舵机）

```c
// horiz_dist: Haversine 水平距离 (m)
// alt_diff  : 飞机高度 - 地面站高度 (m)
double elevation = atan2(alt_diff, horiz_dist) * RAD2DEG;  // [-90, 90]
```

### 6.3 双解模型与最优解选取

任意目标方向存在两组舵机解，几何上等价：

```
解 A（正向）：  az_servo = Az × (300/360)
               el_servo = El              （0° = 水平朝前，90° = 垂直朝上）

解 B（翻转）：  az_servo = (Az ± 180°) × (300/360)   ← 方位转到对侧
               el_servo = 180° - El                  ← 俯仰越过顶部翻转
```

**几何解释：**
- 天线正向朝前（El = 0°→90°），俯仰舵机从水平转到垂直
- 翻转后（El = 90°→180°），方位指向对侧，俯仰继续越过顶部，天线等效指向同一目标

**最优解判据（最小总转动量）：**
```c
float cost_A = fabsf(cur_az_servo - target_az_A) + fabsf(cur_el_servo - target_el_A);
float cost_B = fabsf(cur_az_servo - target_az_B) + fabsf(cur_el_servo - target_el_B);
use_flip = (cost_B < cost_A);
```

**盲区处理（Az ∈ [300°, 360°]）：**
- 解 A 的 az_servo 超出 [0°, 300°]，无效
- 强制使用解 B：az_servo = (Az - 180°) × (300/360)，el_servo = 180° - El
- 此时盲区跳变为"翻转过顶"，跳变过程时间可计算（见 §6.4）

### 6.4 舵机角度映射

```c
// 方位舵机（300°）
// az ∈ [0°, 360°)，解 A：az_A ∈ [0°, 300°] 有效，解 B：az_B = az ± 180°
float servo_az_from_geo(float az_deg) {
    return az_deg * (300.0f / 360.0f);  // [0°, 300°]
}

// 仰角舵机（180°）
// 解 A：el ∈ [0°, 90°]  → servo ∈ [0°, 180°]（水平到垂直）
// 解 B：el ∈ [0°, 90°]  → servo = 180° - el ∈ [180°, 90°]（越顶翻转）
// 低仰角 el < 0°（俯角）最大支持到 -10°，解 A 仅向下延伸
float servo_el_from_real(float el_deg, bool flip) {
    if (!flip) return el_deg + 10.0f;          // [-10°, 90°] → [0°, 100°]... 
    else       return (180.0f - el_deg);        // 翻转解
}
// 脉宽统一由 angle_to_pulse_us(servo_deg, 180.0f) 映射
```

**电气零点定义：**
- 仰角舵机 servo=0° → 脉宽 500 µs → 天线水平朝前
- 仰角舵机 servo=90° → 脉宽 1500 µs → 天线垂直朝上
- 仰角舵机 servo=180° → 脉宽 2500 µs → 天线水平朝后（翻转解最大位置）

### 6.5 跳变过渡时间计算

```c
// 舵机速度：0.16 s / 60° = 2.667 °/ms
#define SERVO_DEG_PER_MS  (60.0f / 160.0f)   // ≈ 0.375 °/ms

// 从当前位置到目标位置的过渡时间（取两轴最大值）
float transition_ms(float cur_az, float cur_el, float tgt_az, float tgt_el) {
    float t_az = fabsf(tgt_az - cur_az) / SERVO_DEG_PER_MS;
    float t_el = fabsf(tgt_el - cur_el) / SERVO_DEG_PER_MS;
    return fmaxf(t_az, t_el);
}
```

**过渡期行为：**
1. 计算当前解与目标解（含翻转）的 cost，选最优解
2. 发送目标位置到舵机（一次性写入，舵机自行运动）
3. `vTaskDelay(pdMS_TO_TICKS((uint32_t)transition_ms + 20))`  ← 多留 20 ms 余量
4. 延迟结束后恢复正常 1 Hz 卡尔曼更新跟踪循环

---

## 7. 启动流程

```
app_main()
  ├── log_service_init()
  ├── nvs_store_init()
  ├── config_load()              ← 读本机坐标、舵机校准值
  ├── esp_event_loop_create_default()
  ├── esp_netif_init()
  ├── espnow_rx_init()           ← 开始监听高频头 ESP-NOW 广播
  ├── servo_driver_init()        ← 初始化 LEDC PWM，舵机归零点
  ├── wifi_config_init()         ← SoftAP，用于配置本机坐标等参数
  └── antenna_control_task()     ← 主跟踪任务（FreeRTOS）
        loop:
          ├── xQueueReceive(gps_queue)   ← 等待新 GPS 数据
          ├── gps_tracker_update(frame)
          ├── calc_pointing(home, target, &az, &el)
          ├── servo_set_angle(SERVO_AZ, az)
          └── servo_set_angle(SERVO_EL, el)
```

---

## 8. CRSF GPS 帧格式（参考）

ELRS 遥测使用 CRSF 协议，GPS 帧 `CRSF_FRAMETYPE_GPS = 0x02`：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| latitude | int32 | 度 × 1e7 | 纬度 |
| longitude | int32 | 度 × 1e7 | 经度 |
| groundspeed | uint16 | km/h × 10 | 地速 |
| heading | uint16 | 度 × 100 | 航向 |
| altitude | uint16 | m + 1000 偏移 | 高度 |
| satellites | uint8 | 颗 | 卫星数 |

---

## 9. 硬件与行为参数（已确认）

| # | 参数 | 确认值 | 影响模块 |
|---|------|--------|---------|
| 1 | ESP-NOW payload 格式 | 裸 CRSF 帧，无额外封装 | `espnow_rx` / `crsf_parser` |
| 2 | 本机坐标来源 | 从 CRSF GPS 帧提取，前 10 次有效坐标平均值；确定后蜂鸣器发出滴滴声 | `gps_tracker` / `buzzer` |
| 3 | 方位舵机安装与盲区 | 中位朝正南（生产）/ 朝机头（测试）；60° 盲区朝正北；见 §10 标定流程 | `antenna_control` |
| 4 | 舵机规格 | 333 Hz 数字舵机，脉宽 500~2500 µs；GPIO 用宏 `SERVO_AZ_GPIO` / `SERVO_EL_GPIO` 占位 | `servo_driver` |
| 5 | 信号丢失行为 | 保持当前位置，等待下次 GPS 更新；禁止归中和扫描 | `antenna_control` |
| 6 | 平滑滤波方案 | 卡尔曼滤波；ELRS 上行速率约 1 Hz，以此为更新节拍 | `pointing_calc` / `antenna_control` |

---

## 10. 现场标定流程

> 每次出飞前执行一次，约 2~3 分钟。

```
步骤 1 — 穿越机 GPS 预热
  ├── 穿越机放在地面，等待 ELRS 高频头开始转发遥测
  ├── AatGo 接收 CRSF GPS 帧，累计有效坐标
  └── 当连续 10 次有效坐标采集完成：
        → 计算平均值 → 存入 home_lat / home_lon / home_alt（NVS）
        → 蜂鸣器发出 "滴滴" 声（标定完成提示）

步骤 2 — 地面站就位
  ├── 听到滴滴声后，穿越机可以移开
  ├── 将 AatGo 放到穿越机原来的位置
  │     ← 此时 home 坐标即为天线架设坐标，误差 < 穿越机尺寸
  └── 用指南针将方位舵机中位对准正南方向

步骤 3 — 跟踪就绪
  └── 移动穿越机即可触发实时跟踪
```

**关键约束：**
- 方位舵机中位 = 正南，60° 盲区朝正北（后方），正常飞行方向 ≠ 正北即可
- 仰角舵机中位朝上，垂直地面；天线随仰角舵机俯仰
- 测试时"中位朝机头"替代"正南"，逻辑相同，方向参考改变

---

## 11. 舵机驱动参数

```c
// servo_driver.h 中定义，GPIO 待硬件确认后替换宏值
#define SERVO_AZ_GPIO       18          // 方位舵机 GPIO（占位）
#define SERVO_EL_GPIO       19          // 仰角舵机 GPIO（占位）

#define SERVO_FREQ_HZ       333         // 数字舵机 333 Hz
#define SERVO_PERIOD_US     (1000000 / SERVO_FREQ_HZ)   // ≈ 3003 µs

#define SERVO_PULSE_MIN_US  500         // 最小脉宽
#define SERVO_PULSE_MAX_US  2500        // 最大脉宽

// 方位舵机（300°）
#define SERVO_AZ_RANGE_DEG  300
#define SERVO_AZ_BLIND_DEG  60          // 盲区，朝正北

// 仰角舵机（180°）
#define SERVO_EL_RANGE_DEG  180
#define SERVO_EL_MIN_DEG    (-10)       // 对应 500 µs
#define SERVO_EL_MAX_DEG    90          // 对应 2500 µs
```

**脉宽映射公式：**
```c
// angle_deg: 舵机物理角度 [0, range]
uint32_t angle_to_pulse_us(float angle_deg, float range_deg) {
    return (uint32_t)(SERVO_PULSE_MIN_US
        + (angle_deg / range_deg) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US));
}
```

---

## 12. 舵机安装标定系统

### 12.1 标定参数模型

舵机安装时无法做到机械精确对中，需要软件补偿。每个舵机有三个标定参数：

```c
// config_store.h 中的标定结构体
typedef struct {
    float az_offset_deg;    // 方位舵机零点偏移（正值=顺时针偏移）
    float el_offset_deg;    // 仰角舵机零点偏移（正值=偏高）
    bool  az_reversed;      // 方位舵机方向是否反装
    bool  el_reversed;      // 仰角舵机方向是否反装
} servo_calib_t;
```

**补偿应用位置：** 在 `servo_driver` 内部，`servo_set_angle()` 调用时施加，上层模块传入的永远是"理想角度"：

```c
// servo_driver.c
static float apply_calib_az(float ideal_deg) {
    float out = calib.az_reversed
                ? (SERVO_AZ_RANGE_DEG - ideal_deg)   // 方向翻转
                : ideal_deg;
    out += calib.az_offset_deg;                       // 零点偏移
    return clamp(out, 0.0f, SERVO_AZ_RANGE_DEG);
}

static float apply_calib_el(float ideal_deg) {
    float out = calib.el_reversed
                ? (SERVO_EL_RANGE_DEG - ideal_deg)
                : ideal_deg;
    out += calib.el_offset_deg;
    return clamp(out, 0.0f, SERVO_EL_RANGE_DEG);
}
```

新增 `servo_set_raw(id, deg)` 接口供标定页面直接驱动原始角度，绕过标定补偿：

```c
void servo_set_angle(servo_id_t id, float ideal_deg);  // 施加标定，正常跟踪用
void servo_set_raw(servo_id_t id, float raw_deg);       // 绕过标定，标定页面专用
```

---

### 12.2 Web 标定页面（`web_calib`）

#### HTTP 接口

| Method | Path | 说明 |
|--------|------|------|
| GET | `/` | 返回标定页面 HTML |
| GET | `/api/status` | JSON：当前舵机原始角度、标定参数、跟踪状态 |
| POST | `/api/servo` | 手动驱动舵机到指定原始角度 |
| POST | `/api/calib/save` | 保存标定参数到 NVS |
| POST | `/api/calib/reset` | 清零所有标定参数 |
| POST | `/api/track/pause` | 暂停/恢复跟踪任务 |

**`POST /api/servo` body（JSON）：**
```json
{ "az": 150.0, "el": 90.0 }
```

**`POST /api/calib/save` body（JSON）：**
```json
{
  "az_offset": -3.5,
  "el_offset":  2.0,
  "az_reversed": false,
  "el_reversed": false
}
```

#### 标定页面 UI 布局

```
┌─────────────────────────────────────────────────┐
│  AatGo 舵机标定                    [暂停跟踪 ●]  │
├─────────────────────────────────────────────────┤
│  方位舵机（AZ）                                  │
│  当前原始角度：[ 148.5 °]                         │
│                                                  │
│  手动控制：  [◀◀ -10°] [◀ -1°] [▶ +1°] [▶▶ +10°] │
│  快速定位：  [归中 150°]  [最小 0°]  [最大 300°]  │
│                                                  │
│  偏移校正：  offset [ -1.5 ] °   [方向反转 □]    │
├─────────────────────────────────────────────────┤
│  仰角舵机（EL）                                  │
│  当前原始角度：[  91.0 °]                         │
│                                                  │
│  手动控制：  [◀◀ -10°] [◀ -1°] [▶ +1°] [▶▶ +10°] │
│  快速定位：  [水平 0°]  [垂直 90°]  [最大 180°]   │
│                                                  │
│  偏移校正：  offset [  1.0 ] °   [方向反转 □]    │
├─────────────────────────────────────────────────┤
│              [保存标定]      [重置标定]            │
└─────────────────────────────────────────────────┘
```

#### 标定操作步骤（页面内说明文字）

```
方位舵机标定：
  1. 点击"暂停跟踪"，进入手动模式
  2. 点击"归中 150°"让舵机到机械中位
  3. 用指南针/参照物确认天线实际朝向
  4. 用 ±1° / ±10° 按钮微调，直到天线正对参考方向（正南或机头）
  5. 记录此时原始角度，计算 offset = 150° - 当前角度，填入偏移框
     （或直接点击按钮自动计算：当前位置设为中位）

仰角舵机标定：
  1. 点击"水平 0°"让舵机到水平位置
  2. 用水平尺确认天线是否真正水平
  3. 微调直到真正水平，同法计算 offset
  4. 点击"垂直 90°"验证垂直时是否准确

完成后点击"保存标定"写入 NVS，重启后自动加载。
```

---

### 12.3 标定模式与跟踪模式切换

标定期间必须暂停跟踪任务，防止跟踪覆盖手动控制：

```c
// antenna_control.h
void antenna_control_pause(bool pause);   // true=暂停，false=恢复
bool antenna_control_is_paused(void);
```

状态机：
```
TRACKING ──[/api/track/pause]──▶ CALIBRATING
              (暂停 FreeRTOS 任务通知)
CALIBRATING ──[/api/track/pause]──▶ TRACKING
              (发送恢复通知，重新从当前位置开始卡尔曼)
```

恢复跟踪时重置卡尔曼滤波器初始状态，避免从标定位置产生大幅跳转。

---

## 13. 蜂鸣器驱动设计

### 12.1 硬件接口

```c
// buzzer.h
#define BUZZER_GPIO   XX   // 待定，占位宏

// 蜂鸣器类型：有源蜂鸣器（GPIO 高电平响）
// 若使用无源蜂鸣器，需改为 LEDC 输出固定频率方波（推荐 2700 Hz）
```

### 12.2 节拍模式定义

```c
typedef enum {
    BUZZER_PATTERN_CALIBRATED,   // 标定完成：滴滴（两短声）
    BUZZER_PATTERN_WARN,         // 预留：单长声（信号丢失警告）
} buzzer_pattern_t;

// 节拍表（ON ms, OFF ms, 重复次数）
static const buzzer_beat_t PATTERNS[] = {
    [BUZZER_PATTERN_CALIBRATED] = {.on_ms = 100, .off_ms = 100, .repeat = 2},
    [BUZZER_PATTERN_WARN]       = {.on_ms = 500, .off_ms = 0,   .repeat = 1},
};
```

### 12.3 实现方式

使用 ESP-IDF `esp_timer`（软件定时器）驱动，不占用 RTOS 任务：

```c
// buzzer.c 核心逻辑
void buzzer_play(buzzer_pattern_t pattern) {
    // 将 pattern 参数存入静态状态机
    // 启动一次性 esp_timer，回调中切换 GPIO 并计数 repeat
    // 全部 beat 完成后停止定时器，GPIO 拉低
}
```

**调用时机：**
- `gps_tracker.c`：第 10 次有效坐标采集后 → `buzzer_play(BUZZER_PATTERN_CALIBRATED)`

## 13. 卡尔曼滤波说明

针对方位角和仰角各维护一个独立的 1-D 卡尔曼滤波器：

```
状态变量 x  ：当前角度估计值
过程噪声 Q  ：角速度方差（穿越机机动性），初始值 0.1°²
观测噪声 R  ：GPS 定位误差引起的角度方差，初始值 1.0°²
更新周期    ：1 Hz（与 ELRS 遥测速率一致）
```

当 GPS 超时（> 3 s 无新帧）时停止更新，滤波器输出保持最后估计值，舵机位置不变。

---

## 16. 开发优先级

```
Phase 1  espnow_rx + crsf_parser
         ← 接收裸 CRSF 帧，验证 GPS 数据字段解析正确

Phase 2  buzzer + gps_tracker（本机坐标标定）
         ← 10 次均值累积；标定完成触发滴滴声

Phase 3  pointing_calc（Haversine + 双解 + 卡尔曼）
         ← 纯数学，可在 PC 上先写单元测试验证

Phase 4  servo_driver（333 Hz LEDC PWM + 标定补偿层）
         ← 验证舵机响应；servo_set_angle / servo_set_raw 双接口

Phase 5  web_calib（标定页面 + HTTP API）
         ← 此时可以在无 GPS 信号的桌面环境完整标定舵机

Phase 6  antenna_control（跟踪闭环）
         ← 联调双解选优、过渡时间、信号丢失保持、标定模式切换

Phase 7  config_store 整合 + 全参数持久化
         ← 坐标、标定偏移、方向反转均落 NVS，重启免重标定
```
