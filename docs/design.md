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
│   ├── gps_tracker.h        # 飞机 GPS 状态维护 + 本机坐标
│   ├── pointing_calc.h      # 方位角 / 仰角计算
│   ├── servo_driver.h       # PWM 舵机驱动
│   ├── antenna_control.h    # 舵机联动 / 跟踪逻辑
│   ├── config_store.h       # NVS 配置持久化（已有）
│   ├── wifi_config.h        # WiFi AP（已有）
│   ├── nvs_store.h          # NVS 初始化（已有）
│   └── log_service.h        # 日志（已有）
└── src/
    ├── app_main.c           # 入口
    ├── espnow_rx.c
    ├── crsf_parser.c
    ├── gps_tracker.c
    ├── pointing_calc.c
    ├── servo_driver.c
    ├── antenna_control.c
    └── ...（已有模块）
```

---

## 5. 各模块职责

| 模块 | 职责 | 关键接口 |
|------|------|---------|
| `espnow_rx` | 初始化 ESP-NOW，注册接收回调，将原始包推入 FreeRTOS 队列 | `espnow_rx_init()` |
| `crsf_parser` | 从 ESP-NOW payload 中识别并解析 CRSF 帧，提取 GPS 帧字段 | `crsf_parse(buf, len, &frame)` |
| `gps_tracker` | 维护飞机实时坐标、本机（地面站）坐标、最后更新时间戳、信号丢失检测 | `gps_tracker_update()` / `gps_tracker_get_target()` |
| `pointing_calc` | Haversine 公式计算方位角，反正切计算仰角 | `calc_pointing(home, target, &az, &el)` |
| `servo_driver` | LEDC PWM 输出，角度→脉宽映射，行程软件限位 | `servo_set_angle(id, deg)` |
| `antenna_control` | 跟踪主循环，坐标系转换，死区/平滑处理，盲区决策 | `antenna_control_task()` |
| `config_store` | NVS 读写：本机坐标、舵机校准值、零点偏移、盲区方向 | `config_load()` / `config_save()` |

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

### 6.3 舵机角度映射

```
方位舵机（300°）：
  有效范围   [0°, 300°]
  盲区方向   [300°, 360°]（安装时盲区背向常用飞行方向）
  映射公式   servo_az = azimuth * (300.0 / 360.0)
  盲区处理   目标进入盲区时，取最近边界快速穿越（待定方案）

仰角舵机（180°）：
  有效范围   [-10°, 90°] → 映射到 [0°, 180°] 舵机行程
  映射公式   servo_el = (elevation + 10.0) / 100.0 * 180.0
```

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

## 9. 待确认事项

| # | 问题 | 影响模块 |
|---|------|---------|
| 1 | 高频头 ESP-NOW payload 格式：裸 CRSF 帧 还是有额外封装？ | `espnow_rx` / `crsf_parser` |
| 2 | 本机坐标来源：NVS 预存 / 板载 GPS 模块 / WiFi 页面配置？ | `gps_tracker` / `config_store` |
| 3 | 300° 舵机盲区方向：安装时朝哪个方位？盲区穿越策略？ | `antenna_control` |
| 4 | 舵机型号与信号：标准 50 Hz PWM (1000~2000 µs)？GPIO 引脚分配？ | `servo_driver` |
| 5 | 信号丢失行为：超时后原地保持 / 缓慢归中 / 扫描模式？ | `antenna_control` |
| 6 | 平滑滤波方案：低通滤波 / 卡尔曼？更新频率目标值？ | `pointing_calc` / `antenna_control` |

---

## 10. 开发优先级建议

```
Phase 1  ESP-NOW 接收 + CRSF 解析        ← 先拿到飞机坐标，数据源最关键
Phase 2  pointing_calc 指向角计算        ← 纯数学，可在 PC 上先验证
Phase 3  servo_driver PWM 驱动           ← 验证舵机响应和行程
Phase 4  antenna_control 跟踪闭环        ← 联调
Phase 5  config_store + WiFi 配置页面    ← 工程化，方便现场调参
```
