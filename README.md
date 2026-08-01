# 基于 STM32WBA 的低功耗 BLE 姿态环境监测节点

基于 STM32WBA5MMG 无线模块的 BLE 姿态环境监测节点示例工程。通过 I²C 读取板载温度传感器（SHT40-AD1B）和六轴惯性传感器（ISM330DHCX），使用 Madgwick AHRS 算法进行姿态融合，输出俯仰角（Pitch）和横滚角（Roll），并通过自定义 BLE GATT 服务以 Notify 方式实时推送到手机端（nRF Connect）。

## 功能

- 温度、三轴加速度、三轴角速度采集（I²C3，100 kHz）
- Madgwick AHRS 六轴姿态融合，输出 Roll / Pitch（20 Hz）
- 自定义 Telemetry GATT 服务（20 字节 Notify 包：时间戳 + 温度 + 加速度 + Roll/Pitch）
- 按键交互：
  - 短按：切换实时采集模式 ↔ 低功耗监测模式
  - 长按 3 秒：进入深度休眠（仅 BLE 断开时生效）
- LED 状态指示：
  - 快闪（2 Hz）= 广播中（未连接）
  - 常亮 = 已连接，实时采集模式
  - 慢闪（0.5 Hz）= 已连接，低功耗监测模式
  - 熄灭 = 深度休眠
- 低功耗优化：
  - 实时模式：传感器 ODR 104 Hz，BLE 连接间隔 ~20 ms，广播间隔 100 ms
  - 低功耗模式：传感器 ODR 12.5 Hz，BLE 连接间隔 ~500 ms，广播间隔 1 s
  - STOP 低功耗模式（STOP1，空闲自动进入、按键唤醒）

## Telemetry 数据格式（小端，20 字节）

| 偏移 | 字节 | 类型 | 含义 |
|---|---|---|---|
| 0 | 4 | uint32 | 时间戳（ms） |
| 4 | 2 | int16 | 温度（0.01 °C） |
| 6 | 2 | int16 | 加速度 X（mg） |
| 8 | 2 | int16 | 加速度 Y（mg） |
| 10 | 2 | int16 | 加速度 Z（mg） |
| 12 | 4 | float | Roll（°） |
| 16 | 4 | float | Pitch（°） |

## 依赖

- STM32CubeIDE
- STM32CubeWBA Firmware Package V1.10.0（STM32Cube_FW_WBA_V1.10.0）
- B-WBA5M-WPAN 开发板（STM32WBA5MMG）
- ST-LINK/V3 调试器（STDC14 接口）
- 手机端 nRF Connect

## 使用方法

1. 下载并解压 STM32CubeWBA V1.10.0 软件包；
2. 将本仓库内容**覆盖**到官方 BLE_Sensor 工程目录：

   ```
   STM32Cube_FW_WBA_V1.10.0/Projects/B-WBA5M-WPAN/Applications/BLE/BLE_Sensor/
   ```

   覆盖其中 `Core/`、`STM32_WPAN/`、`System/`、`STM32CubeIDE/` 等目录；

3. 用 STM32CubeIDE 导入该工程（File → Import → Existing Projects into Workspace）；
4. 编译并烧录（ST-LINK/V3 连接 STDC14）；
5. 打开手机 nRF Connect，扫描并连接设备（设备名 `WBA5M_xx`）；
6. 展开 Telemetry 服务（UUID 以 `7a112d90` 开头），启用 Notify 接收数据。

> 注：本工程并非自包含工程，编译依赖 STM32CubeWBA V1.10.0 软件包中的 Drivers、Middlewares、Utilities 和 Projects/Common 资源（通过相对路径引用）。

## 工程结构

```
Core/                     应用主流程与配置（app_conf.h 含 GATT/LPM 配置）
STM32_WPAN/App/           应用代码
├── app_ble.c/.h          广播间隔、连接参数、按键处理
├── ble_sensor.c/.h       Telemetry 服务注册与数据更新
├── ble_sensor_app.c/.h   传感器采集、姿态融合、模式切换、LED、长按检测
├── attitude_madgwick.c/.h  Madgwick AHRS 算法（新增）
└── sensor_calibration.c/.h 六面加速度计 + 陀螺仪零偏校准（新增）
System/                   低功耗接口
BLE_Sensor.ioc            CubeMX 工程配置
```

## 主要修改点（相对官方 BLE_Sensor 示例）

- 新增 `attitude_madgwick.c/.h`：Madgwick AHRS 姿态融合；
- 新增 `sensor_calibration.c/.h`：传感器校准流程；
- `ble_sensor.c/.h`：新增独立 Telemetry GATT 服务与特征；
- `ble_sensor_app.c`：传感器采集管线、Telemetry 发送、模式状态机、LED 任务、应用层长按检测；
- `app_ble.c/.h`：动态广播间隔、连接参数更新、深度休眠入口；
- `app_conf.h`：GATT 属性扩容、STOP1 低功耗、新增 LED 任务 ID。

## 说明

- 姿态融合采用 Madgwick 算法（STM32CubeWBA V1.10.0 不含适用于本平台的 MotionFX 库）；
- 长按检测为应用层轮询实现（官方 BSP 的下降沿触发方式在该板上无法检测长按）；
- STOP 低功耗模式使用 STOP1（官方 BLE_Sensor 基线仅实现 STOP1 进入路径）。
