# 手势控制无人机系统（Gesture-Controlled Drone）

> 基于 STM32F103C8T6 + MPU6050 + nRF24L01+ + Betaflight 飞控的 IMU 手势控制无人机系统。
> 佩戴手套端，通过手腕姿态与动作无线控制无人机飞行。

[![Platform](https://img.shields.io/badge/Platform-STM32F103C8T6-blue)](#)
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK%20(AC6)-purple)](#)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## 项目简介

传统无人机依赖双摇杆遥控器，学习成本高。本项目用一块戴在手腕上的 STM32 开发板（集成 MPU6050）实时解算手腕姿态并识别手势，通过 2.4GHz nRF24L01+ 无线链路将飞行指令发送到机载接收器，接收器再通过 MSP 协议把指令注入 Betaflight 飞控，实现"手势即遥控"。

### 系统架构

```
┌───────────────────────────┐            ┌───────────────────────────┐
│        手套端 (TX)         │  2.4GHz    │      无人机端 (RX)         │
│  GestureGlove/            │  nRF24L01+ │  DroneReceiver_new/       │
│                           │ ?========? │                           │
│  MPU6050 (I2C1 PB6/PB7)   │            │  nRF24L01+ (SPI1)         │
│      ? 姿态解算(互补滤波)  │            │      ? 数据接收与校验      │
│      ? 手势识别(8种)       │            │      ? 手势→RC通道映射     │
│  nRF24L01+ (SPI1, 发送)   │            │  MSP over UART2 (PA2)     │
│  UART1 (PA9/PA10 调试)    │            │      ? Betaflight 飞控     │
└───────────────────────────┘            │  UART1 (PA9/PA10 调试)    │
                                         └───────────────────────────┘
```

### 手势 → 飞行指令映射

| 手势 | 触发条件 | 飞行指令 |
|------|---------|---------|
| 悬停 | Roll/Pitch 均 < 5° | HOVER（姿态回中，油门保持） |
| 前倾 | Pitch > 15° | FORWARD |
| 后仰 | Pitch < -15° | BACKWARD |
| 左倾 | Roll < -15° | LEFT |
| 右倾 | Roll > 15° | RIGHT |
| 快速上抬 | Z 轴加速度 > 1.5g | UP（先解锁，再升油门） |
| 快速下压 | Z 轴加速度 < -1.5g | DOWN（降油门后上锁） |
| 大幅倾斜 | 任意轴 > 60° | EMERGENCY_STOP（立即上锁断油） |

---

## 目录结构

```
.
├── GestureGlove/                 # 手套端（发送端）Keil 工程
│   ├── Core/Inc/  Core/Src/      # 用户代码
│   ├── Drivers/                  # ST HAL 库 + CMSIS（第三方，保持原许可证）
│   ├── MDK-ARM/                  # Keil 工程文件
│   └── GestureGlove.ioc          # STM32CubeMX 配置
├── DroneReceiver_new/            # 无人机端（接收端）Keil 工程
│   ├── Core/Inc/  Core/Src/
│   ├── Drivers/
│   ├── MDK-ARM/
│   ├── GestureGlove.ioc          # 沿用手套端 CubeMX 配置（仅外设不同）
│   └── test_msp.c                # MSP 直发测试片段（注释形式，可对照调试）
└── docs（*.md）                  # 项目建议书/开题报告/硬件组装指南/进度报告
```

---

## 硬件连接

### 手套端（STM32F103C8T6 最小系统板）

| 外设 | 引脚 | 说明 |
|------|------|------|
| nRF24L01+ CE | PA8 | 芯片使能 |
| nRF24L01+ CSN | PA4 | SPI 片选（软件 NSS） |
| nRF24L01+ SCK/MISO/MOSI | PA5/PA6/PA7 | SPI1 |
| nRF24L01+ VCC/GND | 3.3V/GND | 供电必须稳，推荐模块就近加 10uF 电容 |
| MPU6050 SCL/SDA | PB6/PB7 | I2C1 |
| 调试串口 | PA9(TX)/PA10(RX) | UART1, 115200-8-N-1 |

### 无人机端（STM32F103C8T6 + Betaflight 飞控）

| 外设 | 引脚 | 说明 |
|------|------|------|
| nRF24L01+ CE | PA4 | 芯片使能 |
| nRF24L01+ CSN | PB0 | SPI 片选（软件 NSS） |
| nRF24L01+ SCK/MISO/MOSI | PA5/PA6/PA7 | SPI1 |
| MSP 串口 | PA2(TX) → 飞控 UART-RX | UART2, 115200，务必共地 |
| 调试串口 | PA9(TX)/PA10(RX) | UART1, 115200-8-N-1 |

> 注意：两端 nRF24L01+ 的 CE/CSN 引脚分配**不同**，接线时对照上表，不要照搬手套端。

---

## 编译与烧录

1. 用 Keil MDK（AC6 编译器）打开：
   - 手套端：`GestureGlove/MDK-ARM/GestureGlove.uvprojx`
   - 无人机端：`DroneReceiver_new/MDK-ARM/GestureGlove.uvprojx`
2. **先 Rebuild 全量编译**（重要：避免烧录到旧的中间产物）。
3. ST-LINK 连接 SWD 后 Flash → Download。
4. `.uvoptx`/`.uvguix` 等个人设置文件已被 .gitignore 忽略，打开工程后 Keil 会自动重建。

---

## Betaflight 配置（无人机端）

1. Betaflight Configurator 连接飞控。
2. **端口（Ports）**：找到与 STM32 UART2(TX=PA2) 相连的 UART，打开 **MSP** 开关，波特率 115200。
3. **配置（Configuration）→ 接收机（Receiver）**：
   - 接收机模式 = **MSP RX input**
4. **接收机（Receiver）选项卡**：观察 RC 通道 1到8 是否随手套手势在 1000到2000 之间变化。
5. **模式（Modes）**：为 AUX1 配置 ARM 通道（本固件中 AUX1=CH5）。
6. 建议启用飞控端失控保护（Failsafe），配合接收器 500ms 断链自动上锁形成双重保护。

---

## 联调方法（分步验证）

按以下顺序验证，能快速定位问题出在哪一层：

1. **手套端串口**（UART1）应看到：
   ```
   === BOOT === MPU:1 WHO:0x68 NRF:OK
   NRF: CONFIG=0x0E STATUS=0x0E RF_CH=40 RF_SETUP=0x0F FIFO=0x11
   [xxx] Roll:.. Pitch:.. Az:.. | Hover | TX:OK | cnt:.. ok:.. fail:0
   ```
   - `NRF:OK` 说明手套端 SPI 通路正常；
   - `TX:OK`（且 fail 不增长）说明手套端发出的包收到了无人机端的 ACK —— **无线链路已打通**。
2. **无人机端串口**（UART1）应看到：
   ```
   nRF24L01+ OK
   NRF: CONFIG=0x0F STATUS=0x0E RF_CH=40 RF_SETUP=0x0F FIFO=0x11
   RX#1 Gesture:1 Roll:.. Pitch:..
   ```
   - `RX#` 数字持续增长 = 正在收到手套端数据；
   - 持续打印 `No signal.` = 无线链路不通，回到第 1 步检查 TX 状态。
3. **Betaflight 接收机选项卡**：做前倾手势，Pitch 通道应向 2000 方向变化。
4. 最后再进行上桨实飞测试（见下方安全说明）。

---

## 本项目已修复的问题（历史踩坑记录）

> 上学期联调失败"手套端信息到不了无人机端"的根因，按影响排序：

1. **[致命] 手套端 `NRF24L01_Check()` 检测完芯片后没有恢复通信地址**
   `Check()` 把测试值 `0xA5A5A5A5A5` 写进 TX_ADDR 后没有写回 `0xE7E7E7E7E7`，
   导致手套实际向错误地址发数据，接收端永远收不到，发送永远 MAX_RT 报 `TX:FAIL`。
   → 已在 `GestureGlove/Core/Src/nrf24l01.c` 修复（检测后恢复 TX/RX 地址）。
2. **两端数据包结构体定义不一致**
   发送端 `gesture` 是 4 字节枚举、接收端是 `uint8_t`，此前"碰巧"能用但极脆弱。
   → 已统一为 20 字节固定布局，并加编译期断言（两端任一改动都会编译报错）。
3. **解锁与油门同时给（Betaflight 拒绝解锁）**
   原代码 UP 手势在 AUX1 解锁的同一帧就输出 60% 油门，飞控因"油门不在最低"拒绝解锁，
   表现为飞控没反应。→ 已改为先解锁（油门最低），确认后再平滑升油门。
4. **通道值阶跃跳变**
   原代码手势切换时 RC 通道直接跳变，飞行会明显抖动。→ 已加斜坡平滑过渡。
5. **无人机端 Keil 工程 include 路径指向已不存在的 `D:\CRAIC\DroneReceiver\Core\Inc`**
   （目录改名后工程未同步）。→ 已清理绝对路径，工程可随仓库整体迁移。
6. **无人机端旧固件未全量重编译**
   上次构建日志显示 0 秒增量构建，板子里跑的可能是改动前固件。
   → 上传前请对两端各做一次 **Rebuild** 并重新烧录。

---

## 常见问题排查

| 现象 | 排查方向 |
|------|---------|
| 手套端 `NRF:FAIL` | CE/CSN/SPI 接线、3.3V 供电（PA 模块瞬时电流大，稳压不足会复位）、模块损坏 |
| `NRF:OK` 但 `TX:FAIL` | 无人机端是否上电；两端地址/频道/速率是否一致（本仓库已统一）；接收端 CE 是否被拉高；距离过远 |
| 无人机端一直 `No signal.` | 同上，先确认手套端 `TX:OK` 且 fail 不增长 |
| 收到数据但飞控没反应 | 确认 Betaflight 端口 MSP 开启、接收机模式为 MSP RX；检查 AUX1 是否正确映射 ARM |
| 无法解锁 | 解锁瞬间油门必须为最低（本固件已保证）；检查 Betaflight 的 arming 保护项 |
| 收到数据但通道乱跳 | 两端固件版本不一致，全量 Rebuild 后重烧 |

---

## 安全警告（重要）

- **本系统为教学/演示原型**，无线链路、手势识别均存在误判与中断可能，**不可用于载人/贵重负载或人群上空飞行**。
- 首次试飞请：拆桨验证通道 → 系绳/护桨圈 → 低空短时飞行；随时准备用传统遥控器接管。
- 本固件已内置 500ms 断链自动上锁断油保护，但请勿将其作为唯一安全保障。

---

## 许可证

- 本项目用户代码（`Core/`、`MDK-ARM/`、文档）以 [MIT License](LICENSE) 开源。
- `Drivers/` 目录为 STMicroelectronics HAL 驱动（BSD-3-Clause）与 ARM CMSIS（Apache-2.0），
  版权归各自所有者，保留原始许可证（见 `Drivers/STM32F1xx_HAL_Driver/LICENSE.txt`）。
- Betaflight 为独立开源项目（GPL-3.0），本项目仅通过 MSP 协议与其通信。

---

## 致谢

- 项目文档：`手势控制无人机项目建议书.md`、`手势控制无人机项目开题报告.md`、`无人机硬件组装详细指南.md`
- 团队成员：陈胤成、徐靖、魏铭宏、张益玮（工程学导论项目 B 第二组）
