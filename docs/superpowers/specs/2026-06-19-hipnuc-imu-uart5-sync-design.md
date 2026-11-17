# HiPNUC HI91 IMU 接入 top（UART5 + SYNC_IN 硬件时间同步）

日期：2026-06-19
目标工程：`/home/z/2026balancehero/top`（DJI 开发板 C 型 / STM32F407IGH6，平衡步兵）
参考：HiPNUC《IMU 指令与编程手册》Rev 1.7.1（`~/Downloads/imu_cum_cn.pdf`）、RoboMaster 开发板 C 型用户手册

---

## 0. 更新（2026-06-19，数据口 UART5 → USART1）

最初按 `vision.cpp` 的 V5/V6 块选了 **UART5(PC12/PD2)**，但核 C 板手册发现 **UART5 在 C 板没引出连接器**——
C 板只引出 **USART1(4-pin 对外口, PA9 TX/PB7 RX)** 和 **USART6(3-pin)**（⚠️外壳丝印反: 壳印"UART2"=USART1,
壳印"UART1"=USART6）。故数据口改为 **USART1**:

- 引脚 PA9(TX)/PB7(RX); RX=DMA2_Stream2, TX=DMA2_Stream7; USART1_IRQ; 921600。
- USART1 物理空闲: `USE_VISION=0` → `board.cpp` 的 `Vision.init()` 从不调(视觉走 USB); 只需让出
  `vision.cpp` 的 `IF_USE_DJIC` 块两个 ISR(`USART1_IRQHandler`/`DMA2_Stream7_IRQHandler`), 已用
  `#if !HIPNUC_def` guard(`Vision` 对象仍构造供 USB)。V5 块的 UART5 guard 已回退(不再相关)。
- 下文 §2/§4/§5/§8 中"UART5/PC12/PD2/DMA1_Stream0/DMA1_Stream7/UART5_IRQ"均以本节为准替换为
  USART1 对应资源; SYNC_IN(PI7)触发、ONMARK、1 拍流水线、轴向重映射等其余设计不变。
- 文件名保留 `uart5` 仅为不改链接; 内容以 USART1 为准。

---

## 1. 目标与范围

把一颗 HiPNUC HI 系列外置 IMU 经 **UART5** 接进 top，复用现有 `imutask` 接口：让 `boardImu`
指向新设备类，下游 `gimbal / balance / vision` 读 `boardImu->FusionAngle`、`boardImu->gyro.dps`、
`boardImu->readTime` 一行不改。用**模块自带的 6/9 轴融合**（直接吃帧里的 euler/quat），不在 STM32 上
重跑 Fusion AHRS。IMU 与 C 板之间用 **SYNC_IN 硬件触发**做时间同步（C 板当主）。

**不在本次范围**：CAN/Modbus 接入；模块地磁校准流程；HI83 变长帧；相机-IMU 的 SOUT 分频同步
（留作后续，见 §9）。

## 2. 关键背景（来自代码核实）

- `imutask`（`TDT_Task/.../imu_task.*`）以 **1kHz** 跑，`run()` 调 `boardImu->TDT_IMU_update()`。
  现有设备 `Bmi088/Mpu6050/Scha634_03/Icm45686` 都派生自 `ImuCalc`（`TDT_Alg/inc/imu.h`），
  当前 `BMI088_def = 1`。
- 控制环时基 = **SysTick 1kHz**（`TDT_Bsp/.../systick.cpp` → `TDT_Loop_1000Hz`）；`getSysTimeUs()`
  来自独立 µs 自由计数器（`TDT_Bsp/src/timer.cpp:95`）。BMI088 的 `readTime` 就是在
  `TDT_IMU_update()`（`imu.cpp:106`）里 `getSysTimeUs()` 打的——即"控制 tick 时刻"。
- **UART5（PC12=TX / PD2=RX，921600）当前物理空闲**：UART5 上的 Vision 实例在 `IF_USE_V5_V5PLUS_V6`
  块里，而板子是 `IF_USE_DJIC=1`、`USE_USB_VISION=1`（视觉走 USB）。`UART5_IRQHandler` 与
  `DMA1_Stream7_IRQHandler` 当前未在任何编译单元定义 → HiPNUC 可干净占用。
- UART5 收发资源（与 `vision.cpp` 里 V5 块一致）：RX `DMA1_Channel4/Stream0`，TX `DMA1_Channel4/Stream7`，
  `UART5_IRQn`。
- C 板 7 路 PWM 排针全工程未用：`PE9/PE11/PE13/PE14`(TIM1_CH1‑4)、`PI5/PI6/PI7`(TIM8_CH1‑3)。
- 现有视觉时间同步 `TimeSimultaneity`（`TimeMatch.cpp`）只估计 NUC↔MCU 时钟偏移 `time_diff`，
  全靠 `boardImu->readTime` 当"姿态采样时刻"。**本设计只改 `readTime` 来源，该机制零改动。**

## 3. HI91 帧格式（手册 §5）

二进制流：`SOF(5A A5) | LEN(2, LE, =76) | CRC16(2, CRC‑16/CCITT, init=0) | payload(76)`。
payload tag `0x91`，偏移：

| 偏移 | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0 | tag=0x91 | u8 | - |
| 1 | main_status | u16 | - |
| 3 | temperature | i8 | ℃ |
| 4 | air_pressure | f32 | Pa |
| 8 | system_time | u32 | ms |
| 12 | acc[3] | f32×3 | **G** |
| 24 | gyr[3] | f32×3 | deg/s |
| 36 | mag[3] | f32×3 | µT |
| 48 | roll | f32 | deg |
| 52 | pitch | f32 | deg |
| 56 | yaw | f32 | deg |
| 60 | quat[4] | f32×4 | WXYZ |

CRC：先对 `buf[0..3]`（SOF+LEN）更新，再对 `buf[6..6+LEN-1]`（payload）更新，比对 `buf[4..5]`。

## 4. 架构

新增 `class HipnucImu : public ImuCalc, public USART_DMA`（多继承，复用两套现成基建）：

- `USART_DMA` 侧：直接复用 `vision.cpp` 验证过的 **UART5 + 空闲中断 + DMA** 收发机，构造参数照搬
  V5 块。`ProcessReceivedData()` override 解析 HI91。
- `ImuCalc` 侧：`boardImu` 指向它；`GetOriginData()` override 为空（不走 int16 origin 管线）；
  `TDT_IMU_update()` override 用帧数据直接填解算结果。

**全局对象** `HipnucImu hipnucImu(...)`（不是 `new`，让 ISR 有稳定引用，仿 `VISION Vision`）。

文件改动：

| 文件 | 改动 |
|---|---|
| `TDT_Device/inc/hipnuc_imu.h`（新增） | `HipnucImu` 类、HI91 偏移宏、轴向重映射配置 |
| `TDT_Device/src/hipnuc_imu.cpp`（新增） | 类实现、全局 `hipnucImu`、`UART5_IRQHandler`/`DMA1_Stream7_IRQHandler`、CRC、ONMARK 配置发送，整体 `#if HIPNUC_def` 包裹 |
| `TDT_Alg/inc/imu.h` | `void TDT_IMU_update();` → `virtual void TDT_IMU_update();`（仅此一词；无任何现有子类重写，零行为风险） |
| `TDT_Task/inc/imu_task.h` | 加 `#define HIPNUC_def 0`；选中时置 1、`BMI088_def 0`；`#include "hipnuc_imu.h"` |
| `TDT_Task/src/imu_task.cpp` | `init()` 加 `#if HIPNUC_def` 分支：`boardImu = &hipnucImu;`（跳过 `ParamToImu` flash 零偏 link，改调轴向重映射 setter） |
| `TDT_Device/src/vision.cpp` | `#if IF_USE_V5_V5PLUS_V6` 块加 `&& !HIPNUC_def`，保 UART5 互斥（vision.cpp 已 include imu_task.h） |

## 5. 时间同步：SYNC_IN（C 板当主，相位锁进 1kHz 控制环）

### 触发硬件
- **触发脚：`PI7`（TIM8_CH3，7 路 PWM 排针 C7 那一排）**，当**普通 GPIO 推挽输出**用，不占 TIM8。
  需上电使能 `RCC_AHB1Periph_GPIOI`。
- 模块侧 HI91 配 **ONMARK**（手册 §2.2.1）：不再周期发，每个 SYNC_IN 上升沿吐一帧。

### 触发与打戳
在 `HipnucImu::TDT_IMU_update()`（被 `imutask.run()` 1kHz 调，和 BMI088 读数同一时点）开头：
1. `PI7` 拉上升沿（模块此刻采样）；
2. 同一处 `uint64_t trigTimeUs = getSysTimeUs();` latch；
3. 触发与 `readTime` 的相位 = 现有 BMI088 在 `imu.cpp:106` 的相位，**语义不变**。

### 触发↔帧配对（1 拍流水线）
ONMARK 是 1 触发→1 帧、严格有序。82B@921600 ≈ 0.89ms + 模块时延 → 帧在**下一拍**才到。

- 维护 `trigFifo`（深度 3 的环形）。每拍：
  - 若 `dataReady`（上一拍那次触发的帧已在 idle ISR 收齐）：`readTime = trigFifo.pop()/1e6`，
    消费快照填解算结果，清 `dataReady`；
  - 发新上升沿、`trigFifo.push(trigTimeUs)`。
- 错位兜底：CRC 丢帧会让 fifo 堆积；当 `trigFifo` 满则丢最旧重同步，并置 `offlineFlag`。
- 效果：`boardImu->readTime` = 该帧姿态**真正采样的控制 tick 时刻**，零相位抖动。姿态本身有固定
  ~1ms 管线延迟（UART 不可避免，且 readTime 已如实反映）。

### 模块配置（解决波特率"鸡生蛋"）
- **前置（人工一次，CHCenter）**：模块设 **921600 + HI91 + SAVECONFIG** 持久化。
- **板子侧（每次上电）**：`init()` 经 UART5 TX（DMA1_Stream7）补发 `LOG HI91 ONMARK\r\n` 重申触发模式，
  防模块被复位回周期模式。不发 SAVECONFIG（不磨损 flash）。

## 6. 数据映射（`TDT_IMU_update` override）

从快照填（轴向重映射后）：
- `acc.accValue[i] = acc_G[i] * 9.81f`（G→m/s²）
- `gyro.dps[i] = gyr[i]`；`gyro.radps[i] = gyr[i] * ANGLE_TO_RAD`
- euler → `My_data.nowAngle` / `AHRS_data.nowAngle`，复刻基类 ±200° 过圈累加（`imu.cpp:161‑165`/
  `ImuCrossRoundHandle`）得到**连续** `FusionAngle.yaw` + 填 `AHRS_data.Angle` / `My_data.Angle`
- quat(WXYZ) → `My_data.quaternion` / `AHRS_data.ins_quat`
- `imuValid = 1`；离线看门狗复位

### 轴向重映射
模块输出是其自身机体系下的 euler/gyro/acc。提供可配 **符号 + 轴置换** 层（仿 BMI088 的
`gyroScaleFactor`），默认**单位映射**，`// TODO 真机定标`：真机看 yaw 旋转正方向、pitch/roll 正负、
gyro 三轴符号后定。需确认模块贴装方向与 euler 约定（ENU‑312 / NED‑321）。

## 7. 初始化与离线

- `imutask.init()`：`#if HIPNUC_def` → `boardImu = &hipnucImu;` → `boardImu->TDT_ImuInit()`。
  `TDT_ImuInit` 先调 `init()`（HiPNUC override：`USART_DMA::init()` 起 UART5+DMA+IRQ、配 `PI7` 输出、
  发 ONMARK），其余 `My_DaTaInit/AHRS_dataInit` 用首帧前的零值初始化（首帧到后即被刷新，可接受）。
- 离线：连续 N 拍（默认 100，即 100ms）无有效帧 → `offlineFlag=1`、`imuValid=0`。

## 8. 风险与规避

1. **帧同步**：复用 vision "normal DMA + 空闲重启"模式；解析器**扫帧头 + CRC** 容错。ONMARK 下每触发
   一帧、节奏稳，粘帧风险低。若真机发现粘帧 → 退路改 circular DMA（见 §9）。
2. **波特率鸡生蛋**：靠 §5 的"CHCenter 预存 921600 + 板子补发 ONMARK"规避，不做易碎的在线变波特率握手。
3. **CMake**：实现时先核实是否自动 glob `TDT_Device/src/*.cpp`；不自动则显式加入。
4. **多继承二义**：`USART_DMA` 与 `ImuCalc` 无同名虚函数冲突（`ProcessReceivedData` vs
   `GetOriginData/TDT_IMU_update`），安全。`USART_DMA` 的 TX 队列在只收场景闲置但无害（ONMARK 配置发送会用到 TX）。

## 9. 后续（本次不做）

- SOUT 分频 → 同步相机，做相机-IMU 严格对齐（手册 §2.3 + `CONFIG PMUX2 DIV` + MAIN_STATUS bit12）。
- 升级 B 案：用帧内 `system_time` + 偏移整定进一步剔除 ISR 抖动 / 跟时钟漂移（HI91 只有 1ms 粒度，
  µs 级需切 HI83 帧）。
- circular DMA 收帧（若 normal+idle 在 1kHz 下不稳）。

## 10. 验收

- 编译通过（ARM 交叉编译，先确认工具链）。
- 真机：`g_hipnucDbg` 快照里 acc/gyro/euler/quat 合理；`FusionAngle.yaw` 连续过圈；`readTime` 抖动
  < 1 个任务周期；离线拔线 `offlineFlag` 置位、复位恢复。
- 轴向定标：静放各面 acc≈(0,0,±9.81)；绕各轴转 gyro/euler 符号与机体一致。
