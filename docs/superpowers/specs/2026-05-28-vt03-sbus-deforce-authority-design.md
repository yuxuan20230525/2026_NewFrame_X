# VT03 / SBUS 脱力上力权限仲裁

- 日期: 2026-05-28
- 项目: 2026balancehero/top (云台 C 板, STM32F4)
- 状态: 已确认设计，待实现

## 背景与问题

云台板有两个遥控输入源：

- **SBUS** (`RC`, USART3)：主控，拨杆 `SW[3]` (SWD) 决定脱力/上力，写全局 `deforceFlag`。离线时强制 `deforceFlag=1`。
- **VT03 图传** (`RCVT03`, USART6)：键鼠 + `pause`/`fn_1`/`fn_2`，主要作战控制器。其旧的 `pause`→脱力逻辑 (`taskSchedule`) 已被注释，是死代码。

`deforceFlag` 是被**全局唯一**消费的脱力主标志（云台、视觉、底盘回调、`com_task`→`onforceFlag` 发往底盘 H7）。

问题：SBUS 可能断连。断连后 SBUS 离线逻辑会把 `deforceFlag` 拉死，机器人无法继续作战。需要一条备份链路：用 VT03 图传接管脱力上力。

## 目标 / 非目标

目标：
- `pause` 把脱力上力权限切到 VT03；VT03 `fn_1` 点按上力、`fn_2` 点按脱力。
- VT03 图传掉线 → 脱力（仅当 VT03 持权限时）。
- SBUS 摇杆四角手势 → 权限收回 SBUS。

非目标（其他控制不变）：
- 不动云台/视觉/底盘/腿长/开火等任何现有控制路径。
- 不改 `deforceFlag` 的任何**消费者**。

## 已确认的设计决策

1. **回收键**：SBUS 摇杆四角手势（不额外占通道）。
2. **切 VT03 瞬间**：保持当前力状态（平滑接管，不动 `deforceFlag`）。
3. **图传掉线脱力**：仅当 VT03 持权限时生效；SBUS 持权限时图传掉线不影响。
4. **权限指示**：暂不做（YAGNI），先只做核心逻辑。

## 架构

不引入"最终标志位 + 改所有消费者"。让 `deforceFlag` 本身成为仲裁结果：用 `deforceAuthority` 决定谁能写它，下游消费者零改动。

新增（`dbus.h` 导出 / `dbus.cpp` 定义）：

- `uint8_t deforceAuthority`：`DEFORCE_BY_SBUS=0`（默认） / `DEFORCE_BY_VT03=1`。
- `void DeforceSet(uint8_t want)`：唯一改 `deforceFlag` 的入口。复刻现有回调语义——仅在跳变时调 `deforceCallBack()`/`deforceCancelCallBack()`，上力跳变时复位 `imutask.forceGetOffset`。SBUS 与 VT03 都经此函数改力，回调按真实跳变触发，不重复不遗漏。

仲裁不变量：任一时刻只有持权限的一方调用 `DeforceSet()`；另一方对 `deforceFlag` 只读不写。VT03 在 1kHz 循环中先于 SBUS 执行，但因写操作被权限互斥，执行顺序不影响结果。

## SBUS 侧行为 (`dbus.cpp`)

- `_RC::taskSchedule()`：
  - `remoteUpdate()` 每周期照常（其他控制不变）。
  - 检测**收权手势**（见下），命中则 `deforceAuthority = DEFORCE_BY_SBUS`。
  - 仅当 `deforceAuthority==DEFORCE_BY_SBUS`：按 `SW[3]` 走 `DeforceSet(1/0)`；脱力分支保留 `rstCount()`。
- `_RC::run_1000Hz()` 离线分支：按键 memset 清零照常（其他控制走中立）；仅当 `deforceAuthority==DEFORCE_BY_SBUS` 才 `DeforceSet(1)`。**这是关键**：SBUS 断连且权限已在 VT03 时，绝不让 SBUS 离线逻辑拉死力——正是本功能要解决的场景。

## VT03 侧行为 (`remotevt03.cpp`)

- `run_1000Hz()`：在线时 `handleData()` 解析；每周期调用 `vt03DeforceSchedule()`。
- `vt03DeforceSchedule()`：
  - `pause` 上升沿（仅在线）→ `deforceAuthority = DEFORCE_BY_VT03`，不动 `deforceFlag`（保持当前力状态）。
  - 若 `deforceAuthority != DEFORCE_BY_VT03` → 直接返回，不碰 `deforceFlag`。
  - 持权限时：图传离线 → `DeforceSet(1)`；`fn_1` 上升沿 → `DeforceSet(0)` 上力；`fn_2` 上升沿 → `DeforceSet(1)` 脱力。
- 删除死代码 `taskSchedule()` 与冗余全局 `deforceFlagvt03`（统一到 `deforceFlag`）。`#include "dbus.h"`。

## 收权手势定义（可调）

复用四角打杆，选与现有所有手势（重启 / 陀螺校准 / LegCheck / 锁底盘 / posFlag / 吊射）汉明距离 ≥2 的组合，**按住约 1s**（1000 个 1kHz tick）防误触：

| 通道 | 阈值 | 物理含义 |
|---|---|---|
| `CH[0]` | `< -600` | 右摇杆 → 左 |
| `CH[1]` | `< -600` | 右摇杆 → 下 |
| `CH[2]` | `< -600` | 左摇杆 → 上 |
| `CH[3]` | `> 600`  | 左摇杆 → 左 |

手势检测在 SBUS 在线时运行（与权限无关），故 VT03 持权限期间仍可随时收回。

## 安全性

- 上电默认 `deforceAuthority=SBUS`、`deforceFlag=1`（脱力），安全。
- 收权瞬间 `deforceFlag` 立即跟随 SWD：若收权时 SWD 在上力档，机器人会立即上电。**操作规程**：收权前先把 SWD 拨到脱力档。代码注释会标注此点。
- 切 VT03 后若图传掉线 → 立即脱力。

## 改动文件

- `TDT_Device/inc/dbus.h`：导出 `deforceAuthority`、`DeforceSet()`、`DEFORCE_BY_*` 宏。
- `TDT_Device/src/dbus.cpp`：定义上述；改 `run_1000Hz` 离线分支与 `taskSchedule`（手势 + 权限门控）。
- `TDT_Device/inc/remotevt03.h`：删 `taskSchedule`/`deforceFlagvt03` 声明；加 `vt03DeforceSchedule()`。
- `TDT_Device/src/remotevt03.cpp`：`#include "dbus.h"`；重写 `run_1000Hz`；加 `vt03DeforceSchedule()`；删死代码。
- `TDT_Task/src/com_task.cpp`：清理第 73 行过时注释。

## 验证

- 编译过（cmake_build）。
- 台架：SWD 切脱力/上力正常；拔 SBUS 接收机不再被动拉死（需先按 pause 接管）。
- 按 pause → fn1 上力 / fn2 脱力；拔 VT03 → 脱力。
- 四角手势 1s → 权限回 SBUS，SWD 重新生效。
