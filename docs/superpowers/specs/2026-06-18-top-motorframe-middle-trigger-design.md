# top 电机系触发改造：X→坐下专用，中键+(BASE/SNIPE/OUTPOST)→电机系

日期：2026-06-18
范围：`top`（云台主控，2026balancehero/top）

## 背景与现状

当前 `top` 的「锁底盘电机系手动模式」（`motorFrameActive`）由**长按 X 满 0.5s**（250 帧@500Hz）触发（`gimbal_task.cpp:258-262`）。
进入后：
- 中键按住 → WASD 微调并写回对应模式偏置（`baseYawBias/basePitchBias` 给 BASE/SNIPE，`outpostYawBias/outpostPitchBias` 给 OUTPOST）；
- 中键松开 → 鼠标自由瞄准（编码器系手动）。

X 同时还承担：`baseLock=1`（坐下/锁水平，`com_task.cpp:93`）与开火（`fire_task.cpp` 多处 `X||SNIPE`、`rctrl_task.cpp:298/310` 手动摩擦轮调速）。

视觉模式下（未进电机系）另有 `GimbalBaseBiasCtrl`/`GimbalOutpostBiasCtrl`（`rctrl_task.cpp:346-386`，在 `actuator_task.cpp:30-31` 于 `GimbalMode` 之前调用）：中键+WASD 在跟视觉的同时直接调偏置。

偏置（4 个 float）目前仅存 RAM，未写 Flash；BASE/SNIPE 视觉路径已把偏置叠加到 setpoint（`gimbal_task.cpp:333-349`），故「下次进 base 仍受偏置影响」当前已成立。

## 目标行为

1. **X 只管坐下**：X 不再触发电机系；保留 X 的 `baseLock`（坐下/锁水平）与全部开火职责。
2. **电机系新触发**：`鼠标中键按住 && vision ∈ {BASE, SNIPE, OUTPOST}` → 进入电机系。立即进入（无 0.5s 防抖窗口；中键在这些模式下是刻意操作）。
3. **电机系内正常调偏置**：WASD 微调并写回当前模式偏置（BASE/SNIPE→base 偏置，OUTPOST→outpost 偏置），鼠标不控云台。沿用现有写回与钳幅（±5°）逻辑。
4. **坐下与电机系并存**：因二者解耦，X（坐下）+中键（电机系）可同时生效，偏置照常写回。
5. **偏置持久**：维持 RAM 存储（整局有效、重启清零），不引入 Flash。视觉路径已叠加偏置，无需改动。
6. **切电机系无跳变（bumpless）**：进入瞬间锁定当前编码器角、次帧预载积分复现外环输出；退出沿 reseed。此逻辑已存在，触发源改变后仍成立。
7. **其它模式中键微调**：vision ∉ {BASE,SNIPE,OUTPOST} 时，中键+WASD = `GimbalCtrl` 既有的手动瞄准（移动云台、不写偏置、不锁底盘）。该分支已正确排除 BASE/OUTPOST/SNIPE，无需改动。

## 实现改动清单（最小化）

### 1) `gimbal_task.cpp` 电机系触发（~258-262）
删除 `xMotorArmCnt` / `kXMotorArmFrames` 计数，替换为：
```c
motorFrameActive = RCVT03.Key.mouse_middle &&
                   (rctrl.vision==BASE || rctrl.vision==SNIPE || rctrl.vision==OUTPOST);
```
进入/退出沿（`lastMotorFrameActive`）、`GimbalMotorFrameHold()` 调用、yaw 收尾与 return 结构保持不变。

### 2) `gimbal_task.cpp` `GimbalMotorFrameHold`（~483-488）
中键已是进入条件 → `fineTune` 恒为真，`if(!fineTune)` 鼠标分支为死代码。移除该鼠标分支与 `fineTune` 间接量，WASD 方向直接读 `RCVT03.Key.A/D/S/W`，更新 yaw/pitch 两速微调与写回逻辑（写回块 534-543 不变，本就覆盖 BASE/SNIPE/OUTPOST）。

### 3) 删除 `GimbalBaseBiasCtrl` / `GimbalOutpostBiasCtrl`
- 删 `rctrl_task.cpp:346-386` 两个函数定义；
- 删 `rctrl_task.h:195-196` 两个声明；
- 删 `actuator_task.cpp:30-31` 两处调用（及 `:29` 相关注释更新）。
理由：新方案下电机系是偏置写回唯一路径；保留会与电机系在同帧重复调偏置（double-count）。

### 4) 注释更新
`gimbal_task.cpp:251-257`、`:425-431`、`gimbal_task.h:142-166` 中「按住 X」描述改为「中键+BASE/SNIPE/OUTPOST」。

### 不改动
- X 的 `baseLock`（`com_task.cpp:93`）、开火（`fire_task.cpp`、`rctrl_task.cpp:298/310`）；
- 中键冻结底盘移动（`rctrl_task.cpp:75-77`）；
- bumpless 切换、视觉偏置叠加、`GimbalCtrl` 其它模式微调分支。

## 操作约束（非代码）
电机系沿用「锁底盘」假设（`motorFrame` 档 yaw 不减 `topW`）。BASE 自带锁底盘；OUTPOST/SNIPE 下若底盘在动，编码器系保持会与世界系漂移——实操按你的用法（X 坐下 + 中键）先坐下锁定即可。

## 验证
编译通过；上车实测：
- 在 BASE/SNIPE/OUTPOST 按中键→云台切编码器保持不跳变，WASD 调偏置；松中键→视觉沿用新偏置无跳变。
- X 单独按只坐下、不进电机系；X+中键同时按→坐下且偏置照写。
- 非 base/snipe/outpost 模式中键+WASD→手动瞄准，不写偏置。
- 切模式后偏置仍在（同一上电周期）；重启清零。
