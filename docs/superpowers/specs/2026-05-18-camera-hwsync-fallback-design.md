# 相机硬同步失联软同步兜底设计

- **日期**: 2026-05-18
- **范围**: `2026balancehero/top/TDT_Device/{inc,src}/camera_Timematch.{h,cpp}`
- **作用板**: C 板（STM32F4），H7 主线暂未实现该模块，不在本 spec 范围

## 1. 背景

`CameraTM` 给相机做硬件触发同步：

| 角色 | 实现 |
|---|---|
| 触发输出 | `TIM1_CH2 / PE11` PWM (PSC=168-1, ARR=6000-1, PWM1, 低有效, 5/6 占空) → 166.67 Hz |
| Strobe 回传 | `EXTI9 / PE9` 下降沿 |
| 触发计数 | `TIM1 update IRQ` 每周期 `send_frameID++` |
| Strobe 计数 | EXTI 里 `recieve_frameID++` |
| Pose 抓拍 | EXTI 里把当前 `gimbal.yawAngle / pitchAngle / boardImu->FusionAngle.roll` 存入 `camera_yaw / pitch / roll` |
| 下游消费 | `vision_Send_Data()` 把 `camera_*` 角度 + `recieve_frameID` 打包发 NUC |

## 2. 故障假设

PE9 strobe 通道异常（线脱焊 / EMI / 相机重启 / 相机模式被误改成 free-run）→
- `recieve_frameID` 冻结
- `camera_yaw/pitch/roll` 卡在最后一次 strobe 时刻
- NUC 拿陈旧 pose 反算装甲板坐标 → 弹道偏

电控对此**无任何感知和兜底**，是本次要补的洞。

## 3. 目标

| # | 目标 |
|---|---|
| G1 | 检测 PE9 strobe 持续 ≥ 1 秒缺失 → 进入软同步 |
| G2 | 软同步模式下用 TIM1 update IRQ 当作"虚拟 strobe"抓 IMU pose，让 `camera_*` 持续刷新 |
| G3 | 软同步下 `recieve_frameID` 继续自增，让 NUC 看到 frame ID 不冻结 |
| G4 | 一旦 PE9 再次触发，立即切回硬同步（无迟滞） |
| G5 | 不改对外接口（`vision_Send_Data` 不动），不影响其他模块 |

## 4. 状态机

```
            +--------+        miss_count >= MISS_THRESH         +--------+
            |        | ---------------------------------------> |        |
   boot --> | HW_SYNC|                                          | SW_SYNC|
            |        | <--------------------------------------- |        |
            +--------+        PE9 edge in EXTI9_5_IRQHandler    +--------+
```

- 初始态：`HW_SYNC`
- 切换条件:
  - **HW → SW**: TIM1 update IRQ 中检测到连续 `MISS_THRESH = 167` 次（≈ 1.002 秒）未观测到 `recieve_frameID` 变化
  - **SW → HW**: 任意一次 PE9 下降沿触发 EXTI

## 5. 数据结构变更

`camera_Timematch.h` 在 `CameraTM` 类里新增：

```cpp
enum SyncMode : uint8_t {
    HW_SYNC = 0,
    SW_SYNC = 1,
};

static constexpr uint32_t MISS_THRESH = 167; // 166.67Hz * 1s

volatile SyncMode mode = HW_SYNC;
volatile uint32_t miss_count = 0;
volatile uint32_t last_recv_id_snapshot = 0;
```

注：
- 用 `volatile` 因为字段在 IRQ 和主循环之间共享
- `last_recv_id_snapshot` 是 TIM1 update IRQ 上一次看到的 `recieve_frameID` 值，用来判断这一拍是否真的有新 strobe
- 不动现有公开字段，下游代码零修改

## 6. 检测与抓拍逻辑

### 6.1 `TIM1_UP_TIM10_IRQHandler` 改造

```cpp
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        cameraTM.send_frameID++;

        // 1) Miss 检测：本周期内 PE9 是否带来新 strobe？
        uint32_t cur_recv = cameraTM.recieve_frameID;
        if (cur_recv == cameraTM.last_recv_id_snapshot) {
            if (cameraTM.miss_count < UINT32_MAX) {
                cameraTM.miss_count++;
            }
            if (cameraTM.miss_count >= CameraTM::MISS_THRESH) {
                cameraTM.mode = CameraTM::SW_SYNC;
            }
        } else {
            cameraTM.miss_count = 0;
            cameraTM.last_recv_id_snapshot = cur_recv;
            // 注意：不在这里切回 HW_SYNC，切回由 EXTI 边沿负责（避免 SW 自增触发误切回）
        }

        // 2) 软同步抓拍：在触发瞬间用 IMU 当前值
        if (cameraTM.mode == CameraTM::SW_SYNC) {
            cameraTM.camera_yaw   = gimbal.yawAngle;
            cameraTM.camera_pitch = gimbal.pitchAngle;
            cameraTM.camera_roll  = boardImu->FusionAngle.roll;
            cameraTM.recieve_frameID++;
            // 同步推进 snapshot，避免 SW 自增的递增被下一拍误判为"收到新 strobe"
            cameraTM.last_recv_id_snapshot = cameraTM.recieve_frameID;
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
```

### 6.2 `EXTI9_5_IRQHandler` 改造

```cpp
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
        cameraTM.recieve_frameID++;
        cameraTM.camera_yaw   = gimbal.yawAngle;
        cameraTM.camera_pitch = gimbal.pitchAngle;
        cameraTM.camera_roll  = boardImu->FusionAngle.roll;

        // 真实 strobe 到来 → 立即回 HW 模式
        cameraTM.mode       = CameraTM::HW_SYNC;
        cameraTM.miss_count = 0;
        // last_recv_id_snapshot 在 TIM1 IRQ 里下一拍刷新即可

        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}
```

## 7. 中断优先级正确性

当前优先级（已核对源码）：

| IRQ | Preempt | Sub |
|---|---|---|
| `TIM1_UP_TIM10_IRQn` | 0 | 1 |
| `EXTI9_5_IRQn` | 3 | 0 |

TIM1 update IRQ 抢占级数值更小 → 优先级**更高**，可以抢占 EXTI。

**关键时序分析**：在 TIM1 IRQ 执行期间 EXTI 无法插入，所以 `cur_recv = recieve_frameID` 在该 IRQ 内是稳定的。EXTI 只能在 TIM1 IRQ 退出前/后整段地运行，状态切换没有交错。

**唯一可能的小窗口**：在 SW 模式下，TIM1 IRQ 已读到 `cur_recv == last_recv_id_snapshot` 并准备做 SW 抓拍，与此同时 EXTI9 在 NVIC 里 pending。TIM1 IRQ 退出后 EXTI 才执行 → 这一拍会出现两次抓拍：先 SW（TIM1 IRQ 内）再 HW（EXTI），HW 后写覆盖前者。`recieve_frameID` 在这一拍会 +2。下游看到 frame ID 跳一格，可接受（一次性现象，发生在 HW 恢复瞬间）。

## 8. 不做的事情

- ❌ 触发→曝光延迟补偿（相机端 exposure offset 不可靠，简化处理）
- ❌ `mode` 上行 NUC（vision_Send_Struct 暂不动；本地保留为 public 字段，方便后续接 vofa）
- ❌ HW 回归迟滞（PE9 一来就切，避免短暂阻断后还要等几帧才恢复时间精度）
- ❌ H7 主线移植（H7 暂无 camera_Timematch 模块，等迁移时一起做）

## 9. 验证清单

实现后需要在板上验证（**用户负责执行**，电控测试不在我这里跑）：

1. **正常状态**：上电后 PE9 持续触发 → `cameraTM.mode == HW_SYNC`、`miss_count == 0`、`recieve_frameID` 持续递增
2. **断线注入**：手动拔掉 PE9 → ≈ 1 秒后 `mode` 切到 `SW_SYNC`、`miss_count` 达到 167、`camera_yaw/pitch/roll` 仍持续刷新、`recieve_frameID` 不冻结
3. **恢复测试**：重接 PE9 → 第一个下降沿立刻把 `mode` 拉回 `HW_SYNC`、`miss_count` 清零
4. **反复拔插**：连续拔插多次，状态机不卡死、`recieve_frameID` 单调递增
5. **NUC 端**：vision 上报弹道命中率在断线/恢复瞬间无明显跌落（断线 1 秒内会用陈旧 pose 是已知接受代价）

## 10. 代码影响面

| 文件 | 改动 |
|---|---|
| `top/TDT_Device/inc/camera_Timematch.h` | 加 `SyncMode` 枚举、`MISS_THRESH` 常量、`mode/miss_count/last_recv_id_snapshot` 字段 |
| `top/TDT_Device/src/camera_Timematch.cpp` | 改 `TIM1_UP_TIM10_IRQHandler` 和 `EXTI9_5_IRQHandler`，`init/run/构造函数` 不变 |
| 其他 | 无（不动 vision.cpp、vision.h、actuator_task.cpp、schedule.cpp、上行协议） |
