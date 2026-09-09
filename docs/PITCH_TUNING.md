# Pitch 接入与整定

[文档导航](README.md) · [项目首页](../README.md)

Pitch 和 Yaw 使用相同的滑模核心，但每轴独立配置惯量、前馈、反馈坐标、输出限制和参考约束。DM4310 与 GM6020 的适配不区分轴；是否能控制好 Pitch，取决于负载、有效力矩、测量和调参，不能仅凭报文兼容判断。

本文按“力矩是否够用 → 坐标是否正确 → 接入与限位 → 整定记录”展开。初次接入先读[三个角的区别](#2-分清三个角)和[官方 C 板接法](#3-官方-c-板框架的完整接法)；准备调参时按[第 5 节](#5-pitch-的调参顺序)执行。控制律推导见 [控制器设计](CONTROL_DESIGN.md)，电机协议见 [电机适配](MOTOR_PROTOCOL.md)。

**当前示例的范围是单轴、固定运动平面的 Pitch。** 它提供重力前馈和参考区间约束，没有实现折叠双 Pitch 的机构协调、三轴动力学或机械制动规划。多轴机构可以分别使用核心实例，轴间参考分配与耦合补偿需要上层提供。

原 576 组开源模型测试中有 288 组 Pitch，参考仅为零位保持和 ±0.1 rad 正弦。大行程升降另见 [Pitch 专项验证](PITCH_VALIDATION.md)，原矩阵继续作为回归；两者均不代表本项目已经完成实机调参。

## 1. 为什么 Pitch 需要单独调

采用与核心相同的符号：

$$
J\ddot q=\tau-B\dot q-G(q_g)+d.
$$

这里的 `q` 对应核心的控制角 `theta`，`G` 对应重力负载 `tau_g`，`q_g` 是重力相位。角度统一使用 rad，力矩统一使用关节侧 N·m。

静止时即使角度误差为零，也可能需要非零的 `G(q_g)` 持续支撑。重心偏移、弹量和附件改变会影响保持力矩；负载改变也可能同时改变惯量。不能用位置死区将这部分力矩清零。

在当前角度和速度处，忽略扰动时，两方向可用加速度分别为：

$$
a_{+}=\frac{\tau_{max}-B\dot q-G(q_g)}{J},\qquad
a_{-}=\frac{-\tau_{max}-B\dot q-G(q_g)}{J}.
$$

因此，对称力矩上限不代表两方向有相同的加速/制动能力。参考限制可先用保守的力矩预算筛查：

$$
|G|+J|\alpha_r|+B|\omega_r|+\tau_{reserve}<\tau_{available}.
$$

这里的余量用于反馈修正、摩擦和模型误差，`tau_available` 应来自当前机构可持续使用的力矩限制。此式只是可行性检查，不是稳定性或停车距离证明。实际饱和持续出现时，先检查配平与力矩能力，并配合降低参考速度、调整加速度限制，不能只提高滑模增益。单独降低减速度可能增加到达目标前的制动距离。

## 2. 分清三个角

| 信号 | 用途 | 不能混用的原因 |
|---|---|---|
| `axis.angle_rad` 与 `axis.rate_rad_s` | 被控制的角及其同坐标导数 | 惯性姿态角不能配上未变换的相对转速 |
| `gravity_angle_rad` | 重力模型的相位 | 描述重心相对重力的方向，底座倾斜后通常不等于编码器相对角 |
| `joint_relative_angle_rad` | 相对底座的机械行程 | 惯性 Pitch 为零不代表机构位于机械中位 |

例如同平面底座倾斜 `beta`，编码器相对角为 `q`，零位已对齐时，惯性控制角和重力相位可能为 `q+beta`，机械限位仍检查 `q`。这只是平面运动的例子；大幅 roll、yaw/pitch 耦合或不同轴安装需要由应用层根据姿态和重心计算实际重力投影。

[重力接口](../include/gimbal_pitch.h) `gimbal_pitch_gravity_torque()` 在固定运动平面中计算有符号保持力矩：

$$
G(q_g)=A\sin(q_g)+C\cos(q_g).
$$

`A`、`C` 对应 `gimbal_pitch_gravity_model_t` 中的 `sin_nm`、`cos_nm`，单位为关节 N·m，正方向与核心输出一致。零位偏差可以吸收进系数，或在计算重力相位时统一处理。不能一边标定系数，一边又重复加同一个零偏。`A=C=0` 可表示已配平的近似；函数拒绝非有限参数或角度，失败时清零输出并返回 `false`。

这个接口不是完整三维重力模型，也不直接把加速度计读数当作重力方向。若机构需要完整三维模型，可由应用层计算力矩后使用通用 `gimbal_smc_update()` 的 `feedforward_nm`。

## 3. 官方 C 板框架的完整接法

在 [官方反馈转换](STM32_PORT.md#2-复用官方反馈转换) 后，取得本拍一致的角度、角速度、机械相对角与时间戳。示例位于 [gimbal_pitch_example.h](../examples/gimbal_pitch_example.h) 和 [实现](../examples/gimbal_pitch_example.c)，在已有核心和电机适配之外，还需加入 [gimbal_pitch.c](../src/gimbal_pitch.c) 与 [gimbal_controller_example.c](../examples/gimbal_controller_example.c)。

根据上层已有的目标信息选择接口：

| 上层提供的内容 | 接入方式 | 应用层仍负责 |
|---|---|---|
| 单个位置目标 | `gimbal_pitch_example_step()` | 提供标定后的三个角、时间戳、工作模式和行程配置 |
| 同一时间戳的角度/角速度/加速度参考 | `gimbal_smc_update()` | 直接填入参考三元组，并自行完成行程监督与重力前馈 |
| 三维重力模型或耦合补偿力矩 | `gimbal_smc_update()` 的 `feedforward_nm` | 计算与受控轴同方向、同单位的关节力矩 |

下面的示例使用第一条路径。`gimbal_pitch_example_step()` 接受位置目标，内部生成速度和加速度；它不接收自瞄系统已有的参考三元组。

以下名称 `calibrated_*`、`*_qualified` 等由应用层提供，数值必须对应自己的机构；不要直接把开源模型参数当作上机默认值。

```c
#include "gimbal_pitch_example.h"

static gimbal_pitch_example_t pitch;

bool pitch_init(const gimbal_pitch_example_config_t *calibrated_config)
{
    /* Both smc.wrap_angle and reference.wrap_angle must be false. */
    return gimbal_pitch_example_init(&pitch, calibrated_config);
}

void pitch_control_tick(void)
{
    gimbal_pitch_example_snapshot_t sample = {0};
    gimbal_pitch_example_output_t result;
    sample.axis.angle_rad = gimbal_control.gimbal_pitch_motor.absolute_angle;
    sample.axis.rate_rad_s = gimbal_control.gimbal_pitch_motor.motor_gyro;
    sample.axis.feedback_age_s = oldest_control_feedback_age_s;
    sample.axis.dt_s = measured_dt_s;
    sample.axis.feedback_valid = coherent_pitch_feedback_qualified;
    sample.axis.enabled = mode_allows_pitch_motion;
    sample.joint_relative_angle_rad = calibrated_pitch_joint_angle_rad;
    sample.gravity_angle_rad = calibrated_gravity_phase_rad;
    sample.joint_feedback_age_s = pitch_encoder_age_s;
    sample.gravity_feedback_age_s = gravity_pose_age_s;

    (void)gimbal_pitch_example_step(&pitch, &sample,
                                   pitch_target_absolute_rad, &result);
    /* Check result.control.valid/flags AND result.pitch_flags.
     * Pass result.control to gimbal_example_dm_command(), or to
     * gimbal_example_gm_slot() under the single CAN group owner. */
}
```

DM 使用 MIT 纯力矩，GM 使用已确认的原生电流模式；选择哪种电机不改变重力补偿的关节单位。输出方向仅由约定的电机适配层反转一次，不能再叠加官方 `PITCH_TURN` 对该分支输出取反。该宏也用于编码器反馈和零位处理，不应整体关掉；保留统一坐标后的反馈。协议量程、传动和电流常数见 [电机说明](MOTOR_PROTOCOL.md)。

只有新姿态/编码器样本到达，才能更新时间戳；不能每次读取缓存就重置年龄。重力角、相对角和控制反馈三者的最大年龄参与超时检查。

## 4. 参考包络与机械停车分开验证

通用 `gimbal_reference_step()` 限制速度和加速度，**不保证生成参考相对位置目标无过冲**。其期望速度是 `clamp(position_gain * error, ±max_rate)`。以满速接近固定目标时，从限速段进入减速段的距离约为 `v_max / position_gain`，理想匀减速停车距离为 `v_max² / (2 a_max)`；若前者更小，就已有减速太晚的风险。可据此保守选择速度/加速度的组合，再用实际离散轨迹验证；这个筛查不是所有初态或移动目标下的无过冲证明，更不是电机可用制动力矩保证。

调试时同时记录原始目标、生成参考和实际角度。对生成参考的跟踪 RMS 很小，仍可能伴随生成参考自身明显越过原始目标；应另外统计目标过冲、持续进入误差带的时间和保持误差。降低最大速度通常增加大角度动作时间，这一取舍应明确记录。Yaw 同样需要这项检查。

Pitch 示例用当前快照的 `offset = control_angle - joint_relative_angle`，将机械软区间映射到控制坐标。目标先夹入该区间；参考生成后再次检查，必要时投影参考并同步清除参考速度/加速度，避免内部参考仍向外运行。目标和参考裁剪通过独立 `pitch_flags` 报告。有限行程配置拒绝角度 wrap。

这一步仅约束当前快照下的**生成参考**。边界投影可能中断原来的加速度连续性；底座快速运动会改变下一拍映射区间。它不保证真实机械不越界，也不计算制动距离。软限位留量必须覆盖实际速度、力矩余量、反馈/通信延迟以及基座运动，上层仍需负责运动可行性和限位监督。

正常在软边界保持时继续计算重力补偿，不按“靠近限位就把总力矩置零”处理。机械角超出硬范围会锁存故障；只有应用层明确处理后调用 `gimbal_pitch_example_reset()` 才能清除锁存，随后用有效的新反馈重建参考。不要在每个控制周期自动调用 reset 消除故障。

每拍同时检查 `result.control.valid`、`result.control.flags` 和 `result.pitch_flags`。后两者使用独立的标志位命名空间，不能混用位掩码。

| 状态 | 输出与处理 |
|---|---|
| `GIMBAL_PITCH_TARGET_CLIPPED` / `GIMBAL_PITCH_REFERENCE_CLIPPED` | 目标或生成参考经过裁剪；本身不表示控制器故障，同时记录 `clamped_target_rad` 和 `reference` |
| `GIMBAL_SMC_AMPLITUDE_LIMIT` / `GIMBAL_SMC_SLEW_LIMIT` | 力矩受限但可能仍有效；检查力矩预算与跟踪误差 |
| `GIMBAL_PITCH_HARD_LIMIT` / `GIMBAL_PITCH_FAULT_LATCHED` | 锁存并输出零；上层处理后显式复位，不能自动清除继续运动 |
| `result.control.valid == false` | 不复用上一拍力矩，执行上层故障或禁用策略 |

失能、反馈超时或硬越限后的零输出是退出驱动，**不是保持 Pitch 姿态**。不平衡机构可能失去支撑；掉线工况需要整机的配平、支撑/制动与故障策略。算法不能用陈旧姿态继续宣称可靠的重力保持。

## 5. Pitch 的调参顺序

1. **坐标和配平先确定。** 分别核对受控角、角速度、机械相对角和正力矩方向。检查上下行程；将机构重心尽量靠近转轴，给反馈留出力矩余量。
2. **先取得可用的重力前馈，再提高闭环带宽。** 在受控、可支撑的条件下记录多个安全角度的低速/静止保持力矩，拟合有符号 `A*sin(qg)+C*cos(qg)`。从两个方向到达同一角度，有助于区分重力与摩擦/回差；保留拟合残差。用更多角度检查模型，而不是只校准水平一点。
3. **估计负载惯量并设置运动限制。** 同一电机装在不同 Pitch 上，`J` 和允许加速度都可能不同。用上述力矩预算检查两侧行程，尤其是保持力矩较大的一侧。补偿与惯量同时受新增附件影响时，应分别更新。
4. **从线性面开始。** 保持 `terminal_gain=0`，从低带宽的 `lambda,k` 起步。按照实测陀螺噪声选 `phi`，再用残余扰动力矩与 `J` 的比值评估 `eta`；不能照抄另一轴的数字。增益变大后同时检查位置误差、输出变化、机械声音和温升。
5. **单独检查建立支撑的过程。** `torque_slew_nm_s` 限制的是含重力项的总力矩，控制状态复位后从零起步。忽略反馈修正时，建立保持力矩至少需要约 `|G|/slew` 秒；过小 slew 会造成启用下坠，过大又可能产生冲击，需结合上层使能顺序和机构验证。
6. **按方向和位置看结果。** 分别记录增角运动、减角运动、下/中/上位置保持，以及偏载、底座倾斜和恢复过程。最后再判断是否需要终端项、摩擦补偿或扰动观测器。

源模型角度正方向不一定等于直观的“抬头”。仿真以 `q` 增大/减小分组；上机记录应注明自己的抬头正方向。上述流程与参数预算不替代实际 STM32 时序、CAN 和机械测试。
