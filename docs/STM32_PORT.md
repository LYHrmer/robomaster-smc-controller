# 接入 RoboMaster 官方 C 型开发板例程

[文档导航](README.md) · [工程结构](PROJECT_STRUCTURE.md) · [项目首页](../README.md)

初次接触本库，可先按 [快速开始](QUICKSTART.md) 完成主机运行、接口选择和源文件配置，再回到本页修改固件接入点。

优先参考官方 [19.gimbal_task](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/19.gimbal_task/application/gimbal_task.c) 和 [20.standard_robot](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/gimbal_task.h)。保留官方的模式选择、遥控/自瞄输入、INS 与机械相对角处理，在选定的闭环模式中替换控制计算和电机输出路径。

本页提供修改位置和桥接代码；未针对某一块实际板卡生成完整 Keil 工程，也未完成固件链接、烧录及 CAN 台架验证。不同版本的官方例程需按实际字段核对。

## 1. 加入源文件

将 `src/gimbal_smc.c`、`src/gimbal_motor.c` 及需要时的 `examples/gimbal_controller_example.c` 加入工程，添加 `include/`、`examples/` 头文件路径。采用 Pitch 专用接入时再加入 `src/gimbal_pitch.c` 和 `examples/gimbal_pitch_example.c`。Keil/IAR 开启对应的 C99 支持；GCC 链接 `libm`。

需要在线观察模型参数时，再加入 `src/gimbal_rls.c` 与 `src/gimbal_identification.c`。两者均为纯 C99；只有应用调用时才进行辨识，不依赖电脑端 Python 工具。CMake 中的 `GIMBAL_BUILD_IDENTIFICATION` 仅控制可选离线测试，不控制这两个 C 模块；完整库依赖见 [工程结构](PROJECT_STRUCTURE.md)。

通用位置目标接入每轴使用一个静态 `gimbal_example_axis_t`；Pitch 专用接入使用 `gimbal_pitch_example_t`，直接调用核心则使用 `gimbal_smc_t`。Yaw、Pitch 分开提供各自配置。上电只初始化对象，不在初始化算法时自动发送电机使能。默认参数是示例，需替换机构惯量、力矩上限和参考限制。

STM32F4 的 Cortex-M4F 浮点选项必须与整个固件一致；库和固件不能混用 hard-float/soft-float ABI。M7/H7 按实际芯片调整。禁止 `-ffast-math`，保留 NaN/Inf 检查。

## 2. 复用官方反馈转换

官方 `gimbal_feedback_update()` 中：

| 字段 | 官方所用信号 |
|---|---|
| Pitch `absolute_angle` | INS Pitch |
| Pitch `motor_gyro` | 陀螺 Y 轴 |
| Yaw `absolute_angle` | INS Yaw |
| Yaw `motor_gyro` | `cos(pitch.relative_angle)*gyroZ - sin(pitch.relative_angle)*gyroX` |

优先读取该函数本拍更新后的 `absolute_angle`、`motor_gyro`；官方类型使用 rad、rad/s。不要根据另一份战队框架的轴号重新接线。上述投影仍依赖官方机械和 IMU 安装约定，大角度姿态运动中需检查实际受控角导数与陀螺投影是否一致。

电机编码器相对角继续用于零偏、限位、多圈和底盘相对方向判断；惯性闭环的 `absolute_angle` 不直接代替 Pitch 机械限位角。若将滑模接入相对角控制模式，必须换成相同坐标中的相对角及其导数。

反馈年龄不能每次控制任务读到数据就清零。INS 发布一份新姿态/角速度时记录其采样/更新时间，CAN 收到一份有效电机反馈时记录对应时间，控制拍使用所需反馈中最大的年龄。若只有角度陈旧而陀螺更新，也不能仅根据陀螺时间判定正常。

中断/INS 任务向控制任务发布快照时，用短临界区、双缓冲或有界重试的序列号机制，保证角度、角速度、状态和时间戳来自一致版本。不要仅给结构体加 `volatile` 就认为实现了多字段一致性。

## 3. 修改控制周期与模式接入点

官方示例循环大致为模式选择、模式转换、反馈更新、目标更新、闭环、方向处理、统一 CAN 发送、`vTaskDelay(1)`。`vTaskDelay(1)` 是一个 RTOS tick，不能假定等于 1 ms，也不保证任务开始间隔严格恒定。

建议使用匹配目标频率的定时器通知或周期调度，并通过 DWT/硬件定时器计算实际间隔。DWT 32 位周期计数的差分可使用无符号减法处理一次回绕；首次运行只建立时间基准。计数器在长时间暂停后可能多次回绕，上层应按失活状态重新建立基准。

```c
/* DWT must already be enabled; SystemCoreClock must match actual HCLK. */
uint32_t now_cycles = DWT->CYCCNT;
uint32_t elapsed_cycles = now_cycles - last_cycles;
last_cycles = now_cycles;
float dt_s = (float)elapsed_cycles / (float)SystemCoreClock;
```

进入新控制模式时，先取得本拍有效反馈，再将参考锚定到该角度。不要在官方旧的“模式转换先于反馈更新”位置直接用陈旧角度初始化新参考。最小替换方式如下，名称 `yaw_snapshot` 等是本页示意的应用变量：

```c
gimbal_smc_output_t yaw_out;
gimbal_example_snapshot_t yaw_snapshot = {
    .angle_rad = gimbal_control.gimbal_yaw_motor.absolute_angle,
    .rate_rad_s = gimbal_control.gimbal_yaw_motor.motor_gyro,
    .feedback_age_s = oldest_yaw_feedback_age_s,
    .dt_s = dt_s,
    .feedback_valid = yaw_feedback_qualified,
    .enabled = mode_allows_yaw_motion
};
gimbal_example_axis_step(&yaw_axis, &yaw_snapshot,
                        yaw_target_rad, 0.0f, &yaw_out);
```

Pitch 可使用独立的 `gimbal_pitch_example_step()`：接收控制角、重力相位、机械相对角及其反馈年龄，计算保持力矩并约束当前快照下的参考范围。完整可编译接口和官方框架调用见 [Pitch 接入与整定](PITCH_TUNING.md#3-官方-c-板框架的完整接法)。它不替代机械制动与上层限位监督；需要完整三维重力模型时仍可向通用核心传入应用层计算的前馈。

只有一个闭环能在当前模式驱动同一电机；进入 SMC 分支后不要再对其 N·m 输出串接旧速度 PID。`given_current` 是协议整数容器，不要把浮点力矩直接截断写进去。

上层有解析参考速度/加速度时，直接使用 `gimbal_smc_update()`，避免重复经过位置跟踪器。目标丢失、遥控失联、机械越限等应在模式管理器中明确定义保持、回中或禁用行为。

## 4. 电机使能与输出所有权

### DM4310

上层维护 `DISABLED -> ARMING -> RUNNING`，故障进入 `FAULT`。进入 ARMING 时显式发送一次 FC，等待此后收到的新鲜 `state=1` 反馈；超时进入 FAULT。ARMING 中调用 `gimbal_example_dm_command()` 返回 `NOT_ENABLED` 和 `dlc=0`，应不发送，避免尚未收到反馈就用 FD 取消使能。

RUNNING 中该 helper 根据有效力矩组帧，控制异常生成 FD；DISABLED/FAULT 生成 FD。实际 CAN 发送状态仍由上层检查。不要周期性 FC/FD 对发，也不要故障一恢复就自动重使能。DM 对称 12 位力矩没有精确零码，因此停止示例使用失能帧。

### GM6020

确认固件及原生电流环已经配置，再把 `current_mode_confirmed` 设为 1。将每轴关节力矩经过 `gimbal_gm6020_torque_to_current()` 或示例 `gimbal_example_gm_slot()` 写到本轴槽位，然后按 CAN 总线和组统一发送一次：

```c
/* This array belongs to the CAN group owner, which fills ALL active slots
 * with this cycle's commands. Example: slots 0/1 are two GM6020 axes. */
int16_t group_1_to_4[4] = {0, 0, 0, 0};
gimbal_can_frame_t frame;
gimbal_motor_status_t ys = gimbal_example_gm_slot(&yaw_gm, &yaw_out, group_1_to_4);
gimbal_motor_status_t ps = gimbal_example_gm_slot(&pitch_gm, &pitch_out, group_1_to_4);
/* Fill slots 2/3 here if they belong to other active GM6020 motors.
 * Check ys/ps and apply the group's fault policy before committing. */
if (ys >= 0 && ps >= 0 &&
    gimbal_gm6020_pack_current_group(1, group_1_to_4, &frame) >= 0) {
    /* Submit via the single CAN owner and check its return status. */
}
```

不要从每轴各发一份“其他槽为零”的组帧。输出保留在缓冲区也不代表该槽本周期有效；发送所有者应检测未更新/过期的轴并按其故障策略清零。上例假定配置的 motor_id 属于 1–4；ID 5–7 应使用另一组且保留第 4 槽为零。

旧官方 `CAN_cmd_gimbal()` 的电压组 ID 不适合新的电流模式。若原组中还包含拨弹或其他类型电机，必须按各自协议重新分组，不能整体迁移到 `0x1FE`。同时规划反馈 ID：例如 GM6020 的 `0x205` 与同总线上其他 DJI 电机的反馈可能冲突。主机库无法代替整车 CAN 地址规划。

输出方向只反转一次：在 SMC 分支采用电机 `mechanics.direction` 后，绕过官方 `YAW_TURN`/`PITCH_TURN` 对该分支输出的再次取反。保留已经校准到统一正方向的编码器反馈变换；不要为绕过输出反转而直接关闭这些也参与反馈/零位处理的宏。其他保留的旧模式仍按自身方向约定处理。

## 5. bxCAN HAL 发送桥

以下代码仅依赖 STM32F4 HAL 和本库，可放到工程自己的 CAN 发送模块。它不分配邮箱所有权，也不等待总线完成。

```c
#include "stm32f4xx_hal.h"
#include "gimbal_motor.h"
#include <string.h>

HAL_StatusTypeDef gimbal_bxcan_submit(CAN_HandleTypeDef *can,
                                     const gimbal_can_frame_t *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t payload[8];
    if (can == NULL || frame == NULL || frame->id > 0x7ffu || frame->dlc != 8u)
        return HAL_ERROR;
    if (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0u) return HAL_BUSY;
    header.StdId = frame->id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8u;
    header.TransmitGlobalTime = DISABLE;
    memcpy(payload, frame->data, sizeof(payload));
    return HAL_CAN_AddTxMessage(can, &header, payload, &mailbox);
}
```

HAL_OK 表示提交成功，不等于电机已执行。HAL_BUSY、bus-off、发送错误和长队列应报告给上层；禁止在 1 kHz 任务里无限等待邮箱。恢复总线时不能继续发送队列里过时的非零力矩。FDCAN 需要独立桥接其帧头、DLC 编码和 Classic CAN 数据帧选项，不能直接替换函数名套用 bxCAN。

接收入口验证标准 DATA 帧及 DLC=8 后再调用本库解码函数；将有效状态与本地时间戳一起发布。不要把解析失败后的零结构体当成“电机已停止”的反馈。

## 6. 上板需要记录的结果

先完成方向、模式、单位、零位与低力矩响应确认，再提升闭环带宽。至少记录：

- 控制周期最小/最大值、截止时间超限次数；线性与终端分支在真实中断负载下的 DWT 周期数。
- INS/电机反馈年龄、CAN 提交失败及 bus-off 次数、每总线各 ID 的发送所有者。
- 目标与实际角度/角速度、滑模面、原始/限幅后力矩、量化发送值、限幅比例及温度。
- 进入模式、ARMING 超时、遥控失联、自瞄丢失、机械限位、反馈丢帧和恢复期间的状态与实际发帧记录。

最小移植改动集中在源文件列表、两个控制器实例、闭环分支、模式转换后的参考复位，以及电机分组/发送接口。本文不要求迁移到其他战队整车框架。

## 7. 可选的在线参数观察

每轴、每个固定构形静态分配一个 `gimbal_identification_t`，在初始化时提供模型初值、物理边界和窗口配置。详细接口与示例见 [在线辨识](ONLINE_IDENTIFICATION.md)；电脑端日志验证见 [离线辨识](IDENTIFICATION.md)。原模式、控制计算与 CAN 发送仍各自拥有原来的职责。

| 固件接入点 | 需要提供或处理的内容 |
|---|---|
| 新反馈快照发布处 | 连续关节角及其导数、重力相位、已标定且同步的实际关节力矩、采集时间戳与反馈年龄 |
| 辨识任务 | 按采集顺序消费新快照；用 `operating_conditions_valid` 等字段声明当前固定底座/构形、标定与同步条件成立 |
| 模式与数据中断处 | 数据断续、坐标重定位时更换 `segment_id` 或清观测；新构形使用对应实例或明确重新初始化 |
| 记录与参数配置处 | 保存模型、创新指标、构形 ID；独立验证后由应用显式采纳，`ready` 本身不会改变 SMC |

`timestamp_us` 的单位是微秒，不能直接填 DWT 周期数。重复读取缓存不能冒充新样本；后台队列丢样、超时或阶段变化必须反映到数据状态，让转换层断开积分窗口。微秒时间戳支持自然回绕，相邻时间的先后判定要求间隔小于 `2^31` 微秒，约35.8分钟。停止采样超过该区间时，先调用 `gimbal_identification_clear_observations()` 保留参数/P并重设时基；仅更换 `segment_id` 不能消除这种时间歧义。

固定模型不适用时停止积累辨识数据，保留控制器当前配置。尤其不能将正在折叠、底座明显运动或未经标定的电流指令声明为合格的单轴动力学样本。辨识计算与诊断记录也需要在实际中断和任务负载下测量执行时间；现有代码和主机测试不提供 MCU 截止时间保证。

本次 Cortex-M4F 构建中，完整 `gimbal_identification_t` 状态为 1328 B，已包含内部 RLS。`.su` 文件给出的 `gimbal_rls_update()` 自身栈帧为 800 B，转换层 `update()` 自身为 136 B；它们都不是任务总栈需求。分配任务栈时需要保留整个调用链、数学函数、中断嵌套和上下文保存的开销，并在实际固件中检查栈峰值。工具链与测量口径见 [验证记录](VALIDATION.md)。
