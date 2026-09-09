# RoboMaster 框架接口对照

核对日期：2026-09-09。本项目优先沿用官方 C 板例程的模式管理和反馈更新，再把本库接在明确的物理量接口上。下面的官方与社区工程用于核对架构和坐标语义；没有导入这些工程的控制代码，也不把本库静态库编译通过当作它们的整车固件或实机兼容性证明。

## 来源、采纳点与移植边界

| 固定版本与文件 | 源码所做的事 | 本库采纳点 | 需要单独适配的部分 |
|---|---|---|---|
| [官方 C 板 `gimbal_task.c`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c) | 更新姿态／机械相对角／轴向速度；`gimbal_absolute_angle_limit()` 通过相对角、当前绝对目标偏差及新增目标量判断行程 | 保留模式与反馈处理；机械角约束应映射至当前控制角坐标，不能直接把机械限值当作 IMU 绝对角限值 | 该版本轴变换与安装方向只适用于相应结构；旧 CAN 云台输出也不等同于本库 GM6020 原生电流组 |
| [HNUYueLuRM `gimbal.c`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/gimbal/gimbal.c) | 电机实例绑定外部角度、速度反馈；该版本 pitch 绑定 `Pitch` 与 `Gyro[0]`，重力补偿仅留有待实现注释 | 把模式、反馈来源与具体电机实例分开；每轴参数和坐标分别核对 | 不能套用其它工程的 `Gyro[1]` 或照抄重力参数；本次未追完其 QEKF 单位链，故不直接宣称 `Pitch` 的角度单位 |
| [HNUYueLuRM `dji_motor.c`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/DJImotor/dji_motor.c) | 按配置依次运行角度／速度／电流 PID，期间插入相应前馈；集中填充 CAN 组槽，GM6020 分组使用 `0x1FF/0x2FF` | 保留一个发送者管理同组槽位；替换控制器时明确保留哪些内环 | `DJIMotorSetRef()` 或前馈指针不是天然的 N·m 接口；原 PID 若仍开启，SMC 输出会再次被解释或计算，不能直接接入 |
| [rm-controls `gimbal_base.cpp`](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/src/gimbal_base.cpp) | `setDes()` 先将目标变换至 base 坐标再检查关节限位；`gravityFeedForward()` 将重力向量变换到关节 child 坐标，以力臂叉乘求分量，并在 effort 命令层加入前馈 | 区分目标姿态、机械行程、重力方向和最终力矩输出；前馈应在有明确物理含义的层级相加 | ROS／tf2／URDF 的完整变换和关节接口不能直接搬入 STM32；本库固定平面 `sin/cos` 重力 helper 只覆盖其声明的简化模型 |

上述做法共同说明：可移植的不只是控制器公式，还包括“当前反馈是什么量、目标是什么量、前馈加在哪里、谁最终发送电机命令”。

## 官方例程中的机械角与控制角

官方 `gimbal_absolute_angle_limit()` 使用的偏差为绝对角目标与当前绝对角之差，并作角度格式化。随后检查“当前机械相对角 + 该目标偏差 + 本次目标增量”是否超过机械区间，只修正会继续越界的目标增量。它体现的是**根据当前姿态把目标映射到机械行程**，不是把一组绝对姿态数值直接固定为机械极限。[对应源码](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L879)

本库 [Pitch 接入示例](../examples/gimbal_pitch_example.h)分别接收控制角、重力角和机械角，并维护独立参考状态。这种分离使底座姿态变化时仍有地方表达正确的参考与限位关系；其目标包络和故障处理仍需要结合实际行程、制动能力及结构验证。它没有复制官方函数，也没有声称目标裁剪能保证任何初始速度下都不撞硬限位。

## 替换闭环时先确定输出接口

HNU 的电机控制路径会按开关继续运行角度和速度 PID。若把 N·m 输出写入仍被当作角度、速度或电流给定的字段，会改变控制量含义。它的速度前馈与电流前馈插入位置也不同，必须连同闭环开关、单位、驱动模式和符号一起核查。[对应控制路径](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/DJImotor/dji_motor.c#L238)

rm-controls 在 pitch 控制器更新之后，给 effort joint 的命令加重力前馈；其重力项又在关节 child 坐标中根据向量叉乘计算。可以借鉴“先完成坐标变换，再在力矩层相加”的职责划分，但其参数与变换链不能用本库的两个固定系数直接替代。[输出相加位置](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/src/gimbal_base.cpp#L510)，[重力计算](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/src/gimbal_base.cpp#L552)

本库接口约定为：应用提供同步的角度／角速度及参考三元组，`gimbal_smc_update()` 输出关节 N·m；电机层按已经确认的模式和标定转换，再由一个 CAN 组所有者提交。GM6020 新版原生电流组与旧电压组的差别详见 [电机协议说明](MOTOR_PROTOCOL.md)，不得只依据变量名 `current` 或“前馈”就假定已经具备 N·m 接口。

## 当前覆盖与后续扩展

Yaw、Pitch 使用不同控制实例、参数和参考状态。应用可以增加其它轴实例，但完整的三轴姿态分配、运动学转换、耦合补偿与同步时序仍需在应用层实现；目前没有进行三轴联动验证。

已实现接口和接入步骤见 [STM32 移植说明](STM32_PORT.md)、[Pitch 整定](PITCH_TUNING.md)。模型闭环证据见 [Pitch 专项报告](PITCH_VALIDATION.md)；本轮 Arm 静态库编译及主机回归范围见 [验证记录](VALIDATION.md)。
