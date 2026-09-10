# 快速开始

[文档导航](README.md) · [项目首页](../README.md)

本项目提供 STM32 可用的 C99 控制器、电机适配和接入示例。原云台工程继续负责模式管理、INS 模块提供的姿态与角速度、手瞄/自瞄目标及 CAN 收发。

## 1. 先在电脑上运行

需要 C99 编译器、CMake 3.16+ 和构建工具。Python 3.9+ 用于模型来源检查；运行控制器和在线 C 模块不需要 Python。

在仓库根目录依次执行，无需连接电机：

```sh
# 配置 Debug 构建，生成 build/ 目录
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# 编译 C 库、接入示例和测试程序
cmake --build build --parallel

# 运行测试；失败时显示对应输出
ctest --test-dir build --output-on-failure
```

**成功标志：** 前两条命令无错误退出，CTest 显示 `100% tests passed`。默认配置找到 Python 时共有 **13 项测试**；未找到时为 12 项，少的是模型来源检查。合成仿真生成的轨迹位于 `build/simulation.csv`，当前执行结果与报告见 [验证记录](VALIDATION.md)。

这一步检查主机上的代码和模型；STM32 执行时间、CAN 负载和实机参数仍需在自己的工程中验证。

## 2. 根据目标信号选择接口

控制器需要目标角度、角速度、角加速度。只有目标角度时，可以由接入示例生成另外两项；文档中将这三项合称为“参考三元组”。

| 你已有的输入 | 使用哪个入口 | 每轴保存的对象 |
|---|---|---|
| 只有目标角度，需要生成速度/加速度参考 | [`gimbal_example_axis_step()`](../examples/gimbal_controller_example.c) | `gimbal_example_axis_t` |
| 目标角度、速度、加速度已经齐全，例如自瞄轨迹 | [`gimbal_smc_update()`](../include/gimbal_smc.h) | `gimbal_smc_t` |
| 有限行程 Pitch，已有机械相对角及标定后的平面重力模型 | [`gimbal_pitch_example_step()`](../examples/gimbal_pitch_example.h) | `gimbal_pitch_example_t` |

**每根轴独立初始化、独立配置。** 可运行用法见 [通用接入测试](../tests/test_example.c) 和 [Pitch 接入测试](../tests/test_pitch.c)；示例与默认参数用于演示，应替换为自己机构的配置。

### Yaw：先确定是否保留圈数

- **连续多圈 Yaw：** 控制角、目标角均使用连续展开角；控制器与参考生成器都设置 `wrap_angle=false`。这样 `0 → 4π` 表示转两圈。
- **最短路径 Yaw：** 无需保留目标圈数时，两处均设置 `wrap_angle=true`。例如 `179° → -179°` 按跨越边界的短路径处理。

库不自动完成编码器多圈展开。若大 Yaw 要保留连续角，而视觉只给 `[-π,π]` 目标，应由应用层将目标映射到当前圈附近，再交给连续角控制。不要对手瞄的多圈目标也做最短路径处理。

### Pitch：控制姿态、机械角和重力角分别提供

**控制角用于跟踪目标，机械相对角用于行程限制，重力角用于计算支撑力矩。** Pitch 示例要求同平面、同正方向，控制角与机械角可按偏置换算；控制器和参考生成器的 `wrap_angle` 都为 `false`。参考约束与越限处理见 [Pitch 接入与整定](PITCH_TUNING.md)。

已有自己的重力模型，或已有完整自瞄三元参考时，可直接调用核心，把关节保持力矩填入 `feedforward_nm`，由应用层继续处理行程和轨迹约束。参考被裁剪不等于真实机构已经完成制动。

多轴可以增加独立实例。**折叠双 Pitch、大小 Yaw 的目标分配、构形判断和耦合补偿需要另行实现**；现有 Pitch 示例不是三轴或折叠机构控制器。

## 3. 加入自己的 C99 工程

| 需要的功能 | 加入工程的源文件 | 头文件目录 |
|---|---|---|
| 单轴 SMC 与参考生成器 | `src/gimbal_smc.c` | `include/` |
| DM4310 / GM6020 力矩换算、报文编解码 | 另加 `src/gimbal_motor.c` | `include/` |
| 通用位置目标接入与电机输出示例 | 再加 `examples/gimbal_controller_example.c` | `include/`、`examples/` |
| 完整 Pitch 接入示例 | 在上述基础上加 `src/gimbal_pitch.c`、`examples/gimbal_pitch_example.c` | `include/`、`examples/` |

上述 C 模块无 HAL、RTOS、C++ 或堆内存依赖。GCC 链接数学库 `libm`，Keil/IAR 开启 C99 支持；MCU 与库的浮点 ABI 必须一致。**不要启用 `-ffast-math`**，以保留 NaN/Inf 检查。模块依赖及交叉编译入口见 [工程结构](PROJECT_STRUCTURE.md)。

接线前先统一以下约定：

| 数据 | 本库要求 |
|---|---|
| 角度、角速度、角加速度 | `rad`、`rad/s`、`rad/s²`；角速度是受控角在同一坐标下的导数 |
| 力矩、惯量 | 关节侧 `N·m`、`kg·m²`；传动换算交给电机适配层 |
| `dt_s` | 实测控制间隔，单位秒；需落在控制器及参考生成器配置的范围内 |
| `feedback_age_s` | 本拍所需反馈中最旧样本的年龄，单位秒；读缓存不能刷新时间戳 |
| 有效性与使能 | 角度、速度、状态和时间戳作为一致的反馈快照传入；模式管理决定是否允许驱动 |

一个实例由一个任务更新。进入控制模式或从故障恢复时，以本拍有效反馈建立参考起点；通用示例已处理，直接调用核心时由应用负责。

调用后检查 `output.valid` 和 `output.flags`。无效时核心力矩为零，上层仍需发送停止命令；有效时也可能限幅，应记录标志。

## 4. 按官方 C 板流程接入

保留原有模式管理及遥控/自瞄目标入口，按以下顺序接入：

1. **更新反馈。** 在 `gimbal_feedback_update()` 完成本拍转换后，读取角度、角速度、机械相对角及时间戳。先核对实际 IMU 安装与方向约定。
2. **选择目标来源。** 手瞄与自瞄进入同一轴控制器前统一单位与坐标；切换来源时明确是否重建参考、如何处理目标丢失。
3. **计算关节力矩。** 按上表选一个控制入口；同一电机当前模式只由一套闭环生成输出。
4. **换算电机报文。** 按下表确认电机模式，检查适配函数返回值；输出方向只反转一次。
5. **统一发送。** 由现有 CAN 发送模块提交命令、检查发送状态，并处理未更新或过期的轴输出。

| 电机模式 | 本库输出路径 | 接入时的关键区别 |
|---|---|---|
| DM4310 MIT 纯力矩 | `gimbal_dm4310_pack_torque()`，`kp=kd=0` | 核对实际 PMAX/VMAX/TMAX、CAN/MASTER ID；上层显式使能并等待新鲜使能反馈 |
| GM6020 原生电流 | `gimbal_gm6020_torque_to_current()` → `gimbal_gm6020_pack_current_group()` | ID 1–4 用 **`0x1FE`**，ID 5–7 用 **`0x2FE`**；先确认固件与电流环设置 |
| GM6020 旧电压 | 仅提供 `gimbal_gm6020_pack_voltage_group()` 原始组帧 | 使用 **`0x1FF` / `0x2FF`**；本库未提供额外电流内环，不能直接把 N·m 当电压指令 |

GM 同一组的全部槽位由一个发送者汇总，不能每轴各发一帧并清掉其他轴。旧官方 `CAN_cmd_gimbal()` 也不能原样接收新的电流指令。

具体修改位置、使能过程和 HAL 桥接见 [STM32 移植步骤](STM32_PORT.md)，协议依据见 [电机说明](MOTOR_PROTOCOL.md)。

## 5. 可选：获取物理参数

**辨识**是从数据估计惯量、摩擦、重力等物理参数。它们与 SMC 的控制增益不同；已有标定配置时，可以直接使用控制器。

先按电机选择数据采集与标定说明：[GM6020 辨识指南](GM6020_IDENTIFICATION.md) 或 [DM4310 辨识指南](DM4310_IDENTIFICATION.md)。GM6020 的反馈电流原始值与 DM4310 的驱动力矩报告需要分别核验；完成关节力矩标定、坐标转换和时间对齐后，两种电机共用下列工具。

| 路径 | 运行位置 | 入口与结果 |
|---|---|---|
| 在线拟合 | STM32，纯 C99 | [`gimbal_identification`](ONLINE_IDENTIFICATION.md) 将新采样形成积分窗口，由递推最小二乘（RLS）逐次更新参数 |
| 离线辨识 | 电脑，Python/NumPy | [`identify_gimbal.py`](IDENTIFICATION.md) 读取独立训练/验证日志，导出报告和候选 C 常量 |

“候选参数”表示**尚待验证、没有自动写入控制器的估计值**。在线 `ready` 只表示本次数据满足候选条件；参数验证、采纳和配置切换由应用处理，不自动调整控制增益。

当前两条云台辨识路径要求固定底座、固定构形及同步标定的关节力矩。Yaw 拟合惯量 `J`、黏性阻尼 `B`、库仑摩擦力矩 `Fc`；Pitch 再拟合重力系数 `A/C`。行驶中、折叠中和三轴耦合不属于该模型范围。

接入在线模块时，加入 `src/gimbal_rls.c`、`src/gimbal_identification.c` 和 `include/` 路径，无需 Python。常规 CMake 构建已包含它们；`GIMBAL_BUILD_IDENTIFICATION=ON` 仅增加电脑端离线检查，完整环境共 **15 项 CTest**。依赖和运行命令见 [离线辨识](IDENTIFICATION.md)，在线采样与候选使用见 [在线拟合](ONLINE_IDENTIFICATION.md)。
