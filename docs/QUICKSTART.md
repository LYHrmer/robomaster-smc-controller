# 快速开始

[文档导航](README.md) · [项目首页](../README.md)

本项目提供 **STM32 可用的 C99 控制器、电机协议适配和接入示例**。你仍使用自己的云台工程提供模式管理、INS、遥控/自瞄目标及 CAN 收发。这里从主机运行开始，再说明如何接入官方 C 型开发板例程。

## 1. 先在电脑上运行

准备 C99 编译器、CMake 3.16+ 和构建工具；安装 Python 3.9+ 可同时运行模型来源检查。在仓库根目录执行：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/simulate build/simulation.csv
```

前三步编译实际 C 核心、适配层、在线拟合模块和例程，并运行软件回归及模型仿真；最后一步单独生成 `build/simulation.csv`，便于查看轨迹。默认配置安装了上述 Python 时注册 **13 项 CTest**，未找到 Python 时不注册来源检查。运行后检查各项是否通过，当前执行记录见 [验证记录](VALIDATION.md)。

不需要连接电机。已有图件和结果解释见 [验证记录](VALIDATION.md)、[开源模型验证](OPEN_MODEL_VALIDATION.md) 和 [Pitch 专项验证](PITCH_VALIDATION.md)。主机测试通过后，STM32 执行时间、CAN 负载和实机参数仍需在自己的工程中验证。

## 2. 根据目标信号选择接口

| 你已有的输入 | 使用哪个入口 | 每轴保存的对象 |
|---|---|---|
| 只有目标角度，需要生成速度/加速度参考 | [`gimbal_example_axis_step()`](../examples/gimbal_controller_example.c) | `gimbal_example_axis_t` |
| 目标角度、速度、加速度已经齐全，例如自瞄轨迹 | [`gimbal_smc_update()`](../include/gimbal_smc.h) | `gimbal_smc_t` |
| 有限行程 Pitch，已有机械相对角及标定后的平面重力模型 | [`gimbal_pitch_example_step()`](../examples/gimbal_pitch_example.h) | `gimbal_pitch_example_t` |

**每根轴独立初始化、独立配置。** 现有可运行用法见 [通用接入回归](../tests/test_example.c) 和 [Pitch 接入回归](../tests/test_pitch.c)；其中参数用于测试，应替换为自己机构的参数。默认 SMC 参数也不是 DM4310 或 GM6020 的上机参数表。

### Yaw：先确定是否保留圈数

- **连续多圈 Yaw：** 控制角、目标角均使用连续展开角；控制器与参考生成器都设置 `wrap_angle=false`。这样 `0 → 4π` 表示转两圈。
- **最短路径 Yaw：** 无需保留目标圈数时，两处均设置 `wrap_angle=true`。例如 `179° → -179°` 按跨越边界的短路径处理。

库不自动完成编码器多圈展开。若大 Yaw 要保留连续角，而视觉只给 `[-π,π]` 目标，应由应用层将目标映射到当前圈附近，再交给连续角控制。不要对手瞄的多圈目标也做最短路径处理。

### Pitch：控制姿态、机械角和重力角分别提供

Pitch 示例用于同平面、同正方向、控制角与机械角可按偏置换算的机构。它接收三个不同用途的角，约束当前参考范围，并提供硬越限故障锁存；详见 [Pitch 坐标与整定](PITCH_TUNING.md)。控制器和参考生成器的 `wrap_angle` 都必须为 `false`。

已有自己的重力模型，或已有完整自瞄三元参考时，可直接调用核心，把关节保持力矩填入 `feedforward_nm`，由应用层继续处理行程和轨迹约束。参考被裁剪不等于真实机构已经完成制动。

多轴可以增加独立实例。**折叠双 Pitch、大小 Yaw 的目标分配、构形判断和耦合补偿需要另行实现**；现有 Pitch 示例不是三轴或折叠机构控制器。

这些接口负责控制输出。若要从反馈中获取物理参数，再按本页末尾的 [可选辨识入口](#5-可选获取物理参数) 加入独立模块。

## 3. 加入自己的 C99 工程

| 需要的功能 | 加入工程的源文件 | 头文件目录 |
|---|---|---|
| 单轴 SMC 与参考生成器 | `src/gimbal_smc.c` | `include/` |
| DM4310 / GM6020 力矩换算、报文编解码 | 另加 `src/gimbal_motor.c` | `include/` |
| 通用位置目标接入与电机输出示例 | 再加 `examples/gimbal_controller_example.c` | `include/`、`examples/` |
| 完整 Pitch 接入示例 | 在上述基础上加 `src/gimbal_pitch.c`、`examples/gimbal_pitch_example.c` | `include/`、`examples/` |
| 在线云台参数拟合（可选） | `src/gimbal_rls.c`、`src/gimbal_identification.c` | `include/` |

仅需重力计算函数时，`gimbal_pitch.c` 也可以独立使用；SMC 不依赖在线拟合模块。上述 C 模块不依赖 HAL、RTOS、C++ 或堆内存；GCC 链接数学库 `libm`，Keil/IAR 开启相应 C99 支持。保留 NaN/Inf 检查，**不要启用 `-ffast-math`**；MCU 与库的浮点 ABI 必须一致。模块依赖见 [工程结构](PROJECT_STRUCTURE.md)，交叉编译见 [构建与验证](VALIDATION.md)。

接线前先统一以下约定：

| 数据 | 本库要求 |
|---|---|
| 角度、角速度、角加速度 | `rad`、`rad/s`、`rad/s²`；角速度是受控角在同一坐标下的导数 |
| 力矩、惯量 | 关节侧 `N·m`、`kg·m²`；传动换算交给电机适配层 |
| `dt_s` | 实测控制间隔，单位秒；需落在控制器及参考生成器配置的范围内 |
| `feedback_age_s` | 本拍所需反馈中最旧样本的年龄，单位秒；读缓存不能刷新时间戳 |
| 有效性与使能 | 使用一致的反馈快照；应用层决定当前模式是否允许驱动 |

一个控制器实例由一个任务更新。进入控制模式或从故障恢复时，以本拍有效反馈重建参考；通用示例已经处理重新锚定。直接用核心时，这部分由调用者管理。

调用后先检查 `output.valid` 和 `output.flags`，再换算电机命令。`valid=false` 时核心力矩为零，但库不会自行发送停机帧；上层需要提交相应的停止命令。`valid=true` 仍可能发生限幅，应记录对应标志。

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

具体修改位置、DM 使能过程和 bxCAN HAL 桥接见 [STM32 移植步骤](STM32_PORT.md)；模式、量程和制造商依据见 [电机协议说明](MOTOR_PROTOCOL.md)。需要对照其他 RM 框架时阅读 [框架接口对照](RM_FRAMEWORK_INTEGRATION.md)。

## 5. 可选：获取物理参数

控制器可以直接使用已经标定的配置。需要辨识惯量、摩擦和重力时，有两条独立路径：

| 路径 | 运行位置 | 入口与结果 |
|---|---|---|
| 在线拟合 | STM32，纯 C99 | [`gimbal_identification`](ONLINE_IDENTIFICATION.md) 从新采样形成积分窗口，交给 RLS 输出物理参数候选 |
| 离线辨识 | 电脑，Python/NumPy | [`identify_gimbal.py`](IDENTIFICATION.md) 读取独立训练/验证日志，导出报告和候选 C 常量 |

两条云台辨识路径都要求固定底座、固定构形和同步标定的关节力矩；Yaw 拟合 `J/B/Fc`，Pitch 再拟合重力系数 `A/C`。在线 `ready` 仅表示本拍候选数据门槛满足，**不会自动更新 `gimbal_smc_t` 或调整控制增益**。行驶中、折叠中及三轴耦合不属于当前云台模型范围。

在线 C 模块已包含在常规 CMake 构建中。`GIMBAL_BUILD_IDENTIFICATION=ON` 仅增加电脑端离线检查，完整环境共有 15 个 CTest 入口；依赖安装与运行命令见 [离线辨识说明](IDENTIFICATION.md)，在线采样与候选使用流程见 [在线拟合说明](ONLINE_IDENTIFICATION.md)。
