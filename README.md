# RoboMaster SMC Controller

**面向 STM32 云台的纯 C99 滑模控制器库，提供 DM4310 与 GM6020 力矩适配。**

[![C99 controller tests](https://github.com/LYHrmer/robomaster-smc-controller/actions/workflows/c-tests.yml/badge.svg)](https://github.com/LYHrmer/robomaster-smc-controller/actions/workflows/c-tests.yml)
[![C99](https://img.shields.io/badge/language-C99-blue)](include/gimbal_smc.h)
[![Code license MIT](https://img.shields.io/badge/code_license-MIT-green)](LICENSE)

[快速开始](docs/QUICKSTART.md) · [STM32 接入](docs/STM32_PORT.md) · [Pitch 整定](docs/PITCH_TUNING.md) · [全部文档](docs/README.md)

把同一坐标系下的**目标、角度和角速度**交给控制器，得到关节力矩 **N·m**，再由电机适配器转换为 CAN 命令。算法无 HAL、RTOS、堆内存或 C++ 依赖；原工程继续负责模式管理、INS 和 CAN 发送。

Yaw、Pitch 分别使用独立实例与参数。优先提供 [RoboMaster 官方 C 型开发板例程](https://github.com/RoboMaster/Development-Board-C-Examples)的接入方法，适合已有电控框架、希望替换或比较云台闭环的开发者。

## 从这里开始

| 你想做什么 | 阅读入口 |
|---|---|
| 先在电脑上运行，再了解接口 | [快速开始](docs/QUICKSTART.md) |
| 接到官方 C 板 / 自己的 STM32 工程 | [移植步骤](docs/STM32_PORT.md) → [电机协议与模式](docs/MOTOR_PROTOCOL.md) |
| 调整 Pitch 重力补偿、上下行程和参考限制 | [Pitch 接入与整定](docs/PITCH_TUNING.md) |
| 理解公式、参数含义与改进 | [控制器设计](docs/CONTROL_DESIGN.md) |
| 查看仿真依据、结果及当前局限 | [验证记录](docs/VALIDATION.md) |
| 了解大小 Yaw / 折叠双 Pitch 如何扩展 | [多轴扩展方向](docs/MULTI_AXIS.md)（规划阶段） |

## 先在电脑上运行

需要 C99 编译器、CMake 3.16+；安装 Python 3.9+ 可运行完整的 9 个 CTest 测试入口。

```sh
git clone https://github.com/LYHrmer/robomaster-smc-controller.git
cd robomaster-smc-controller
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

完整环境下应看到 `100% tests passed`，共 9 项。测试包含控制器、电机协议、接入示例和多场景仿真；生成的数据保存在 `build/`。下一步见 [快速开始](docs/QUICKSTART.md)，其中说明如何选择接口、复制源文件和接入反馈。

## 控制链路与主要功能

```mermaid
flowchart LR
    A["应用层<br/>参考、反馈、模式"] --> B["每轴一份 C SMC<br/>关节力矩 N·m"]
    B --> C["电机适配<br/>DM4310 / GM6020"]
    C --> D["原工程<br/>CAN 组帧与发送"]
```

| 模块 | 提供的能力 |
|---|---|
| 单轴滑模核心 | 默认线性滑模与可调边界层；可选正则化终端项 |
| 参考生成 | 位置目标生成角度、速度、加速度；已有解析三元参考可直接输入核心 |
| 输出与有效性 | 力矩限幅、可选变化率限制、速度低通；检查周期、反馈超时和非有限值 |
| Pitch 辅助 | 有符号重力模型；独立的控制角、重力角与机械相对角接入示例 |
| 电机适配 | 按方向、传动与电机参数换算力矩，提供协议编解码及统一组帧接口 |

参考生成器限制速度和加速度，**不保证参考无过冲**；Pitch 示例的参考包络也不等于机械停车保证。调参方法与适用条件见 [控制器设计](docs/CONTROL_DESIGN.md) 和 [Pitch 整定](docs/PITCH_TUNING.md)。

## 电机和轴怎么选

| 对象 | 接入方式 | 关键条件 |
|---|---|---|
| DM4310 | MIT 模式，`kp=kd=0`，发送力矩前馈 | 核对模式、ID、协议量程、传动和看门狗 |
| GM6020 | 已启用的原生电流模式 | 核对电机固件、力矩常数、电流限制和 CAN 分组 |
| 连续多圈 Yaw | 连续角反馈与目标，`wrap_angle=false` | 保留累计圈数；最短路径角差只用于明确需要该语义的场景 |
| 有限行程 Pitch | 独立参数、重力前馈及机械角约束 | 明确重力相位、零位和上下行程，分别检查加速与制动余量 |
| 三轴 / 折叠双 Pitch | 单轴核心可复用，协调层待实现 | 当前没有三轴目标分配、折叠状态机或三轴联动验证 |

**GM6020 原生电流组为 `0x1FE/0x2FE`，旧电压组为 `0x1FF/0x2FF`。** 不能把电流命令直接交给未修改的官方旧 `CAN_cmd_gimbal()`；旧电压模式目前仅提供原始报文编码。详见 [电机说明](docs/MOTOR_PROTOCOL.md)。

接入时统一使用 **rad、rad/s、rad/s²、关节 N·m**；角速度必须是所用控制角在同一坐标系下的导数。保留有限数值检查，编译时不要启用 `-ffast-math`。每个电机只由一个有效控制路径输出，同组 CAN 帧由一个发送者合并。

## 已验证到哪一步

| 验证层级 | 当前结果 | 查看依据 |
|---|---|---|
| 主机回归 | Debug、Release、ASan/UBSan 各通过 9 个 CTest 入口 | [验证记录](docs/VALIDATION.md) |
| 开源模型闭环 | 两份模型派生参数，576 个组合通过 | [模型来源与结果](docs/OPEN_MODEL_VALIDATION.md) |
| Pitch 专项 | 20 个验收场景通过；另保留 4 个错误坐标诊断 | [大行程、偏载与重力坐标](docs/PITCH_VALIDATION.md) |
| STM32 工具链 | Cortex-M4F hard-float 四个静态库交叉编译通过 | [构建与边界](docs/VALIDATION.md) |

这些结果属于软件与模型验证。**本公共库尚无实机精度、整车固件或 MCU 最坏执行时间验证。** 仿真参数需要按自己的机构标定；4 个诊断场景不计入 Pitch 验收，也不能作为错误重力坐标可用的证明。

下面是 Pitch 专项的分方向误差，完整条件、门槛和原始数据见 [报告](docs/PITCH_VALIDATION.md)。

![Pitch 合成模型的上行、下行与保持误差；不是实机精度](sim/results/pitch/pitch_directional_summary.png)

## 源码入口

| 文件 / 目录 | 内容 |
|---|---|
| [gimbal_smc.h](include/gimbal_smc.h) / [gimbal_smc.c](src/gimbal_smc.c) | 单轴控制器与参考生成器 |
| [gimbal_motor.h](include/gimbal_motor.h) / [gimbal_motor.c](src/gimbal_motor.c) | DM4310 / GM6020 物理量换算与协议 |
| [gimbal_pitch.h](include/gimbal_pitch.h) / [gimbal_pitch.c](src/gimbal_pitch.c) | 固定平面 Pitch 重力辅助函数 |
| [examples/](examples/) | 通用轴和 Pitch 接入示例 |
| [tests/](tests/) / [sim/](sim/) | 回归测试、实际调用 C 核心的仿真与数据 |
| [docs/](docs/README.md) | 入门、原理、移植、整定、验证及参考资料 |

## 来源、许可与反馈

参考复旦大学星云 EGA 的 [滑模开源项目](https://github.com/xinruilee04/smc_controller)与[教学文章](https://bbs.robomaster.com/article/1939327?source=1)，按原理独立实现 C 接口。设计与框架对照见 [控制器设计](docs/CONTROL_DESIGN.md)、[RM 框架接口对照](docs/RM_FRAMEWORK_INTEGRATION.md)和[参考与致谢](docs/REFERENCES.md)。

本仓库新编写的代码与说明采用 [MIT](LICENSE)；归档的第三方模型保留其 BSD-3-Clause / Apache-2.0 许可及原始声明。

反馈问题时，请附电机型号/固件/模式、MCU、控制周期、轴与单位约定、参数，以及带时间戳的目标/反馈角度、速度、输出力矩和故障标志。可在 [Issues](https://github.com/LYHrmer/robomaster-smc-controller/issues) 提交。
