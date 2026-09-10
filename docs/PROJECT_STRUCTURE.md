# 工程结构与模块选择

[项目首页](../README.md) · [文档导航](README.md) · [快速开始](QUICKSTART.md)

本页用于确定“该加哪些文件、模块依赖什么、接口在哪里”。首次构建命令见[快速开始](QUICKSTART.md)，修改已有固件的接入点见[STM32移植](STM32_PORT.md)。

## 目录树

```text
robomaster-smc-controller/
├── include/                                公共 C 头文件，与 src/ 中模块同名
├── src/
│                                           ├── gimbal_smc.c                 单轴滑模控制与参考生成
│                                           ├── gimbal_motor.c               关节力矩转换、电机报文与反馈解析
│                                           ├── gimbal_pitch.c               固定平面重力模型
│                                           ├── gimbal_rls.c                 通用递推最小二乘
│                                           └── gimbal_identification.c      云台数据资格检查与积分回归窗口
├── examples/
│                                           ├── gimbal_controller_example.c  通用位置目标接入，配套同名 .h
│                                           └── gimbal_pitch_example.c       Pitch 接入与参考约束，配套同名 .h
├── tests/                                  C 回归、耐久测试及离线辨识检查
├── sim/
│                                           ├── *.c                         调用实际 C 库的合成验证程序
│                                           ├── open_models/                固定版本模型、来源与派生参数
│                                           └── results/                    归档指标、图件和验证条件
├── tools/
│                                           ├── identify_gimbal.py           电脑端离线辨识与候选参数导出
│                                           ├── validate_identification.py   离线辨识到 C 闭环的完整演示
│                                           └── derive_open_model_profiles.py  开源模型参数复算
├── cmake/arm-none-eabi.cmake               Cortex-M4F 交叉工具链示例
├── docs/                                   接入、原理、整定和验证文档
├── .github/workflows/c-tests.yml           主机回归与 Arm 静态库编译
└── CMakeLists.txt                          库、示例、测试与仿真构建入口
```

`include/`、`src/` 和需要的 `examples/` 文件可以直接加入 STM32 工程；`tools/` 是电脑端工具。源码路径保持稳定，不要求把整车工程迁入本仓库。

## 按接入目标选择文件

| 接入目标 | 需要加入的源文件 | 头文件路径与说明 |
|---|---|---|
| **只接控制器** | [src/gimbal_smc.c](../src/gimbal_smc.c) | 添加 `include/`；输入参考三元组（角度、角速度、角加速度）和反馈，得到关节力矩。采用本库 DM4310/GM6020 适配时，再加 [src/gimbal_motor.c](../src/gimbal_motor.c) |
| **接 Pitch 示例** | `src/gimbal_smc.c`、`src/gimbal_motor.c`、[src/gimbal_pitch.c](../src/gimbal_pitch.c)<br>[examples/gimbal_controller_example.c](../examples/gimbal_controller_example.c)、[examples/gimbal_pitch_example.c](../examples/gimbal_pitch_example.c) | 添加 `include/`、`examples/`；这是位置目标路径，组合参考生成、重力前馈与当前参考区间约束。接法见 [Pitch 整定](PITCH_TUNING.md) |
| **加在线辨识** | 在所选控制路径之外，加入 [src/gimbal_rls.c](../src/gimbal_rls.c)、[src/gimbal_identification.c](../src/gimbal_identification.c) | 添加 `include/`；辨识器独立读取合格反馈、输出候选模型，不会直接修改控制器。要求和调用流程见 [在线辨识](ONLINE_IDENTIFICATION.md) |

通用位置目标接入可在控制器、电机模块上增加 `examples/gimbal_controller_example.c` 和 `examples/` 头文件路径，不必使用 Pitch 专用示例。若已有参考角速度/加速度，或已有合适的电机适配层，可直接使用核心接口。

各轴分别保存自己的控制器或辨识器状态。多轴协调与折叠运动模型属于[后续机构层设计](MULTI_AXIS.md)，不会因增加实例数量自动获得。

## CMake 目标依赖

六个目标均生成静态库，依赖由 [CMakeLists.txt](../CMakeLists.txt) 声明。表中的 `m` 是数学库；直接加入源文件的 GCC 工程也需要链接数学库。

| CMake目标 | 接口与实现 | 直接链接依赖 |
|---|---|---|
| `gimbal_smc` | [头文件](../include/gimbal_smc.h) / [C实现](../src/gimbal_smc.c) | `m` |
| `gimbal_motor` | [头文件](../include/gimbal_motor.h) / [C实现](../src/gimbal_motor.c) | `m` |
| `gimbal_pitch` | [头文件](../include/gimbal_pitch.h) / [C实现](../src/gimbal_pitch.c) | `m` |
| `gimbal_rls` | [头文件](../include/gimbal_rls.h) / [C实现](../src/gimbal_rls.c) | `m` |
| `gimbal_identification` | [头文件](../include/gimbal_identification.h) / [C实现](../src/gimbal_identification.c) | `gimbal_rls`、`m` |
| `gimbal_example` | [通用轴](../examples/gimbal_controller_example.c) / [Pitch](../examples/gimbal_pitch_example.c) | `gimbal_smc`、`gimbal_motor`、`gimbal_pitch` |

`gimbal_example` 目标同时包含两个示例源文件，因此会链接 Pitch 模块。在线辨识目标不依赖 `gimbal_smc`；采纳模型参数的流程由应用层连接。

## 构建选项

| 选项 | 默认 | 控制的内容 |
|---|---|---|
| `GIMBAL_BUILD_TESTS` | ON | 主机回归入口；不控制库本体是否构建 |
| `GIMBAL_BUILD_EXAMPLES` | ON | `gimbal_example` 库及启用测试时的示例回归 |
| `GIMBAL_BUILD_SIM` | ON | 合成验证程序，以及启用测试时的相应 CTest 入口 |
| `GIMBAL_SANITIZE` | OFF | GNU/Clang 主机构建的 AddressSanitizer/UBSan |
| `GIMBAL_BUILD_IDENTIFICATION` | OFF | 额外的电脑端离线辨识检查；要求本机构建、测试开启及 Python 3.9+/NumPy |

**在线 C 模块始终定义为库目标，不受 `GIMBAL_BUILD_IDENTIFICATION` 控制。** 此选项只增加电脑端测试；固件是否进行在线拟合，取决于应用是否创建实例并调用接口。

Arm 交叉编译关闭主机测试和仿真，示例库按需保留，并保持离线辨识选项关闭。工具链的浮点 ABI 必须与完整固件一致。具体命令、测试数量和已执行结果统一见[验证记录](VALIDATION.md)。

## 按功能查接口

| 要做的事 | 接口入口 | 进一步说明 |
|---|---|---|
| 计算一拍滑模力矩 | `gimbal_smc_init()`、`gimbal_smc_update()` | [控制律与参数](CONTROL_DESIGN.md) |
| 使用通用位置目标接入示例 | `gimbal_example_axis_init()`、`gimbal_example_axis_step()` | [通用轴接口](../examples/gimbal_controller_example.h) |
| 将位置目标变成参考三元组 | `gimbal_reference_init()`、`gimbal_reference_step()` | [gimbal_smc.h](../include/gimbal_smc.h)；速度/加速度受限的参考跟踪器 |
| 换算力矩、组包或解析反馈 | `gimbal_dm4310_pack_torque()`、`gimbal_gm6020_torque_to_current()` 等 | [电机接口](../include/gimbal_motor.h)与[协议说明](MOTOR_PROTOCOL.md)；发送由原工程负责 |
| 只计算 Pitch 保持力矩 | `gimbal_pitch_gravity_torque()` | [重力模型接口](../include/gimbal_pitch.h) |
| 接入 Pitch 位置目标示例 | `gimbal_pitch_example_init()`、`gimbal_pitch_example_step()` | [示例接口](../examples/gimbal_pitch_example.h)；真实行程保护仍由应用监督 |
| 从逐次云台反馈得到模型候选 | `gimbal_identification_init()`、`gimbal_identification_update()` | [在线辨识](ONLINE_IDENTIFICATION.md) |
| 使用自己的线性参数化模型 | `gimbal_rls_init()`、`gimbal_rls_update()` | [通用 RLS 接口](../include/gimbal_rls.h)；应用提供回归特征 |

各头文件给出了单位、生命周期和返回状态约定；调用方应按这些约定处理无效反馈、模式变化和参数采纳。
