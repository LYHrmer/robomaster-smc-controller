# 文档导航

[返回项目首页](../README.md)

本项目的主要入口是纯 C99 云台控制器。建议先在电脑上运行，再接入自己的工程；在线拟合和离线辨识可按需加入。想先了解文件分工与依赖，阅读 [工程结构](PROJECT_STRUCTURE.md)。

## 推荐阅读路线

| 阶段 | 文档 | 读完能解决什么 |
|---|---|---|
| 1. 运行示例 | [快速开始](QUICKSTART.md) | 环境要求、主机测试、接口选择、需要加入工程的文件 |
| 2. 接入硬件框架 | [STM32 移植](STM32_PORT.md) | 官方 C 板的反馈、模式、力矩输出和 CAN 接入顺序 |
| 3. 核对电机 | [电机协议与模式](MOTOR_PROTOCOL.md) | DM4310 / GM6020 的量程、方向、传动、电流/电压模式和组帧 |
| 4. 整定控制器 | [控制器设计](CONTROL_DESIGN.md) | 控制律、符号、C 字段、增益、限幅与默认配置 |
| 5. 单独处理 Pitch | [Pitch 接入与整定](PITCH_TUNING.md) | 重力补偿、三个角度坐标、参考过冲、行程与制动力矩 |
| 可选：用日志获取模型 | [离线辨识与整定流程](IDENTIFICATION.md) | 在电脑上辨识固定构形参数，独立验证后导出候选 C 常量 |
| 可选：在 STM32 上拟合 | [在线参数拟合](ONLINE_IDENTIFICATION.md) | 用 C99 RLS 随新数据更新物理参数候选，检查激励、冻结和恢复 |

辨识结果不会自动写入控制器。在线 `ready` 是本拍候选数据门槛；模型验证与应用参数切换由上层处理。两种辨识流程均不自动选择 `lambda/k/eta/phi`。

## 查看验证证据

| 文档 | 内容与范围 |
|---|---|
| [验证记录](VALIDATION.md) | 当前主机测试、Arm 静态库编译及未覆盖事项的总览 |
| [开源模型验证](OPEN_MODEL_VALIDATION.md) | 两份模型、576 个闭环组合、指标、门槛和复现方式 |
| [Pitch 专项验证](PITCH_VALIDATION.md) | 20 个验收场景和 4 个错误重力坐标诊断，包含上下行程与偏载 |
| [在线拟合验证范围](ONLINE_IDENTIFICATION.md#6-验证与适用范围) | RLS、流式积分与合成闭环程序的用途和限制；执行结果汇总在验证记录 |
| [原合成算例假设](../sim/assumptions.md) | 原始合成控制对照的参数、延迟、扰动和统计口径 |
| [开源参数审查](OPEN_SOURCE_PARAMETER_REVIEW.md) | 参数来源、可采用内容与不能作为辨识数据的部分 |
| [归档模型说明](../sim/open_models/README.md) | 固定版本原文件、许可、哈希与参数派生过程 |

默认完整环境注册 **13 个 CTest 入口**，包含在线 C 模块、RLS 长时数值回归和多场景程序。`GIMBAL_BUILD_IDENTIFICATION=ON` 另加两个电脑端离线检查入口，共 **15 个**，需要 Python/NumPy；它不是在线模块的开关。条件依赖见 [构建选项](PROJECT_STRUCTURE.md#构建选项)。

入口数量与场景数量分开计数：576 是开源模型组合数，Pitch 的 4 个诊断单独报告，离线辨识后的 8 个候选闭环案例见 [辨识说明](IDENTIFICATION.md)。当前执行结果以 [验证记录](VALIDATION.md) 为准；软件/模型验证不代表实机精度或三轴联动验证。

## 对照框架与扩展设计

| 文档 | 适合什么时候看 |
|---|---|
| [RM 框架接口对照](RM_FRAMEWORK_INTEGRATION.md) | 接入官方、HNU 或 rm-controls 风格工程，核对反馈、前馈和输出职责 |
| [多轴扩展方向](MULTI_AXIS.md) | 了解大小 Yaw、折叠双 Pitch 需要新增什么；该部分尚处规划阶段 |
| [参考与致谢](REFERENCES.md) | 查原滑模项目、官方/社区源码和许可归属 |

## 按 API 查源码

| 任务 | 入口 |
|---|---|
| 完整参考三元组 → 关节力矩 | [`gimbal_smc_update()`](../include/gimbal_smc.h) |
| 只有位置目标 → 参考三元组 | [`gimbal_reference_step()`](../include/gimbal_smc.h) |
| 电机物理量与 CAN 报文 | [`gimbal_motor.h`](../include/gimbal_motor.h) |
| 固定平面保持力矩 | [`gimbal_pitch_gravity_torque()`](../include/gimbal_pitch.h) |
| 通用轴的反馈有效性、参考锚定与输出接入 | [`gimbal_controller_example.h`](../examples/gimbal_controller_example.h) |
| Pitch 的重力、机械相对角与参考约束 | [`gimbal_pitch_example.h`](../examples/gimbal_pitch_example.h) |
| 时间戳反馈 → 在线云台物理参数候选 | [`gimbal_identification.h`](../include/gimbal_identification.h) |
| 自定义回归特征 → 通用在线 RLS | [`gimbal_rls.h`](../include/gimbal_rls.h) |

首次启用的参考锚定属于接入示例的逻辑；直接调用核心时，参考由调用者完整提供。控制器输出力矩，辨识器输出模型候选，二者可独立接入。具体入口选择见 [快速开始](QUICKSTART.md)。
