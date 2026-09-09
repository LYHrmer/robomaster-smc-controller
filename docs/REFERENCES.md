# 参考与致谢

[文档导航](README.md) · [项目首页](../README.md)

本库的 C 控制器与协议编解码按原理独立实现，不附带原滑模 C++ 源码、厂商 PDF 或完整官方例程。为复现开源参数验证，`sim/open_models/sources/` 额外归档了两份开源机器人模型的必要原始文件及其许可，保留原有 BSD-3-Clause/Apache-2.0 条款；这些文件不改授 MIT。以下为主要来源，框架接口补充核验日期为 2026-09-09。

| 来源 | 本项目借鉴或核对的内容 |
|---|---|
| [复旦大学星云 EGA 滑模开源](https://github.com/xinruilee04/smc_controller/tree/a5ad3746196fd1683af0da2ab1b1a87d419e884b) | 云台单轴模型、线性/终端滑模、原实现周期与状态审查 |
| [对应教学文章](https://bbs.robomaster.com/article/1939327?source=1) | 项目来源和作者归属；网页标示 CC BY-NC-SA 4.0，本仓库不转载文章正文 |
| [RoboMaster 官方 19.gimbal_task](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c) | 主接入框架、反馈坐标转换、模式与计算顺序；机械相对角和绝对角目标偏差的限位映射 |
| [官方 gimbal_task.h](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/gimbal_task.h) | 云台角度/角速度字段和单位 |
| [官方 CAN_receive.c](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/CAN_receive.c) | 原统一组帧/发送入口及旧协议边界 |
| [HNUYueLuRM 云台模块](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/gimbal/gimbal.c) | 反馈源与电机实例分离；该版本 pitch 速度绑定 Gyro[0]，重力补偿仅为待实现注释 |
| [HNUYueLuRM DJI 电机模块](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/DJImotor/dji_motor.c) | CAN 集中发送、槽位所有权，以及原有 PID／前馈字段不能直接视为力矩接口的边界 |
| [HNUYueLuRM INS 模块](https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/imu/ins_task.c) | 实测采样时间的工程做法 |
| [rm-controls 云台控制器](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/src/gimbal_base.cpp) | base 坐标内的目标限位、关节 child 坐标中的重力向量与力臂叉乘、effort 层加入前馈 |

固定版本对应的采纳点、不能直接照搬之处和当前实现范围见 [RoboMaster 框架接口对照](RM_FRAMEWORK_INTEGRATION.md)。该对照仅借鉴接口职责与坐标处理，没有导入战队控制源码。

DM4310 与 GM6020 的制造商手册、厂商源码入口、协议量程和固件要求集中列于 [MOTOR_PROTOCOL.md](MOTOR_PROTOCOL.md)，避免多处维护冲突。

感谢原滑模项目作者及复旦大学星云 EGA 战队公开分享设计，感谢官方及社区电控开源工程提供可核对的接口依据。本库采用新的函数、数据结构、保护逻辑及测试，实现独立的 C 控制器接口；外部项目的许可不被本库 MIT 许可覆盖。
