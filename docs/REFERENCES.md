# 参考与致谢

本库的 C 控制器与协议编解码按原理独立实现，不附带原滑模 C++ 源码、厂商 PDF 或完整官方例程。为复现开源参数验证，`sim/open_models/sources/` 额外归档了两份开源机器人模型的必要原始文件及其许可，保留原有 BSD-3-Clause/Apache-2.0 条款；这些文件不改授 MIT。以下为主要来源，核验日期为 2026-09-08。

| 来源 | 本项目借鉴或核对的内容 |
|---|---|
| [李欣睿 / 复旦大学星云 EGA 滑模开源](https://github.com/xinruilee04/smc_controller/tree/a5ad3746196fd1683af0da2ab1b1a87d419e884b) | 云台单轴模型、线性/终端滑模、原实现周期与状态审查 |
| [对应教学文章](https://bbs.robomaster.com/article/1939327?source=1) | 项目来源和作者归属；网页标示 CC BY-NC-SA 4.0，本仓库不转载文章正文 |
| [RoboMaster 官方 19.gimbal_task](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/19.gimbal_task/application/gimbal_task.c) | 主接入框架、反馈坐标转换、模式与计算顺序 |
| [官方 gimbal_task.h](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/gimbal_task.h) | 云台角度/角速度字段和单位 |
| [官方 CAN_receive.c](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/CAN_receive.c) | 原统一组帧/发送入口及旧协议边界 |
| [HNUYueLuRM 云台模块](https://github.com/HNUYueLuRM/basic_framework/blob/master/application/gimbal/gimbal.c) | 少量架构对照：反馈源与电机实例分离，不同框架的 IMU 轴定义不同 |
| [HNUYueLuRM DJI 电机模块](https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/motor/DJImotor/dji_motor.c) | CAN 总线与组的集中发送、槽位所有权 |
| [HNUYueLuRM INS 模块](https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/imu/ins_task.c) | 实测采样时间的工程做法 |

DM4310 与 GM6020 的制造商手册、厂商源码入口、协议量程和固件要求集中列于 [MOTOR_PROTOCOL.md](MOTOR_PROTOCOL.md)，避免多处维护冲突。

感谢原滑模项目作者及复旦大学星云 EGA 战队公开分享设计，感谢官方及社区电控开源工程提供可核对的接口依据。本库采用新的函数、数据结构、保护逻辑及测试，实现独立的 C 控制器接口；外部项目的许可不被本库 MIT 许可覆盖。
