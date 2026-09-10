# 电机适配与 CAN 接入

[文档导航](README.md) · [项目首页](../README.md)

本模块只做关节力矩换算、CAN 数据编码和反馈解析，使用 C99，不依赖 STM32 HAL，也不会自行使能、清错、保存零点或发送 CAN。闭环控制器输出单位为关节侧 N·m。

本文帮助你选对电机工作模式、完成力矩到报文的转换，并处理反馈与失败状态。先读[共用坐标和约束](#共用坐标和约束)，再选择对应电机；HAL 发送示例见 [STM32 接入](STM32_PORT.md#5-bxcan-hal-发送桥)，资料出处集中在[文末](#已核验的资料)。

| 电机与模式 | 力矩命令路径 | 接入前需要确认 |
|---|---|---|
| [DM4310 MIT](#dm4310mit-纯力矩) | 关节 N·m → MIT 力矩帧 | 电机模式、ID、PMAX/VMAX/TMAX、使能状态 |
| [GM6020 原生电流模式](#gm6020明确区分两种模式) | 关节 N·m → 单电机电流命令 → 统一组帧 | 固件版本、电流环开关、转矩常数与电流限值 |
| GM6020 原始电压模式 | 仅提供原始电压命令组包 | 需要另行实现并验证电流内环，本库未提供 |

## API 速查

所有声明位于 [gimbal_motor.h](../include/gimbal_motor.h)。表中函数只计算或组包，不会发送 CAN。

| 操作 | 函数 | 输出 |
|---|---|---|
| DM 力矩编码 | `gimbal_dm4310_pack_torque()` | 单电机 CAN 帧、量化后的名义关节力矩 |
| DM 使能/失能编码 | `gimbal_dm4310_pack_enable()` | 使能或失能 CAN 帧 |
| DM 反馈解析 | `gimbal_dm4310_decode()` | 电机坐标下的位置、速度、力矩及状态 |
| GM 力矩换算 | `gimbal_gm6020_torque_to_current()` | 单电机 `int16_t` 电流命令、量化后的名义关节力矩 |
| GM 电流组帧 | `gimbal_gm6020_pack_current_group()` | 一组电机共用的 CAN 帧 |
| GM 原始电压组帧 | `gimbal_gm6020_pack_voltage_group()` | 一组电机共用的原始电压 CAN 帧 |
| GM 反馈解析 | `gimbal_gm6020_decode()` | 电机坐标下的位置、速度、电流原始值及温度 |

## 共用坐标和约束

配置 `gear_ratio = 电机模组输出轴转速 / 关节转速`，仅计模组之后的外部传动，直接连接为 1。DM4310 的 MIT 力矩/速度已按模组输出轴定义，不能把内部减速比再乘一次。`motor_torque_limit_nm` 同样位于模组输出轴侧。`efficiency` 是外部传动效率，范围 `(0,1]`；`direction` 只能为 `+1` 或 `-1`。换算为：

```text
motor_torque = direction * joint_torque / (gear_ratio * efficiency)
```

`motor_torque_limit_nm`、GM6020 的 `current_limit_a` 必须是针对实际云台选定的应用限值。编码量程、堵转能力、瞬时上限不等于可持续输出。温升降额、机械限位、急停、通信超时与重启授权由上层状态机管理。

解析函数返回电机坐标，不自动做关节零偏、减速比换算、多圈展开或 IMU 姿态变换。上层使用编码器前需统一方向与角度单位；云台惯性稳定通常使用 IMU 姿态和角速度，电机编码器用于机械相对位置及限位。

统一按返回状态决定是否使用本次结果：

| 返回状态 | 本次结果 | 调用方处理 |
|---|---|---|
| `GIMBAL_MOTOR_OK`（0） | 有效 | 按发送或反馈处理路径继续 |
| `GIMBAL_MOTOR_CLAMPED`（1） | 有效，但应用限值或量化约束触发限幅 | 可以使用，记录限幅与 `applied_joint_nm` |
| 负值 | 不可用于控制或发送 | 执行上层故障路径，不复用上一拍结果 |

编码失败时输出帧清零且 `dlc=0`，避免上一周期命令残留；这**不等于已经让电机停机**。DM 反馈的未使能/故障结果仅保留状态和温度诊断，其位置、速度、力矩清零。

## DM4310：MIT 纯力矩

1. 在达妙调试工具中核对 MIT 模式、CAN ID、MASTER_ID、PMAX、VMAX、TMAX，以及通信看门狗。
2. `command_id` 直接使用配置 CAN ID，MIT 不增加 `0x100/0x200` 模式偏移；`feedback_id` 使用 MASTER_ID，两者不可混淆。MASTER_ID 可以是 0。
3. 本模块按厂商示例解析反馈首字节低 4 位的 ID 与高 4 位状态；`motor_id` 要与 command_id 的低 4 位一致。为避免多个电机在共享 MASTER_ID 下发生低位别名，建议每台电机配置独立 MASTER_ID，并在工程中分配无冲突的控制 ID。
4. `gimbal_dm4310_pack_torque()` 将 `kp=kd=0`，位置/速度设置为最近的零编码，只通过 `t_ff` 给定力矩。kp、kd 的零编码是精确零，因此位置、速度的量化偏差不产生 P/D 力矩。

MIT 八字节布局为 `p:16, v:12, kp:12, kd:12, t_ff:12`，高位在前。反馈布局为首字节 ID/状态、位置 16 位、速度 12 位、力矩 12 位、MOS 温度和线圈温度各 8 位。

厂商当前参考表中 DM4310 是 `PMAX=12.5 rad, VMAX=30 rad/s, TMAX=10 N·m`，另有 48 V 条目 `VMAX=50 rad/s`。这些值能够由调试工具修改，库中不提供型号默认配置；必须以连接的电机实际设置为准。尤其不能直接把历史 MIT 示例中的 ±18 N·m 套给 DM4310。

力矩编码只能表达 4096 个离散值，其分辨率为 `2*TMAX/4095`。对称编码没有精确的零力矩码；本实现就近取整，并给出 `applied_joint_nm` 供记录实际量化给定。零命令的量化偏差上限约为半个编码步长，机械侧再乘传动增益。因此不能用零力矩帧代替失能。量化之后仍限制在配置力矩范围内；若限值小到连最接近零的编码都容不下，则返回配置错误。

`gimbal_dm4310_pack_enable(cfg,1,...)` 生成 `FF FF FF FF FF FF FF FC`；传入 0 生成末字节 FD 的失能帧。应用显式发送并核验使能反馈。本实现将状态 0 视为未使能、1 视为使能，其他状态一律拒绝参与闭环；上机前需核验该固件的状态解释。故障 8–E 分别涉及过压、欠压、过流、MOS 过温、线圈过温、通信丢失、过载。不得在控制周期中反复自动清错、重新使能或保存零点。

需要从 DM4310 反馈获取模型参数时，见 [DM4310 辨识指南](DM4310_IDENTIFICATION.md)，按实际电机配置记录原始反馈，完成力矩标定、时间与坐标核验后再整理辨识数据。

## GM6020：明确区分两种模式

| 用途 | ID 1–4 组 | ID 5–7 组 | 有符号命令量程 |
| --- | --- | --- | --- |
| 转矩电流模式 | `0x1FE` | `0x2FE` | `[-16384,16384]` 对应 `[-3,3] A` |
| 原始电压模式 | `0x1FF` | `0x2FF` | `[-25000,25000]`，本库不假定其单位为 V |

均为 CAN 标准数据帧、DLC 8，每个电机占大端有符号 16 位；第二组末两字节保留为零。波特率 1 Mbit/s。不要套用 M3508/C620 的 ±20 A 电流换算，也不要沿用旧示例的 ±30000 电压指令。

**推荐路径：启用 GM6020 原生电流环。** 官方手册要求电机固件 `>=1.0.11.2`，RoboMaster Assistant `>=2.7`，且在软件中打开电流环开关。确认完成后才设置 `current_mode_confirmed=1`。库不会升级固件或切换电机模式。

`gimbal_gm6020_torque_to_current()` 先以显式 `torque_constant_nm_per_a` 将电机力矩换算为 A，再按 `16384/3` 转成命令并实施限幅。手册提供 0.741 N·m/A 的转矩常数，可作为初始参数；实际云台应校核有效转矩、摩擦、温度与传动影响。函数的 `applied_joint_nm` 是量化指令对应的名义力矩，不能当成实测力矩。

每组必须有一个统一发送者，每周期收集四个槽位后仅组包发送一次。`gimbal_gm6020_pack_current_group(1,commands,frame)` 的槽位对应 ID 1–4；`first_motor_id=5` 对应 5–7，第 4 槽必须为零。不要让每个轴分别发一帧并把其他槽位置零，否则会覆盖同组其他电机命令。

旧固件或未启用原生电流环时，只提供 `gimbal_gm6020_pack_voltage_group()` 原始编码函数。若必须保持电压模式，需另行设计、标定并验证 MCU 电流 PI 内环，把目标电流与实测电流误差转换为电压命令；本库尚未提供该内环，不能将 N·m 线性缩放为电压后声称完成力矩闭环。

反馈 ID 为 `0x204+motor_id`，DLC 8，约 1 kHz。编码器 0–8191，转速单位 rpm，随后是转矩电流原始值、温度和保留字节。官方所查手册没有给出反馈电流的安培换算比例，所以默认 `feedback_current_a_per_count=0`，仅提供 `current_raw`、`current_a_valid=0`。拿到对应固件的反馈比例并核验后再配置；不能因为命令按 3 A 满量程，就自动给反馈也套同一比例。

## STM32 边界

接收层只把标准 DATA 帧交给本库，先拒绝 RTR、扩展帧及 CAN FD 变体；结构体本身不携带这些标志。反馈必须带独立的本地时间戳用于失联判断。HAL 中断与控制任务共享反馈时，应使用快照或短临界区，避免混读两个 CAN 周期。

发送层只发送返回值非负且 `dlc=8` 的帧，检查发送邮箱/FIFO、返回值和总线错误。保护停机与控制帧优先级应在总线上机联调中验证。一个命令组内存在其他电机时，应用必须维护全部槽位的所有权，不能由这个库猜测其他电机应继续还是停机。

主机单元测试覆盖独立协议字节向量、端点、方向、减速比与效率、应用限幅、NaN/Inf、非法 ID/DLC、DM 未使能/故障、GM 四槽组帧和反馈单位。测试不替代电机台架或 STM32 的时序与 CAN 负载验证。

## 已核验的资料

- DJI 官方产品入口：[GM6020](https://www.robomaster.com/zh-CN/products/components/general/GM6020)。下载列表由官方 [simple_cms.json](https://rm-static.djicdn.com/live_json/simple_cms.json) 提供。
- DJI 官方 [GM6020 使用说明书，文件日期 20231013](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf)，印刷页 6–7 是 CAN 协议，页 11 是电机参数。
- 达妙厂商原始 [DM-J4310-2EC V1.1 使用说明书](https://doc.switch-science.com/media/files/3c435a26-84da-4dc2-8322-c144e29b6518.pdf)，由经销商镜像托管；印刷页 4、6–7 说明 MIT 与反馈格式。
- 达妙官方 [STM32 dm4310_drv.h](https://github.com/dmBots/motor-control-routine/blob/master/stm32%E4%BE%8B%E7%A8%8B/dm_ctrl%28f4%29-4310_v1.0%20%E8%A3%B8%E6%9C%BA/User/motor/dm4310_drv.h)、[dm4310_drv.c](https://github.com/dmBots/motor-control-routine/blob/master/stm32%E4%BE%8B%E7%A8%8B/dm_ctrl%28f4%29-4310_v1.0%20%E8%A3%B8%E6%9C%BA/User/motor/dm4310_drv.c)。厂商 [damiao.cpp](https://github.com/dmBots/motor-control-routine/blob/master/orin%E8%BD%BD%E6%9D%BFcan%E6%8E%A7%E5%88%B6%E8%BE%BE%E5%A6%99%E7%94%B5%E6%9C%BA%E4%BE%8B%E7%A8%8B/dm_hw/src/hardware_interface/damiao.cpp) 也提供型号范围表。

资料核验日期：2026-09-08。协议事实独立实现，未复制厂商或参考仓库的源代码。
