# 开源云台参数筛选记录

核验日期：2026-09-08。本文件记录参数的来源、含义与采用边界，不是仿真结果报告。以下官方例程、Combat 等效模型和 DM4310 单体台架数据均为候选证据，不能据此声称本项目已经模拟或复现了它们。

本轮正式仿真的参数以 [`sim/open_models`](../sim/open_models) 的归档模型、来源版本和派生记录为准。该目录负责保存正式模型的来源与坐标换算依据；本文件中的候选数值不覆盖正式模型。

## 1. DJI 官方例程：名义 1 ms 周期的证据

仓库 `RoboMaster/Development-Board-C-Examples`，已通过仓库 API 锁定 `master` 提交：

```text
59d12b1adcd321dbf1f9e9166aef5eb95ab657bf
commit date: 2022-09-03
```

| 数值或行为 | 精确来源 | 证据类型与适用边界 |
| --- | --- | --- |
| `GIMBAL_CONTROL_TIME=1` | [gimbal_task.h](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.h#L114) | 控制任务的延时 tick 数，不能单独解读为秒或毫秒。 |
| `configTICK_RATE_HZ=1000` | [FreeRTOSConfig.h](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/Inc/FreeRTOSConfig.h#L61) | RTOS 配置，1 tick 对应名义 1 ms。 |
| 循环末尾 `vTaskDelay(GIMBAL_CONTROL_TIME)` | [gimbal_task.c](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L368) | 以上两项结合可支持“官方例程采用名义 1 ms 调度”的描述。它不是执行时间、抖动或严格 1 kHz 的实测结果。 |
| 角环输出限幅 `10.0`，随后作为 `motor_gyro_set` | [参数定义](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.h#L56)、[控制路径](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L1003) | 含义是速度参考限幅，单位由控制路径确定为 rad/s；不是电机可在任意负载下达到的物理最大速度。 |
| 机械相对角上下限由标定传入 | [set_cali_gimbal_hook](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L398) | 例程没有提供可直接推广到另一台云台的固定机械工作角范围。 |

这套例程没有给出云台质量、质心和可核验的物理转动惯量。PID 增益、PID 输出限幅和标定驱动原始值都不能替代这些物理参数。

## 2. Combat 战队：有单位的工程等效模型

来源为作者公开的 [YAW_Auto_Controller 仓库](https://github.com/LamdaDay/YAW_Auto_Controller)，及 [RoboMaster 原始开源文章](https://bbs.robomaster.com/article/1935544)。以下读取自 `main`，核验日期如上；本次未成功锁定该仓库的不可变提交，因此不能把分支链接视为永久固定版本。

[`yaw_auto_lqr_tune.m`](https://github.com/LamdaDay/YAW_Auto_Controller/blob/main/yaw_auto_lqr_tune.m) 声明的模型是：

```text
J * theta_ddot + Bv * theta_dot = tau
```

| 参数 | 公开数值 | 单位 | 能支持的结论 |
| --- | --- | --- | --- |
| `J` | 0.039 | kg·m² | 作者控制模型的等效惯性参数。 |
| `Bv` | 0.30 | N·m·s/rad | 同一模型的等效阻尼参数。 |
| `Ts` | 0.001 | s | 该离线调参脚本使用的采样周期。 |
| `torqueLimit` | 7.0 | N·m | 脚本/固件的命令力矩限幅。 |
| `torqueSlewRate` | 7000.0 | N·m/s | 脚本的命令力矩变化率限制。 |
| 离线阶跃及时长 | 5.0；1.0 | °；s | 脚本自身的阶跃检查工况，尚未在本项目中复现。 |

作者 [README](https://github.com/LamdaDay/YAW_Auto_Controller/blob/main/README.md) 明确指出，其模型依据命令力矩辨识，所得参数不是实际物理参数。因此 `J=0.039`、`Bv=0.30` 不能被标成经过 CAD 或独立力矩传感器确认的真实云台参数；尤其不能把执行链已包含在辨识结果里的影响再次叠加，然后称为忠实复现。若未来采用，应单列为“公开工程等效模型”的测试，而不是替代本轮采用的 URDF/SDF 模型参数。

`7 N·m` 是源码选择的命令限幅，不能据此确定另一台 DM4310 的额定连续力矩、持续堵转能力或安全上机限额。电机版本、温升、散热、供电和机械传动仍需分别核验。

原始文章称这台云台采用一个 DM4310 驱动 yaw、两个 DM4310 驱动 pitch，并给出标注为“3 Hz 20 deg”“5 Hz 20 deg”的跟随曲线。文章文字没有明确 20° 是峰值还是峰峰值，不能擅自将其写成严格的 `A=20°` 正弦基准。若未来采用这些频率，应公开写出幅值约定、参考角速度/角加速度、力矩可行性及原图幅值含义尚未核实的事实。

## 3. DM4310 单体数据：不能当成云台负载模型

公开台架原始记录：[`takarakasai/misa-actuator/doc/bench-measurements-2026-07-30.md`](https://github.com/takarakasai/misa-actuator/blob/main/doc/bench-measurements-2026-07-30.md)。核验的是作者记录的一台 DM-J4310-2EC，固件字符串为 `5019`、子版本 `5`；它不是 RoboMaster 云台辨识报告。本次读取 `main`，未锁定不可变提交。

| 原始记录项 | 数值 | 必须保留的解释 |
| --- | --- | --- |
| `Gr` | 10 | 该电机模组的内部减速比。 |
| `GREF` | 1.0 | 该样机寄存器给出的效率因子；不是对任意实际负载测得的恒定传动效率。 |
| `Inertia` | 1.7915e-5 | 寄存器读数，表中称为转子惯量，但该行未明确单位及折算坐标，不能直接作为云台侧 `J`。 |
| `Damp` | 5.2233e-4 | 寄存器读数，不能直接等同于整台云台的粘性阻尼。 |
| `PMAX/VMAX/TMAX` | 12.5 / 30 / 10 | 样机的 MIT 编码范围；不是机械限位或连续运行额定值。 |
| 作者推算的输出轴 `Kt` | 0.94324 N·m/A | 由磁链、极对数、内部减速比和效率因子计算；作者明确指出 DM 的电流/力矩校核存在代数循环，未形成独立物理测量。 |

如果要把转子惯量反射到输出轴，首先必须确认寄存器代表哪个轴、单位是什么，以及正式模型是否已经包含该项。仅在确认是转子侧惯量且尚未计入时，才可使用减速比平方反射；还需要加入减速器和真实云台负载的对应惯量。不能把上表单体读数用作整个云台的惯量。

**接口的传动比必须避免重复换算。** 本项目 DM4310 MIT 角度、速度和力矩的“电机侧”应理解为电机模组的协议输出侧。`mechanics.gear_ratio` 应表示该输出轴与云台关节之间的外部传动比：模组输出轴直连云台时填 1，不因为 DM4310 内部有 10:1 减速器再次填 10。MIT 输出量已经处于模组输出侧，内部减速比不能再对命令或反馈重复使用。

达妙 [官方型号参考表](https://github.com/dmBots/motor-control-routine/blob/master/orin%E8%BD%BD%E6%9D%BFcan%E6%8E%A7%E5%88%B6%E8%BE%BE%E5%A6%99%E7%94%B5%E6%9C%BA%E4%BE%8B%E7%A8%8B/dm_hw/src/hardware_interface/damiao.cpp) 也区分 DM4310 与 DM4310_48V 的速度编码范围；仍应以实际固件与调试参数为准。

## 4. 机械开源文章：工作角度可以引用，重力参数仍然缺失

港科大 ENTERPRIZE 的 [2025 中供弹英雄原始开源说明](https://bbs.robomaster.com/article/761377?source=8) 给出 pitch 工作范围 −18° 至 +45°，对应 DM4310 驱动，并介绍了气弹簧重力补偿结构。

这能支持该台机械方案的公开工作范围和执行器配置，不能直接提供本项目所需的负载惯量、质心偏距、气弹簧力—行程曲线或剩余重力矩幅值。将其换算成某个 `gravity_amplitude_nm` 会新增未经证实的假设，因此本轮没有据此补造重力参数。

## 5. 本轮采用规则

- 正式仿真的质量、质心、惯性矩阵和关节坐标来自 `sim/open_models` 中实际归档、经过坐标核对的模型。
- 官方 1 ms 调度配置可作为离散控制周期的来源证据，但运行抖动与 STM32 执行时间需要独立验证。
- 工程等效辨识参数、单体电机寄存器和纯 PID/归一化参数分别保留原义，不混成一份“实测云台参数”。
- 本文件没有宣称已运行 Combat 跟随工况、DM 单体台架复现或 ENTERPRIZE 机械模型。实际完成哪些仿真，以仿真脚本、结果和报告为准。
