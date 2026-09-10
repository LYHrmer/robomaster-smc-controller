# 通用在线辨识：STM32 最小二乘参数拟合

[文档导航](README.md) · [离线辨识](IDENTIFICATION.md) · [工程结构](PROJECT_STRUCTURE.md)

在线模块使用递推最小二乘（RLS），每次收到新数据更新少量状态，无需保存整段日志或在STM32运行Python。控制器继续输出关节力矩；辨识器独立读取标定反馈，输出模型候选。**拟合与将参数应用到闭环是两个独立步骤。**

**本页由 GM6020 和 DM4310 共用。** 先按 [GM6020 采集与标定](GM6020_IDENTIFICATION.md) 或 [DM4310 采集与标定](DM4310_IDENTIFICATION.md) 准备反馈，再接下面的 C99 接口。辨识器读取统一的关节侧数据，不直接接收两种电机的原始 CAN 报文。

## 1. 两层接口

| 层 | 源文件 | 输入与用途 |
|---|---|---|
| 通用RLS | [gimbal_rls.h](../include/gimbal_rls.h)、[gimbal_rls.c](../src/gimbal_rls.c) | 最多5项回归特征与一个观测值；应用可提供自己的线性参数化模型 |
| 云台积分转换 | [gimbal_identification.h](../include/gimbal_identification.h)、[gimbal_identification.c](../src/gimbal_identification.c) | 接收带时间戳的角度、速度、重力角和实际力矩，形成回归窗口 |

两层均为C99、单精度、固定大小数组，无堆内存、HAL、RTOS或CAN发送依赖。每根轴或每个固定构形使用独立实例，由一个任务更新。只使用SMC时不需要添加这两个模块。

Yaw模型拟合`J、B、Fc`，要求重力绕该轴的投影可忽略；Pitch模型再拟合有符号`A、C`。摩擦速度尺度`epsilon`固定：

$$
\tau=J\ddot q+B\dot q+F_c\tanh(\dot q/\epsilon)+A\sin q_g+C\cos q_g+r.
$$

转换层与离线工具采用同一积分方程，默认每50 ms形成一行回归数据。角加速度项使用窗口两端的速度差，不直接微分每拍陀螺数据；非线性项逐样本积分，不能先平均速度再计算`tanh`。

**当前云台转换层要求固定底座、固定构形、同步且标定的关节力矩。** 在线计算不代表已经支持行驶中、折叠中或三轴耦合辨识。这些工况需要进一步建模，并由应用向通用RLS提供相应特征。

## 2. 如何接入已有工程

将`src/gimbal_rls.c`、`src/gimbal_identification.c`加入工程并添加`include/`路径；GCC链接数学库。禁止`-ffast-math`。辨识器本体应静态分配，不要放在短生命周期任务栈上。

每轴初始化一次：

```c
#include "gimbal_identification.h"

static gimbal_identification_t pitch_identifier;

bool identification_start(const gimbal_identification_config_t *calibrated_config)
{
    return gimbal_identification_init(&pitch_identifier, calibrated_config);
}

void identification_tick(const gimbal_identification_sample_t *snapshot,
                         gimbal_identification_output_t *result)
{
    gimbal_identification_update(&pitch_identifier, snapshot, result);
    /* result->ready is a NEW candidate event, not a command to change SMC. */
}
```

`gimbal_identification_default_config()`可作为配置结构起点，数值只是合成示例。初始化前按自己的机构设置下面几类配置，并检查返回值：

| 配置 | 含义 |
|---|---|
| `axis`、`configuration_id` | 模型维数和所属固定构形/负载；每个输入样本必须匹配 |
| `window_s`、`max_gap_s`、`feedback_timeout_s` | 积分窗口、采样间隔及反馈年龄限制；RLS的dt范围要覆盖可能的完整窗口 |
| `friction_velocity_rad_s` | 固定摩擦速度尺度，与离线模型约定一致 |
| `rls.initial/minimum/maximum` | 初值与物理边界；惯量下界必须为正，阻尼/摩擦下界非负，重力系数可有正负 |
| `feature_scale/output_scale` | 固定的特征与观测归一化尺度，适应不同物理量级 |
| `forgetting_time_s` | 有效辨识数据的记忆时间；0表示不遗忘，正值用于跟踪缓慢变化 |
| `excitation_window`、能量/主元门槛 | 有限滑窗的信息检查，最多32行；并非统计可辨识性证明 |
| `min_updates`、`innovation_rms_limit` | 连续有效更新数及更新前预测残差RMS门槛 |

原工程提供一个一致快照：

| 输入 | 要求 |
|---|---|
| `timestamp_us` | 新采样的32位微秒时间戳，支持自然回绕；不能用重复读取缓存的时间代替 |
| `angle_rad/rate_rad_s` | 连续关节角及其同坐标导数；大Yaw必须保留圈数 |
| `gravity_angle_rad` | 固定平面中的重力相位；与机械相对角分开 |
| `torque_nm` | 同步、标定后的关节力矩反馈；不能直接用命令或`applied_joint_nm`代替 |
| `feedback_age_s` | 所需反馈中最旧样本的年龄 |
| `enabled/valid/saturated` | 辨识使能、数据有效性和已知输出限幅状态 |
| `operating_conditions_valid` | 上层确认固定底座/构形及当前模型适用 |
| `torque_calibrated/torque_time_aligned` | 上层已独立完成标定和时间对齐的声明 |
| `segment_id/configuration_id` | 连续采集阶段及固定机构配置标识 |

手瞄、自瞄可共用同一轴辨识实例。切换造成数据或参考不连续时更换`segment_id`；不同折叠构形用另一实例或明确重新初始化，不能仅更换阶段号混合参数。发生角坐标重定位时也要中断观测窗口。

## 3. 什么时候更新，什么时候冻结

| 情况 | 参数和P | 观测窗口与候选 |
|---|---|---|
| 正常新样本，积分窗口未完成 | 保持 | 累积积分；本拍`ready=false` |
| 完整窗口但信息不足 | 冻结，不施加遗忘 | 刷新特征滑窗，清候选连续计数/创新记录 |
| 信息足够且数值检查通过 | 递推更新 | 满足连续更新数与创新RMS后才给新候选 |
| 拟议更新越过物理边界 | 投影并回写内部参数 | 标记`PROJECTED`，本拍不ready |
| 失能、坏数据、饱和、超时或阶段中断 | 保留 | 清部分积分、激励和创新历史 |
| 构形ID不匹配 | 保留，不混合 | 拒绝该样本，清观测历史 |
| 非有限数、协方差越界/失去正定性 | 本次参数/P更新回滚 | 报告故障，不输出可采纳候选 |

`gimbal_identification_clear_observations()`保留参数与P；`gimbal_identification_reset()`恢复配置初值与P0。长时间暂停不累计遗忘，恢复后重新积累新鲜激励。原始时间戳重复、倒序或间隔过大不会跨接积分。

32位时间戳的顺序判断要求相邻调用间隔小于`2^31`微秒（约35.8分钟）。如果停止采样更久或计时器重新启动，恢复前调用`clear_observations()`建立新的时间基准；参数仍保留。库不会把无法区分的长间隔自行当成正常回绕。

`result.flags`与`result.rls.flags`是两个独立位域；`window_updated`表示本拍新窗口被RLS接受，`ready`进一步表示候选门槛满足。未满窗和失败拍均不会复用上一拍的ready。`P`及其对角线仅是归一化算法状态，不输出统计置信区间。

## 4. 递推与数值处理

以特征尺度`s_i`、力矩尺度`s_y`归一化：

$$
x_i=\varphi_i/s_i,\quad y=\tau/s_y,\quad
w_i=\theta_i s_i/s_y,\quad \theta_i=w_i s_y/s_i.
$$

每个接受的完整窗口按其时长`dt`计算遗忘因子，`rho=exp(-dt/T_f)`；不遗忘时`rho=1`。令`P^-=P/rho`、`S=1+x^TP^-x`，使用：

$$
K=P^-x/S,\quad w^+=w+K(y-x^Tw),\quad
P^+=(I-Kx^T)P^-(I-Kx^T)^T+KK^T.
$$

这就是归一化观测方差取1的Joseph形式。实现还会对称化、检查正定性和P上界。投影约束针对物理参数，之后映射回内部状态，避免只夹紧显示值而内部继续漂移。

激励检查先检查各列平均能量，再对归一化相关矩阵做Cholesky主元检查。它可以拒绝部分静止、共线和弱激励情况，不能自动识别错误的力矩比例、坐标标定或遗漏的动力学项。

`P0`越大，早期更新通常越激进；反复投影可能影响后续有限数据上的估计。先按机构量级配置初值、尺度与边界，观察`PROJECTED`和创新记录，再调整激励门槛与`P0`，不能仅以更快出现`ready`为整定目标。

## 5. 如何使用候选参数

建议先记录`model`、时间戳、构形ID、`accepted_updates`和创新指标，使用独立轨迹和离线工具复核。只有上层明确接受后，才通过原工程的配置切换流程应用`J/B`与摩擦、重力前馈。

采样日志、候选参数及参数采纳记录的保留规范见 [测试数据清单与模板](TEST_DATA.md)。

参数应用需保留原有力矩约束、模式管理和参考重建流程。候选`J`改变也会改变实际反馈力矩；仅仅保持滑模增益数字相同，并不代表切换无扰。转换层和RLS自身都不会写入`gimbal_smc_t`、改变电机使能或发送CAN。

`lambda/k/eta/phi`是控制增益，不是这组RLS拟合的物理参数。其选择仍按[控制器设计](CONTROL_DESIGN.md)与[Pitch整定](PITCH_TUNING.md)验证。

## 6. 验证与适用范围

普通CMake主机测试包含通用RLS、流式积分转换和[在线闭环验证程序](../sim/validate_online_identification.c)。后者使用实际C控制器、电机组包后的独立字节解码和合成力矩反馈；分别比较保持初始模型与一次显式采纳在线候选的两种运行。

[长期数值回归](../tests/test_rls_endurance.c)另用12万次五维观测检查缓慢变化参数、有限噪声、失能与低激励恢复。每拍用独立double分解检查P的正定性，并对冻结状态逐位比较；评分剔除开始或中断后的800拍恢复段。[结果CSV](../sim/results/online_identification/endurance_metrics.csv)是归一化参数误差，不能当作云台角度精度或STM32时序指标。

闭环验证包含连续Yaw、Pitch重力、新鲜时间戳回绕、拟合暂停/恢复以及新的评分轨迹。其门槛和结果见[验证记录](VALIDATION.md)。仿真里的参数采纳是验证程序明确执行的动作，不是库内自动改参。

可复现结果保存在[指标CSV](../sim/results/online_identification/metrics.csv)与[条件/源码摘要](../sim/results/online_identification/summary.json)。其中`ready_time_s`是24秒后首次采纳候选的时刻，不是从启动到收敛所需时间；两类运行行中的`J/B/Fc/A/C`均记录候选，`parameters`列说明控制器使用初值还是该候选。

28秒直接切换参考频率会产生参考跳变，随后2秒转换段不计入跟踪指标，评分区间为30至44秒。因此这里的RMS/峰值不是全程误差或参数切换瞬间的保证。

Cortex-M4F hard-float、GNU Arm 13.2.1目标ABI下，RLS实例占1028字节，完整云台辨识实例占1328字节（已包含RLS）。三根轴各一实例合计3984字节，不含任务栈、采样缓存与控制器。`-O2 -fstack-usage`报告两个更新函数自身栈帧分别为800和136字节；实际调用还包含子函数、数学库及中断栈，不能据此直接设定任务栈大小。更新的计算量有固定上界，但尚未在STM32测量执行时间。

积分/RLS没有自动消除闭环测量噪声偏差。力矩统一缩放与重力角一致零偏仍可能通过残差检查。传动几何、折叠或底座运动违反当前模型时，需要重新建模，而不能依赖遗忘因子掩盖差异。

数学背景可参考[MIT机械系统辨识讲义](https://underactuated.mit.edu/sysid.html)及[MathWorks递推最小二乘说明](https://www.mathworks.com/help/ident/ref/recursiveleastsquaresestimator.html)。本模块独立实现，不依赖这些平台，也未复制其源码。
