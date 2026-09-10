# 从实测日志到候选模型参数

[文档导航](README.md) · [在线辨识](ONLINE_IDENTIFICATION.md) · [工程结构](PROJECT_STRUCTURE.md)

本页介绍电脑端离线辨识工具：由独立训练、验证日志估计惯量、摩擦和重力系数，再导出 C 常量。STM32 继续运行现有 C99 控制器；工具不连接电机、不自动改写控制增益。

需要在 STM32 上逐次拟合时，使用独立的 [C99 在线 RLS 与积分转换模块](ONLINE_IDENTIFICATION.md)。离线工具便于检查整段日志及独立验证集，在线模块输出当前模型和数据质量；两者都只提供候选参数，由应用层决定何时采纳。

**V1 适用于固定底座、固定构形、已标定关节力矩的单轴实验。** 不识别运动底座、折叠过程、轴间耦合或电机内环延迟。附带数据全部为合成数据，没有实机辨识结果。

## 1. 先运行完整演示

除普通主机构建环境外，需要 Python 3.9+、NumPy 和主机 C99 编译器。建议在自己的 Python 虚拟环境中安装可选依赖：

```sh
python3 -m pip install -r requirements-identification.txt
python3 tools/validate_identification.py --output-dir build-identification
```

脚本依次生成 Yaw/Pitch 的独立解析训练与验证轨迹，辨识参数，导出头文件，再编译并运行实际的 `gimbal_smc.c`、`gimbal_motor.c`。输出包括：

| 路径 | 内容 |
|---|---|
| `build-identification/yaw/`、`pitch/` | CSV、元数据、合成真值、候选报告与 C 头文件 |
| `build-identification/summary.json` | 两轴、两种电机适配及两种负载条件下的闭环指标 |

已归档本次[闭环结果](../sim/results/identification/summary.json)、[Yaw候选报告](../sim/results/identification/yaw_report.json)及[Pitch候选报告](../sim/results/identification/pitch_report.json)。闭环对象接收实际组包后独立解码的名义力矩，仍假设声明的电机力矩比例成立。

辨识数据为无噪声解析轨迹；闭环另用不同轨迹，并加入确定性测量扰动、3 ms 命令延迟与3 ms一阶执行器滞后。压力场景将真实惯量增加20%、重力系数增加10%，并加入外扰。这些是**人为规定的合成压力条件，不是辨识置信区间**。

共运行8个候选模型案例，以及8个相同增益的粗模型对照。粗模型人为取 `J×0.7、B×0.5`，关闭摩擦/重力前馈；它不是原开源项目或实机已整定控制器。通过对照只能说明这组模型和实验下，辨识参数发挥了预期作用。

候选闭环门槛预设为 RMS≤0.035 rad、峰值≤0.10 rad、限幅事件≤100，统计区间为1–18 s；另检查连续Yaw终角超过两圈及RMS低于粗模型。具体结果以生成的报告为准。方向统计同时记录样本数，缺少某方向样本时输出 `null`。

可选测试入口：

```sh
cmake -S . -B build-id -DCMAKE_BUILD_TYPE=Release -DGIMBAL_BUILD_IDENTIFICATION=ON
cmake --build build-id --parallel
ctest --test-dir build-id --output-on-failure
```

当前完整环境的默认构建包含 13 项 CTest；开启上述选项后再增加离线辨识检查和导出参数闭环两个入口，共 15 项。`GIMBAL_BUILD_IDENTIFICATION` 仅控制电脑端 Python/NumPy 检查，默认关闭；在线 C 模块始终参与常规库构建，不受该选项控制。模块依赖见 [工程结构](PROJECT_STRUCTURE.md)，已实际执行的结果见 [验证记录](VALIDATION.md)。

## 2. 辨识什么模型

固定底座时，关节相对角和所用控制角之间应仅相差常量。统一角度正方向，以关节侧 N·m 表示力矩：

$$
\tau=J\ddot q+B\dot q+F_c\tanh(\dot q/\epsilon)
+A\sin q_g+C\cos q_g+r.
$$

`q` 为连续关节角，`q_g` 为标定平面内的重力相位；`r` 为未解释的作用。固定正值 `epsilon` 后，Yaw辨识 `J、B、Fc`，Pitch再辨识 `A、C`。Yaw路径假定重力绕该轴的投影可忽略；倾斜Yaw不能直接套用这个三参数模型。

为避免直接差分陀螺速度，在短窗口内建立积分方程：

$$
\int\tau\,dt=J\Delta\dot q+B\Delta q+
F_c\int\tanh(\dot q/\epsilon)\,dt+
A\int\sin q_g\,dt+C\int\cos q_g\,dt+\int r\,dt.
$$

工具用梯形积分并除以各窗口实际时长，再对回归列归一化后做SVD最小二乘。默认窗口50 ms，可按采样率和实验频带调整。窗口积分区间不重叠，但相邻窗口会共享端点，因此不能将它们视为独立随机样本。

训练和验证力矩若同时乘以错误比例，全部力矩参数可同比缩放而残差仍很小；重力角的一致零偏也可被`A/C`旋转吸收。测试保留了这两种可通过拟合的反例，说明独立力矩标定与坐标校验不可由残差门槛替代。

完整往复周期的 `Δq、Δqdot` 可能都为零，无法辨识惯量和阻尼；不同速度、加速度与角度必须得到充分激励。工具分别检查训练和验证矩阵的秩、条件数及窗口数量，但这不是全工况持续激励证明。

## 3. 最小侵入的日志接入

保留原工程的模式、INS、CAN与控制任务。建议在反馈快照完成后记录反馈，在最终执行器输出确定后记录命令与状态；由低优先级任务导出，避免在控制周期内阻塞打印。原始日志应保留请求力矩、最终命令、实际力矩反馈、时间戳和模式；整理成下列精简CSV再交给工具。

**`torque_nm` 必须是已对齐时间的实际关节力矩估计，来源是经过标定的反馈。** 电机报文中的名义给定、`applied_joint_nm`、SMC请求都不能直接充当它。力矩比例未知时，只能得到参数组合，不能将其当作真实kg·m²惯量。GM6020反馈电流比例需要按固件单独核验，详见[电机说明](MOTOR_PROTOCOL.md)。

每份CSV仅包含一根轴、一个固定构形/负载，首行严格为：

```csv
time_s,angle_rad,rate_rad_s,gravity_angle_rad,torque_nm,enabled,valid,saturated,segment
```

| 字段 | 约定 |
|---|---|
| `time_s` | 秒，严格递增；不可将重复缓存读取当作新采样 |
| `angle_rad` | 连续展开关节角；Yaw跨圈不回绕到±π |
| `rate_rad_s` | 同坐标角的导数；运动底座不在V1范围内 |
| `gravity_angle_rad` | 固定平面中的重力相位，不能未经转换使用机械角 |
| `torque_nm` | 标定并对齐的关节侧力矩 |
| `enabled/valid/saturated` | 严格为0或1；限幅包括算法、适配器和已知执行器限流 |
| `segment` | 整数阶段号，切换时断窗；不同构形需要分别辨识，不能仅换阶段号混合拟合 |

所有行都要求有限数值。缺测行可填有限占位值并将`valid=0`；失能、无效、饱和行及大时间间隔会切断窗口。默认最大相邻采样间隔20 ms，可由`--max-gap-s`调整。

元数据示例（只用于合成演示）：

```json
{
  "schema_version": 1,
  "axis": "pitch",
  "configuration": "synthetic_fixed_plane_v1",
  "data_kind": "synthetic",
  "torque_source": "synthetic_truth",
  "torque_time_aligned": true,
  "base_motion": "fixed",
  "angle_continuous": true,
  "gravity_frame": "fixed_plane",
  "friction_velocity_rad_s": 0.075
}
```

实测数据使用`data_kind=hardware`与`torque_source=calibrated_feedback`；必须先完成对应标定与同步。元数据是实验声明，工具不能代替实际测量验证这些声明。`friction_velocity_rad_s`是固定的平滑摩擦速度尺度，不是工具拟合出的参数；应比较不同选择在独立数据上的效果。

## 4. 实验与输出

1. 固定底座与构形，核对轴零位、方向、反馈单位和力矩比例。Pitch先检查多个角度的支撑余量。
2. 结合多角度保持和同角度双向低速通过，检查重力、静摩擦与滞回。`tanh(0)=0`不能描述真实静摩擦支撑，不应仅靠静止点估计全部重力。
3. 在稳定闭环中施加受限的多频激励，包含正反向、多速度和加减速段。记录独立参考/激励，供后续闭环偏差诊断。
4. 另做验证实验，改变频率、相位或允许范围内的固定底座倾角，保持待辨识机构/负载一致。不要随机打散同一条轨迹后称作独立实验。

```sh
python3 tools/identify_gimbal.py \
  --train build-identification/pitch/train.csv \
  --validation build-identification/pitch/validation.csv \
  --metadata build-identification/pitch/metadata.json \
  --max-validation-rmse-nm 0.003 \
  --output build-identification/pitch/candidate.json \
  --header build-identification/pitch/candidate.h \
  --symbol-prefix pitch_ident
```

这里0.003 N·m是合成演示门槛，不能作为实机通用标准；它衡量的是**窗口平均力矩残差**，不是逐点力矩误差或角度精度。工具检查正惯量、非负摩擦、float32可表示性和独立验证表现，失败则返回非零，不发布本次候选结果。失败时可能保留之前成功输出；只采纳本次退出成功且数据哈希匹配的报告。

JSON包含参数、单位、数据/元数据/工具哈希和验证指标，状态始终为`candidate_only`。C头文件只包含参数常量；使用不同`--symbol-prefix`分别导出Yaw/Pitch，避免多轴名称冲突。

导出前还会把参数转换为float32，包含摩擦速度尺度，再重建回归窗口、复验同一误差门槛。该检查针对参数量化；浮点控制运算另由C闭环演示覆盖。

| 候选值 | 接入位置 |
|---|---|
| `inertia_kg_m2`、`viscous_nm_s_rad` | 对应SMC配置字段 |
| `gravity_sin_nm`、`gravity_cos_nm` | Pitch模型的`sin_nm`、`cos_nm` |
| `coulomb_nm`及固定摩擦速度尺度 | 应用层计算摩擦前馈，与重力一起送入`feedforward_nm` |

核心已计算`B*omega`，前馈中不要重复补偿该项。该工具不选择终端项，也不生成自动生效的控制增益。

## 5. 从模型到滑模整定

在线性滑模、连续时间、模型/前馈匹配、无滤波/延迟/限幅且处于边界层内时，令`kappa=k+eta/phi`，可得：

$$
\ddot e+(\lambda+\kappa)\dot e+\lambda\kappa e=0.
$$

局部极点为`-lambda、-kappa`。选择响应速度时，要同时考虑`eta/phi`，并满足`k=kappa-eta/phi>=0`；该正增益结构只对应实极点，不能任意指定欠阻尼二阶参数。实际采样、参考生成、滤波和限幅后，需要用C闭环重新验证。

建议先关闭终端项，依据测得频响、延迟、结构振动和力矩余量选保守响应速度；用实测滑模面噪声选择边界层，再根据独立残差与扰动实验调整抗扰增益。窗口平均残差不能直接当作瞬时扰动上界。Pitch还需按方向扣除重力、摩擦所占力矩，分别检查加速与制动，而不能将相同参考加速度用于所有角度和方向。

积分最小二乘不会自动消除闭环噪声相关性或回归量误差。若出现随激励、闭环增益变化的参数漂移，应进一步比较工具变量、输出误差/预测误差方法和时延模型；V1没有实现这些估计器，也不输出统计置信区间。

## 6. 折叠构形与在线模块

折叠Pitch先固定工作位分别辨识，检查不同构形的惯量和重力变化。折叠运动本身涉及变惯量、几何关系及耦合，不能把多份固定模型直接当作整个过程已验证；见[多轴扩展方向](MULTI_AXIS.md)。同一构形与负载下，手瞄和自瞄可共用物理模型，但应分别验证参考输入和切换行为。

本库现已提供 [gimbal_rls](../include/gimbal_rls.h) 与 [gimbal_identification](../include/gimbal_identification.h) 两个在线 C99 模块：前者更新有限维回归参数，后者从逐次反馈形成积分窗口，并检查时间、构形与数据资格。它们不改写 SMC，也不自动选择控制增益；完整调用流程、清观测与复位的区别见 [在线辨识说明](ONLINE_IDENTIFICATION.md)。

在线模块同样要求固定底座、固定构形和已标定的同步力矩，不能将“在线计算”理解为已经支持折叠中的耦合辨识。采纳参数仍需独立验证与回退路径。[RM2024-PowerModule](https://github.com/hkustenterprize/RM2024-PowerModule/blob/23613f8544a44060e91c4fc33b68c9668cfb5f3b/Utils/RLS.hpp)提供了递推估计的工程参考，但其对象是底盘功率损耗；本库的云台模型、数据窗口和数值处理由独立 C 实现承担。
