# 可离线复算的 RoboMaster 开源云台参数

这些参数来自作者公开的机器人描述模型，用于提高合成闭环仿真的机械参数依据。它们不是本项目云台的实测辨识值，也不能证明 DM4310 / GM6020 的实机性能。控制器、摩擦、扰动、延迟、噪声和扭矩上限仍由仿真实验另行指定；本目录不从 URDF 的 `effort` 字段推断电机容量。

## 复算

在项目根目录执行，Python 3.9+ 标准库即可，不依赖 ROS、xacro、NumPy 或网络：

```sh
python3 tools/derive_open_model_profiles.py
python3 tools/derive_open_model_profiles.py --check
```

第一条命令生成 [profiles.csv](profiles.csv) 和 [details.json](details.json)。第二条只读核验所有原始文件的 SHA256、Git blob SHA，并重新推导后比较两个输出。`--root DIR` 可指定另一个归档目录。CSV 使用 12 位有效数字，模型名称是稳定标识。

| model | axis | J / kg·m² | gravity_sin / N·m | gravity_cos / N·m | 验证区间 / rad |
|---|---|---:|---:|---:|---|
| rmoss_rmua19 | yaw | 0.00608000833333 | 0 | 0 | [-0.5, 0.5] |
| rmoss_rmua19 | pitch | 0.004898175 | 0 | -0.220725 | [-0.785, 0.5652] |
| dynamicx_standard3 | yaw | 0.00713692704547 | 0 | 0 | [-0.5, 0.5] |
| dynamicx_standard3 | pitch | 0.00975136641445 | -0.45483175984 | -0.361892836604 | [-0.72, 0.457] |

**两个 yaw 的 ±0.5 rad 都是本项目选定的验证区间，不是声称模型机械限位。** RMOSS 原模型 yaw 限位为 ±1.57 rad；Standard3 使用很大的 ±1e10 rad。Pitch 区间直接取原始模型限位。

## 来源与建模等级

- `rmoss_rmua19`：[robomaster-oss/rmoss_gz_resources](https://github.com/robomaster-oss/rmoss_gz_resources/tree/20530be7afda1c6a15cab90a46706aa58463acba)，固定 commit `20530be7afda1c6a15cab90a46706aa58463acba`，读取 `resource/models/rmua19_standard_robot/model.sdf`。作者 [README](sources/rmoss_gz_resources/README.md) 说明几何图纸来自官方 RM AI 机器人资料，模型由维护者构建。公开的 SDF 参数可用于模型仿真，不是官方发布的实测惯量。原始模型没有相机载荷；README 中另加相机的示例没有被混入本次模型。
- `dynamicx_standard3`：[rm-controls/rm_description](https://github.com/rm-controls/rm_description/tree/a4b31828bf964e491d3ba5a731d12c1213562feb)，固定 commit `a4b31828bf964e491d3ba5a731d12c1213562feb`，入口 `urdf/standard3/standard3.urdf.xacro`，所有 `load_*` 使用源码默认 `true`。这是 DynamicX 作者公开的机器人模型常量；归档文件没有说明惯量是否实测或如何由 CAD 导出，因此这里不作此断言。相机虚拟光学坐标系有一处明确、有限的模型修正，见下文。

[manifest.json](manifest.json) 逐一记录 11 个原始文本文件的仓库、commit、上游路径、永久链接、SHA256、Git blob SHA 和许可。`sources/` 保留原始字节，包括换行。`details.json` 为每个 link 和 joint 提供源码行号；宏生成的相机、摩擦轮和 IMU 同时记录模板与实例化位置，便于查安装位姿。

## 完整转动子树与坐标约定

基座固定且水平，世界重力为 `(0, 0, -9.81)` m/s²；所有关节位置与速度取零。模型中的 joint origin、axis、link inertial origin 都参与坐标变换，RPY 使用 `Rz(yaw) Ry(pitch) Rx(roll)`。SDF 的 `relative_to` 按模型语义解析，零位不会被强行替换为电控 IMU 的安装零位。

- RMOSS yaw 子树：`gimbal_yaw`、`gimbal_pitch`、`speed_monitor`；pitch 子树为后两者。转动总质量分别为 1.6 kg 与 0.6 kg。
- Standard3 yaw 子树：`yaw`、无质量 `supply_frame`、`pitch`、左右摩擦轮、`camera_link`、`camera_optical_frame`、`gimbal_imu`；pitch 子树去掉前两者。转动总质量分别为 1.33709 kg 与 1.13709 kg。原模型的 `trigger`、`cover` 安装在固定 `base_link`，不随所选轴转动，因此不计入。

所有子关节（包括摩擦轮）在本提取中锁定，不加未在模型里出现的转子或传动反射惯量。这样得到所选姿态的锁定子树等效惯量，而不是整台运动机器人在所有姿态下的完整动力学。

令关节世界单位轴为 `u`，第 i 个 link 的质心相对关节原点位置为 `r_i`，惯量坐标系到世界的旋转为 `R_i`，原始质心惯量为 `I_i`，质量为 `m_i`，则：

```text
J = Σ [ uᵀ R_i I_i R_iᵀ u + m_i (r_iᵀ r_i - (uᵀ r_i)²) ]
```

因此这里没有将单个 link 的 `iyy` 或 `izz` 当作完整云台惯量。`details.json` 列出各 link 旋转后的完整张量、质心位置、质心惯量投影及平行轴贡献。

Pitch 的其余子关节锁定后，其自身 J 不随 pitch 角变化。Yaw 的 J 则通常随冻结的 pitch 角变化；CSV 取 pitch=0。例如 RMOSS 可由同一组模型常量独立得到：

```text
J_yaw(pitch) = 0.002043183333333333 + 0.004036825*cos(pitch)^2
```

本文件中的数值不包含双轴同时运动的耦合、科氏项、基座运动、摩擦轮高速旋转的陀螺项或额外线束力矩；这些需要完整机器人动力学或台架辨识。

## 重力补偿的符号

CSV 中的重力字段表示需要施加的保持补偿力矩，使用下式：

```text
tau_g(q) = gravity_sin_nm*sin(q) + gravity_cos_nm*cos(q)
J*qdd = command_torque - B*qdot - tau_g(q) + disturbance
```

设 `P = Σ m_i*r_i`、`P_perp = P - u*(uᵀP)`、世界重力向量为 `g`，则：

```text
gravity_sin = -uᵀ[(u × P) × g]
gravity_cos = -uᵀ[P_perp × g]
```

例如 RMOSS pitch 零位受重力实际驱动为 **+0.220725 N·m**，需要的保持补偿为 **-0.220725 N·m**。`details.json` 同时存储 actual gravity drive 和 required hold torque，避免混淆反号。Standard3 两个系数均非零，这是负载质心具有 x、z 偏移的结果。基座水平时，两模型的竖直 yaw 轴重力补偿为零。

## 唯一模型修正：虚拟光学坐标系

Standard3 引用的 [camera.urdf.xacro 第 39–43 行](https://github.com/rm-controls/rm_description/blob/a4b31828bf964e491d3ba5a731d12c1213562feb/urdf/common/camera.urdf.xacro#L39) 为 `camera_optical_frame` 分配 0.001 kg 质量及下列 kg·m² 张量：

```text
[[ 2.129e-9,  7.329e-9, -1.110e-9],
 [ 7.329e-9,  2.198e-9, -8.040e-9],
 [-1.110e-9, -8.040e-9,  2.197e-9]]
```

其特征值约为 `[-8.15792145541e-9, 1.05464842572e-9, 1.36272730297e-8]`，含负值，不是物理有效的惯量张量。本工程仅对这个固定来源、名字、质量和完整数值均精确匹配的虚拟 link 使用**零质心惯量的点质量近似**。保留其 1 g 质量、原始安装位置与旋转、平行轴贡献及其 IMU 子 link；没有把相机或 IMU 删除。

该修正对任意单位轴的质心惯量投影改变量上界为 `1.36272730297e-8 kg·m²`，约为所得最小 Standard3 关节惯量的 0.000191%。这只是原异常张量替换的算术影响上界，不是模型准确度保证。原文保持不变，修正前后矩阵、特征值、理由与上界均写入 `details.json`；遇到其他非物理张量会失败。

## 解析范围与许可

脚本是固定版本模型的受限提取器，不是通用 xacro/SDF 引擎。只允许明确的有限数值、变量名与 `+ - * /` 常量表达式；函数调用、属性访问、下标、幂运算和未知替换会被拒绝，不使用 `eval`。Standard3 只展开经过核对的 gimbal、shooter、两个摩擦轮、相机与 IMU 组合；底盘、传动和 Gazebo 插件不属于固定基座转动子树。输入哈希不符时必须先人工审查新版本，不能直接套用旧配方。

归档的 RMOSS SDF XML 按 [Apache-2.0](sources/rmoss_gz_resources/LICENSE) 提供；作者 README 对 mesh 另有版权说明，本目录没有再分发 mesh。rm_description 文本保留作者的 [BSD-3-Clause](sources/rm_description/LICENSE) 许可。两份原始 README 和 LICENSE 均原样保留；这些第三方文件的许可不被本项目主许可替换。
