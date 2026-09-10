# 验证记录

更新日期：2026-09-10。主机环境：Linux、GCC 11.4.0、C99；交叉工具链：GNU Arm Embedded 10.3-2021.10（GCC 10.3.1）。以下区分主机执行、Arm 静态库编译与尚未进行的硬件测量。

## 已执行

| 检查 | 结果 |
|---|---|
| Debug，`-Wall -Wextra -Wpedantic -Werror` | 编译通过，9/9 CTest 通过 |
| Release，同样的警告检查 | 编译通过，9/9 CTest 通过；测试目标显式保留断言 |
| Debug + AddressSanitizer/UBSan | 编译通过，9/9 CTest 通过 |
| 可选离线辨识，Release | 11/11 CTest 通过；新增入口内含21项Python检查及8个候选C闭环案例 |
| Cortex-M4F hard-float 交叉编译 | `gimbal_smc`、`gimbal_motor`、`gimbal_pitch`、`gimbal_example` 四个静态库通过；对象属性确认 Thumb-2／VFPv4／VFP 参数 ABI |
| 控制核心接口单测 | 方向、过零制动、前馈、角度回绕、限幅/斜率/滤波、超时、NaN/Inf、错误周期及恢复通过 |
| 正则化终端项 | 导数与独立 double 中心差分对照通过 |
| 电机协议单测 | 独立字节向量、方向/传动、量化/限幅、非法配置、DLC/ID、DM 故障、GM 四槽组帧通过 |
| 接入单测 | 首次参考锚定、陈旧反馈、恢复、同组其他槽保留、DM ARMING 不发送 FD 通过 |
| Pitch 接口及约束单测 | 重力符号、独立反馈坐标、参考包络、隐藏参考状态与故障锁存等回归通过 |
| 合成仿真 | 12 条闭环与 3 条目标差分诊断通过预设回归包络 |
| 开源模型参数复算 | 11 份固定版本原文件的 SHA256/Git blob 校验、2 模型/4 关节惯量和重力离线复算通过 |
| 开源模型闭环 | 576 PASS、0 FAIL、0 INFEASIBLE；两种电机实际组包后独立字节解码 |
| Pitch 专项闭环 | 20 个验收场景通过；4 个错误坐标诊断保留原始 PASS，但单列且不作为正确坐标证明 |
| Pitch 力矩不可行负路径 | 从归档 Standard3/pitch 参数读取，±0.25 N·m 下两种电机均正确识别 INFEASIBLE |
| 开源验证器负路径 | 10 类无效输入拒绝；超能力、失稳合成样例分别返回不可行/失败，未运行指标为 NaN |
| 图件 | 保留原合成及开源模型图；新增 pitch 方向误差和重力坐标对照两组 PNG/SVG/PDF，实际打开检查排版 |

默认 9 个 CTest 名称为 `smc`、`motor`、`example`、`pitch`、`simulation`、`open_model_validation`、`pitch_validation`、`pitch_validator_negative`、`open_model_provenance`。最后一项依赖 Python 3.9+；本轮测试环境已安装 Python。

2026-09-10新增可选`identification_checks`与`identified_model_closed_loop`，需`GIMBAL_BUILD_IDENTIFICATION=ON`和NumPy。21项检查覆盖合成参数恢复、坏数据/激励不足、float32导出、多轴头文件，以及力矩比例/重力零偏无法由残差识别的反例。8个候选闭环案例按两轴×两电机×两种负载条件展开，另有8个手定粗模型对照；实际C核心和电机组包后由独立字节解码驱动合成对象。脚本中的C程序按`-O2`独立编译，未继承ASan配置；该新增结果仅按Release主机验证报告。[辨识方法与复现](IDENTIFICATION.md)、[闭环记录](../sim/results/identification/summary.json)。

Arm 编译发现 DM 示例的枚举有符号性差异：原范围判断与零比较在该 ABI 下触发 `-Werror=type-limits`。已改为单次无符号范围检查，保留负枚举及超过上界的拒绝行为；补充非法枚举回归，Debug／Release／ASan+UBSan 的受影响接入测试复验通过。

ASan/UBSan 检查的主机环境受 ptrace 限制，运行测试时使用 `ASAN_OPTIONS=detect_leaks=0` 关闭 LeakSanitizer；没有关闭地址或未定义行为检测。算法和电机库不使用堆内存；主机仿真使用标准文件/内存函数。

可复现命令：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure

cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure

cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DGIMBAL_SANITIZE=ON
cmake --build build-sanitize --parallel
ASAN_OPTIONS=detect_leaks=0 ctest --test-dir build-sanitize --output-on-failure

# arm-none-eabi-gcc / ar / ranlib 必须在配置和构建两个步骤的 PATH 中。
cmake -S . -B build-arm -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DGIMBAL_BUILD_TESTS=OFF -DGIMBAL_BUILD_SIM=OFF
cmake --build build-arm --parallel
```

## 仿真所显示的取舍

新增开源模型验证采用独立的参数与实验矩阵。其 576 组合中，1 s 后窗口最坏位置 RMS 为 0.003161458 rad，峰值误差为 0.005400267 rad；最大实际报文力矩为 0.673928284 N·m，小于选择的 1 N·m 应用限制。具体来源、前馈假设、模型修正、门槛及局限见 [开源模型报告](OPEN_MODEL_VALIDATION.md)。这组结果不是电机实测精度。

新增 [Pitch 专项报告](PITCH_VALIDATION.md)采用更大参考区间、分方向统计、重力误差、附载与静态底座倾斜。20 个验收场景中最坏方向 RMS 为 0.009262987 rad、峰值为 0.019725037 rad，给定力矩峰值为 0.688039124 N·m。4 个错误坐标诊断虽也未超出既定门槛，Standard3 下行 RMS 仍比正确补偿大约 7.2 倍。实际角度越过参考端点最多约 0.351°，仍留有约 4.232° 硬边界余量；这是声明模型中的轨迹检查，不是软限位控制或三轴联动认证。

以下保持原始合成算例的参数口径，与新模型矩阵分开报告。

以 1 kHz 的“2 倍惯量、负载扰动、1 ms 指令延迟、3 ms 执行器动态、测量纹波”合成工况为例：

| 指标 | 线性滑模 | 正则化终端滑模 |
|---|---:|---:|
| 1 s 后位置 RMS | 0.002043 rad | 0.000929 rad |
| 全时段指令总变化量 / 时长 | 1.388 N·m/s | 2.022 N·m/s |

该配置的终端项降低了误差，同时提高输出变化量。它额外增加了反馈增益，两种方案没有匹配局部带宽，不能把误差变化全归因于终端结构。输出总变化量也不是抖振频谱。默认线性方案仍是初始移植路径。

详细条件、判据与未模拟因素见 [仿真假设](../sim/assumptions.md)，可查看 [指标 CSV](../sim/results/metrics.csv) 与 [曲线](../sim/results/closed_loop_comparison.png)。

## 尚未验证

已完成本公共库四个静态库的 Arm 交叉编译，但没有针对完整官方例程完成最终固件链接、烧录、CAN 台架测试或 MCU 最坏执行时间测量。静态库编译不会解析并验证整车工程的最终符号、调度、总线或机械行为。官方框架接入是经源码核对的接入设计，尚非实机兼容性认证。

需要在实际云台上完成惯量/阻尼/重力与有效力矩标定、坐标和方向核验、固件模式配置、总线 ID/负载检查，以及温升、结构振动和模式切换验证。主机测试不支持宣称“已达到某跟踪精度”“可直接上车无需整定”或“优于原队实战控制器”。
