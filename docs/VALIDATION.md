# 验证记录

更新日期：2026-09-10。

[文档导航](README.md) · [在线辨识](ONLINE_IDENTIFICATION.md) · [工程结构](PROJECT_STRUCTURE.md)

主机环境：Linux、GCC 11.4.0、C99；本轮新增在线模块使用 GNU Arm 13.2.1、`-O2`、Cortex-M4F hard-float 完成交叉编译。以下区分主机执行、Arm 静态库编译与尚未进行的硬件测量。

默认完整环境包含 13 项 CTest；开启 `GIMBAL_BUILD_IDENTIFICATION=ON` 后包含 15 项。该选项仅增加电脑端 Python/NumPy 检查；`gimbal_rls` 与 `gimbal_identification` 是始终参与库构建的 C99 模块，未开启该选项也会编译。关闭示例、仿真或未安装 Python 时，测试入口数量会相应减少。

## 已执行

| 检查 | 结果 |
|---|---|
| Debug，`-Wall -Wextra -Wpedantic -Werror`，离线辨识 ON | 编译通过，15/15 CTest 通过 |
| Release，同样的警告检查，离线辨识 ON | 编译通过，15/15 CTest 通过；测试目标显式保留断言 |
| Debug + AddressSanitizer/UBSan，默认测试入口 | 编译通过，13/13 CTest 通过 |
| 可选离线辨识 | 两个入口内含21项Python检查及8个候选C闭环案例；包含于上述15项 |
| Cortex-M4F hard-float 交叉编译 | `gimbal_smc`、`gimbal_motor`、`gimbal_pitch`、`gimbal_example`、`gimbal_rls`、`gimbal_identification` 六个静态库通过 |
| 新增在线模块的编译选项负路径 | `gimbal_rls.c` 与 `gimbal_identification.c` 分别使用 `-ffast-math`、`-ffinite-math-only` 的4次检查均拒绝编译，防止有限值检查被这些选项破坏 |
| 在线辨识接口与数据窗口 | 通用RLS与流式积分转换的独立C测试通过；包含模型拟合、低激励、数值保护、断窗、时间回绕及构形隔离 |
| RLS长序列数值回归 | 120000次观测、113171次接受更新；2400次失能及4429次低激励均逐位保持参数/P，25段初始或恢复后评分通过 |
| 在线辨识闭环 | 4个显式采纳候选案例及4个保持初值对照通过各路径检查；实际C控制器及两种电机组包/独立解码参与 |
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

当前默认完整环境的 13 个入口如下：

| 类型 | CTest 名称 |
|---|---|
| C接口与数值回归（7项） | `smc`、`motor`、`example`、`pitch`、`rls`、`identification_online`、`rls_endurance` |
| C合成验证程序（5项） | `simulation`、`open_model_validation`、`pitch_validation`、`pitch_validator_negative`、`online_identification_closed_loop` |
| 模型来源复算（1项） | `open_model_provenance`，依赖Python 3.9+；本轮环境已安装 |

2026-09-10新增可选`identification_checks`与`identified_model_closed_loop`，需`GIMBAL_BUILD_IDENTIFICATION=ON`和NumPy。21项检查覆盖合成参数恢复、坏数据/激励不足、float32导出、多轴头文件，以及力矩比例/重力零偏无法由残差识别的反例。8个候选闭环案例按两轴×两电机×两种负载条件展开，另有8个手定粗模型对照；实际C核心和电机组包后由独立字节解码驱动合成对象。脚本中的C程序按`-O2`独立编译，未继承ASan配置；该新增结果仅按Release主机验证报告。[辨识方法与复现](IDENTIFICATION.md)、[闭环记录](../sim/results/identification/summary.json)。

早期 Arm 编译发现 DM 示例的枚举有符号性差异：原范围判断与零比较在该 ABI 下触发 `-Werror=type-limits`。已改为单次无符号范围检查，保留负枚举及超过上界的拒绝行为；补充非法枚举回归，Debug／Release／ASan+UBSan 的受影响接入测试复验通过。当前六库交叉编译继续覆盖这一路径，新增两个库的外部数学依赖为单精度函数，未发现 double 软浮点辅助运算依赖。

ASan/UBSan 检查的主机环境受 ptrace 限制，运行测试时使用 `ASAN_OPTIONS=detect_leaks=0` 关闭 LeakSanitizer；没有关闭地址或未定义行为检测。算法和电机库不使用堆内存；主机仿真使用标准文件/内存函数。

GitHub Actions 增加独立的 [Arm 静态库编译任务](../.github/workflows/c-tests.yml)：安装 Arm C 编译器和 newlib，记录编译器版本，以 `-O2 -fno-fast-math` 构建六个库。它用于发现目标 ABI、编译器及数学库相关回归，不运行 Cortex-M 指令，也不替代完整固件链接或执行时间测量。

可复现命令：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DGIMBAL_BUILD_IDENTIFICATION=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure

cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DGIMBAL_BUILD_IDENTIFICATION=ON
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure

cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DGIMBAL_SANITIZE=ON
cmake --build build-sanitize --parallel
ASAN_OPTIONS=detect_leaks=0 ctest --test-dir build-sanitize --output-on-failure

# arm-none-eabi-gcc / ar / ranlib 必须在配置和构建两个步骤的 PATH 中。
cmake -S . -B build-arm -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DGIMBAL_BUILD_TESTS=OFF -DGIMBAL_BUILD_SIM=OFF \
  -DCMAKE_C_FLAGS_RELEASE="-O2 -DNDEBUG -fno-fast-math"
cmake --build build-arm --parallel
```

前两个命令开启了可选离线检查，需要先安装 [requirements-identification.txt](../requirements-identification.txt)；只运行默认 13 项时省略该选项。

## 在线辨识的目标内存与闭环结果

下表来自上述 GNU Arm 13.2.1、Cortex-M4F hard-float、`-O2` 目标构建：

| 对象或函数 | 本次目标构建结果 | 范围 |
|---|---:|---|
| `sizeof(gimbal_rls_t)` | 1028 B | 一个通用RLS实例的状态 |
| `sizeof(gimbal_identification_t)` | 1328 B | 一个完整云台辨识实例，**已包含**其RLS状态 |
| `gimbal_rls_update()` 的 `.su` 记录 | 800 B | 单函数自身静态栈帧 |
| `gimbal_identification_update()` 的 `.su` 记录 | 136 B | 单函数自身静态栈帧 |

实例状态适合静态分配。这些数字不包含应用快照、输出和队列等缓冲；单函数栈帧也不是完整调用链栈峰值，不能直接当作 FreeRTOS 任务栈大小。数学函数、下级调用、中断嵌套及上下文保存仍需纳入目标固件的栈分析和实测。当前没有 MCU 最坏执行时间数据。

[在线闭环指标](../sim/results/online_identification/metrics.csv)及[汇总](../sim/results/online_identification/summary.json)覆盖 Yaw/Pitch、DM4310/GM6020 和显式参数采纳对照。采纳后的 Yaw RMS 为 0.0006061–0.0006116 rad，Pitch 为 0.0005475–0.0005513 rad；非零模型参数的最大相对误差为 0.2343%。

这些结果依赖已知的模型结构、固定摩擦速度尺度和经过标定的**合成**力矩反馈，不能解释为未知机构上的自动辨识精度。参数采纳由仿真程序明确执行；库本身只输出候选。RLS的P0、归一化与激励门槛仍需选择，有限记录内的边界投影可能影响后续拟合。方法、接口及不能由残差排除的错误见 [在线辨识](ONLINE_IDENTIFICATION.md)。

[RLS长序列回归](../tests/test_rls_endurance.c)另用120000次合成观测检查慢变参数、有限噪声、失能与低激励后的恢复。25个评分段中的最大归一化RMSE为0.01118、峰值为0.01646，均低于预设0.035/0.07门槛。该测试检验递推数值状态与数据门控，既不是机械闭环仿真，也不是 STM32 执行性能测量。

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

已完成本公共库六个静态库的 Arm 交叉编译，但没有针对完整官方例程完成最终固件链接、烧录、CAN 台架测试或 MCU 最坏执行时间测量。静态库编译不会解析并验证整车工程的最终符号、调度、总线或机械行为。官方框架接入是经源码核对的接入设计，尚非实机兼容性认证。

需要在实际云台上完成惯量/阻尼/重力与有效力矩标定、坐标和方向核验、固件模式配置、总线 ID/负载检查，以及温升、结构振动和模式切换验证。主机测试不支持宣称“已达到某跟踪精度”“可直接上车无需整定”或“优于原队实战控制器”。
