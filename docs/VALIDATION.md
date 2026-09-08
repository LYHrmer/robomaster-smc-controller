# 验证记录

日期：2026-09-08。环境：Linux 主机，GCC 11.4.0，C99。以下结果来自本工程实际编译/执行，不包含 STM32 或电机实测。

## 已执行

| 检查 | 结果 |
|---|---|
| Debug，`-Wall -Wextra -Wpedantic -Werror` | 编译通过，6/6 CTest 通过 |
| Release，同样的警告检查 | 编译通过，6/6 CTest 通过；测试目标显式保留断言 |
| Debug + AddressSanitizer/UBSan | 编译通过，6/6 CTest 通过 |
| 控制核心接口单测 | 方向、过零制动、前馈、角度回绕、限幅/斜率/滤波、超时、NaN/Inf、错误周期及恢复通过 |
| 正则化终端项 | 导数与独立 double 中心差分对照通过 |
| 电机协议单测 | 独立字节向量、方向/传动、量化/限幅、非法配置、DLC/ID、DM 故障、GM 四槽组帧通过 |
| 接入单测 | 首次参考锚定、陈旧反馈、恢复、同组其他槽保留、DM ARMING 不发送 FD 通过 |
| 合成仿真 | 12 条闭环与 3 条目标差分诊断通过预设回归包络 |
| 开源模型参数复算 | 11 份固定版本原文件的 SHA256/Git blob 校验、2 模型/4 关节惯量和重力离线复算通过 |
| 开源模型闭环 | 576 PASS、0 FAIL、0 INFEASIBLE；两种电机实际组包后独立字节解码 |
| 开源验证器负路径 | 10 类无效输入拒绝；超能力、失稳合成样例分别返回不可行/失败，未运行指标为 NaN |
| 图件 | 从 CSV 重算指标，生成 3 组 PNG/SVG，检查图形排版 |

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
```

## 仿真所显示的取舍

新增开源模型验证采用独立的参数与实验矩阵。其 576 组合中，1 s 后窗口最坏位置 RMS 为 0.003161458 rad，峰值误差为 0.005400267 rad；最大实际报文力矩为 0.673928284 N·m，小于选择的 1 N·m 应用限制。具体来源、前馈假设、模型修正、门槛及局限见 [开源模型报告](OPEN_MODEL_VALIDATION.md)。这组结果不是电机实测精度。

以下保持原始合成算例的参数口径，与新模型矩阵分开报告。

以 1 kHz 的“2 倍惯量、负载扰动、1 ms 指令延迟、3 ms 执行器动态、测量纹波”合成工况为例：

| 指标 | 线性滑模 | 正则化终端滑模 |
|---|---:|---:|
| 1 s 后位置 RMS | 0.002043 rad | 0.000929 rad |
| 全时段指令总变化量 / 时长 | 1.388 N·m/s | 2.022 N·m/s |

该配置的终端项降低了误差，同时提高输出变化量。它额外增加了反馈增益，两种方案没有匹配局部带宽，不能把误差变化全归因于终端结构。输出总变化量也不是抖振频谱。默认线性方案仍是初始移植路径。

详细条件、判据与未模拟因素见 [仿真假设](../sim/assumptions.md)，可查看 [指标 CSV](../sim/results/metrics.csv) 与 [曲线](../sim/results/closed_loop_comparison.png)。

## 尚未验证

没有可用的 `arm-none-eabi-gcc` 和目标硬件连接，本轮没有交叉编译、完整官方固件链接、烧录、CAN 台架测试或 MCU 最坏执行时间测量。官方框架接入是经源码核对的接入设计，尚非实机兼容性认证。

需要在实际云台上完成惯量/阻尼/重力与有效力矩标定、坐标和方向核验、固件模式配置、总线 ID/负载检查，以及温升、结构振动和模式切换验证。主机测试不支持宣称“已达到某跟踪精度”“可直接上车无需整定”或“优于原队实战控制器”。
