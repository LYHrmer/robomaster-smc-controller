# 测试记录空白模板

[完整说明与字段来源](../../TEST_DATA.md)

这些 CSV 仅有表头，是拟定的保存格式。旧固件不会自动输出这些表，也不是离线辨识工具的九列输入。没有采集的字段留在原始记录中标注缺失，不用零伪装为测量值；生成辨识输入时另按工具规则处理无效行和断窗。

| 文件 | 谁提供数据 |
|---|---|
| [usb_existing.template.csv](usb_existing.template.csv) | 现有 USB 包加 PC 接收时间、PC 包序号、CRC 检查；无需改 MCU 包格式 |
| [mcu_control_existing.template.csv](mcu_control_existing.template.csv) | 旧 MCU 已有目标、姿态、陀螺、输出、状态；需新增快照与导出 |
| [can_rx_raw.template.csv](can_rx_raw.template.csv) | GM6020 / DM4310 通用原帧；需新增 CAN 接收记录点 |
| [motor_rx.template.csv](motor_rx.template.csv) | 双 GM6020 原帧及解析值；需新增记录点，不适用于 DM4310 解码字段 |
| [can_tx.template.csv](can_tx.template.csv) | GM6020 四槽命令的实际软件提交值；需在发送分支新增记录点 |
| [run.template.json](run.template.json) | 人工填写的实验、配置、采集能力说明；不是工具 `--metadata` 文件 |

`run.template.json` 的电机配置以已核对的双 GM6020 旧工程为起点。使用 DM4310 时，替换 `motors` 中的型号、ID、模式及反馈单位，去掉 GM 四槽/电流计数字段，并按 [4310 说明](../../DM4310_IDENTIFICATION.md#2-用实际电机配置建立-mit-力矩通路)补充实际量程和驱动参数；不要保留不适用的默认值。

约定：

- `host_rx_time_s` 是 PC 单调接收时钟的秒值；`host_packet_index` 是 PC 本地编号，不能检出 MCU 未发出或途中完全丢失的包。CRC 失败包保留原字节，解码字段不可信。
- `*_tick_ms` 是拟在记录位置读取的 MCU `HAL_GetTick()`；32 位毫秒计数会回绕。`*_seq` 是拟新增的日志序号。原协议没有它们，它们也不是电机内部采样时刻。每次重启分开记录；同一毫秒允许多条原始事件。
- `bus` 如 `CAN1`；`feedback_id/std_id` 用 `0x206` 形式；`dlc` 为实际长度；`data_hex` 如 `0102030405060708`，按接收顺序保留字节。非法长度保留原帧，不解析成有效电机数值。
- GM6020 `encoder_raw` 为无符号编码器计数；`speed_raw_rpm` 是有符号协议转速；`current_raw` 是有符号反馈计数；`temperature_raw` 为反馈温度字节。`yaw_control_raw/pitch_control_raw` 和 `slot*_raw` 均是有符号命令整数。
- `yaw_target_deg/pitch_target_deg` 是控制类接受的角度目标；`yaw_total_deg` 为连续 IMU Yaw；`pitch_deg/roll_deg` 为 IMU 欧拉角；三轴 gyro 是机体系 rad/s。模板未包含 SMC 参考生成结果，SMC 测试应按主说明追加对应诊断。
- `aiming_flag` 保存旧 `aimingFlag`，`stop_flag` 保存旧停止标志的值；它们不能单独证明进入了哪个控制分支。实际手瞄/自瞄分支与切换另存事件记录。
- JSON 中 `null`、空列表和占位名称都必须按本次实验填写。`false` 表示模板默认未确认实现/标定，不能因为期望某功能存在就改为 `true`。

将各流的记录位置、时钟、序号生成规则、字段来源写入实验目录的解码说明。导出丢弃计数、新鲜度、CAN 发送状态等额外诊断仅在实际实现记录后填写。
