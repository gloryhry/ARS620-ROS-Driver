# ars620_driver

这是一个 ROS1 Noetic 驱动包，用于通过随包提供的厂商 USBCAN-FD SDK 读取 ARS620 雷达的公开 CAN-FD 输出。

驱动会动态加载 `libcontrolcanfd.so`，接收 CAN-FD 帧，按照 ARS620 DBC V1.7 解码公开输出报文，并将 RDI 点云簇和 OD 目标分别发布为结构化 ROS 消息和 `sensor_msgs/PointCloud2`。本驱动不使用 SocketCAN，不需要 Linux `can0` 网络接口。

英文说明见 [README.md](README.md)。

## 支持的协议接口

当前驱动解析以下 ARS620 公开 CAN-FD 报文：

- `0x50` `ARS_CONFIG_STATE`
- `0x52` `ARS_SYS_PERFORM`
- `0x100..0x140` RDI header 和 cluster 数据帧
- `0x200..0x219` OD header 和 object 数据帧

DBC 中标注总线类型为 `CAN FD`，ARS620 输出帧格式为 `StandardCAN_FD`。解码路径会忽略扩展帧、RTR 帧和错误帧；标准数据帧如果 ID 不支持或长度不符合预期，也会被忽略。打开全量保存功能时，这些被解码路径过滤掉的原始帧也会在过滤前保存。

## 硬件连接和 CAN-FD 参数

根据线束资料，读取公开输出应连接 Public CAN：

- 公 CAN 低：`Public CAN_L`，Pin 9
- 公 CAN 高：`Public CAN_H`，Pin 10
- Pin 9 和 Pin 10 需要使用双绞线

默认 USBCAN-FD 参数：

- `device_type`: `41`，对应 `USBCANFD_200U`
- `device_index`: `0`
- `channel_index`: `0`
- `abit_baud`: `500000`，仲裁域/nominal 波特率，500 kbit/s
- `dbit_baud`: `2000000`，数据域/data phase 波特率，2 Mbit/s
- `canfd_standard`: `0`
- `termination_enable`: `false`

只有在台架缺少终端电阻、需要使用适配器内置终端时，才建议设置 `termination_enable:=true`。如果雷达接在已有终端的整车或台架 CAN 总线上，保持 `false`。

## SDK 库文件

节点通过 ROS 参数 `library_path` 动态加载厂商库。x86_64 默认路径为：

```text
$(find ars620_driver)/lib/libcontrolcanfd/x86_64/libcontrolcanfd.so
```

随包 SDK 位于：

```text
src/ars620_driver/lib/libcontrolcanfd/
```

当前包含 `x86_64`、`aarch64`、`aarch32`、`riscv64` 等目录。部署到其他架构或使用外部安装的 SDK 时，可以覆盖 `library_path`。

## 编译

在 catkin 工作空间根目录执行：

```bash
cd /home/glory/Code/ARS620
catkin_make
source devel/setup.zsh
```

## 启动

当前 ARS620 CAN-FD 参数建议按以下方式启动：

```bash
roslaunch ars620_driver ars620_driver.launch \
  channel_index:=0 \
  abit_baud:=500000 \
  dbit_baud:=2000000
```

如果线束接在双通道 USBCAN-FD 的另一路，使用：

```bash
roslaunch ars620_driver ars620_driver.launch channel_index:=1
```

如果需要先确认 SDK 是否收到原始 CAN-FD 帧，可以打开原始帧调试日志：

```bash
roslaunch ars620_driver ars620_driver.launch debug_raw_frames:=true
```

收到原始帧时，节点会每秒节流打印类似日志：

```text
received 12 raw CAN-FD frames: 0x100=1 0x101=1 ...
```

如果需要查看驱动处理链路的运行耗时，可以打开 timing 日志：

```bash
roslaunch ars620_driver ars620_driver.launch debug_timing:=true timing_log_period:=1.0
```

timing 日志只作为诊断 ROS 日志，不改变任何发布数据。`receive` 阶段包含厂商 SDK 接收调用及其等待/阻塞时间，因此它用于分析循环运行耗时，不代表雷达协议延迟。

如果需要保存厂商 SDK 返回的每一帧 CAN-FD 原始报文，打开全量保存：

```bash
roslaunch ars620_driver ars620_driver.launch \
  save_all_canfd_frames:=true \
  save_all_canfd_path:=/tmp/ars620_canfd_test
```

`save_all_canfd_path` 表示目录。节点会在目录下创建带启动时间戳的文件：

```text
ars620_canfd_YYYYmmdd_HHMMSS.asc
ars620_canfd_YYYYmmdd_HHMMSS.raw.csv
ars620_canfd_YYYYmmdd_HHMMSS.mf4
```

节点运行期间实时写入 ASC 和 raw CSV；正常 Ctrl-C/退出时，通过 `scripts/raw_canfd_csv_to_mf4.py` 将 raw CSV 转成 MF4。MF4 转换依赖可按以下方式安装：

```bash
python3 -m pip install asammdf
```

如果当前环境没有 `asammdf`，节点会在退出时打印明确错误，并保留 `.asc` 和 `.raw.csv`。生成的 MF4 是原始 CAN-FD 帧 MDF4 容器，包含 `timestamp`、`can_id`、标志位、长度和 `data_00..data_63` 等字段；它不是 DBC 解码后的信号通道文件。

如果打开 `debug_raw_frames` 后没有任何原始帧计数，应优先检查雷达供电、Public CAN 接线、USBCAN-FD 通道、终端电阻和 CAN-FD 波特率，不要先怀疑解码器。

## Launch 参数

- `library_path`: `libcontrolcanfd.so` 路径
- `device_type`: 厂商设备类型，默认 `41`
- `device_index`: 厂商设备序号，默认 `0`
- `channel_index`: CAN 通道号，默认 `0`
- `abit_baud`: 仲裁域波特率，默认 `500000`
- `dbit_baud`: 数据域波特率，默认 `2000000`
- `canfd_standard`: 厂商 CAN-FD 标准选择参数，默认 `0`
- `termination_enable`: 是否打开适配器终端电阻，默认 `false`
- `frame_id`: 点云和目标数组的 ROS 坐标系名，默认 `ars620`
- `stamp_policy`: 时间戳策略，`vendor_timestamp` 或 `ros_time`，默认 `vendor_timestamp`
- `publish_partial`: 是否在超时后发布不完整 RDI/OD 周期，默认 `false`
- `rdi_max_targets`: 每个 RDI 周期最多组包并发布的 targets 数，默认 `256`；设置为 `0` 表示不限制
- `partial_timeout`: 不完整周期超时时间，单位秒，默认 `0.1`
- `receive_wait_ms`: SDK 接收等待时间，单位毫秒，默认 `20`
- `debug_raw_frames`: 是否打印原始 CAN-FD ID 计数，默认 `false`
- `debug_timing`: 是否打印处理链路 timing 日志，默认 `false`
- `timing_log_period`: timing 日志统计窗口，单位秒，默认 `1.0`
- `save_all_canfd_frames`: 是否保存收到的全部 CAN-FD 帧，并在正常退出时转换 MF4，默认 `false`
- `save_all_canfd_path`: 全量帧保存目录，默认 `~/.ros/ars620_canfd_logs`

## ROS 话题

- `/ars620/rdi_points` (`sensor_msgs/PointCloud2`)
- `/ars620/od_points` (`sensor_msgs/PointCloud2`)
- `/ars620/config_state` (`ars620_driver/Ars620ConfigState`)
- `/ars620/system_status` (`ars620_driver/Ars620SystemStatus`)
- `/ars620/rdi_targets` (`ars620_driver/Ars620RdiTargetArray`)
- `/ars620/od_targets` (`ars620_driver/Ars620OdTargetArray`)

RDI 点云字段：

```text
x, y, z, range, azimuth, elevation, vrad_rel, rcs, snr, dyn_prop, quality, cluster_index
```

OD 点云字段：

```text
x, y, z, vx, vy, ax, ay, rcs, length, width, orientation, yaw_rate,
object_id, classification, dyn_prop, prob_of_exist, maintenance_state
```

## 周期组包逻辑

RDI 周期以 `0x100` 开始。DBC 定义的 RDI 数据帧覆盖 `0x101..0x140`，协议上可表示最多 512 个 cluster，每个数据帧包含 8 个 cluster。当前 ARS620 毫米波模式下，驱动默认每周期最多组包并发布 256 个 RDI targets；如果其他模式需要不同上限，可调整 `rdi_max_targets`，设置为 `0` 表示按 header 原始数量组包。`0x101` 对应 clusters `0..7`，`0x102` 对应 `8..15`，依此类推；数据帧允许乱序到达，驱动按 CAN ID 固定槽位组包。最后一个 RDI 数据帧中的尾部 padding targets 会被忽略。OD 周期以 `0x200` 开始，最多 50 个 object，每个数据帧包含 2 个 object。

默认情况下，驱动只在完整周期组包完成后发布 RDI/OD 输出。设置 `publish_partial:=true` 后，驱动会在 `partial_timeout` 秒超时后发布不完整周期。

完整模式下，RDI 周期超时后不会发布不完整点云，驱动会继续保留缓冲等待迟到数据帧补齐。类似 `missing RDI frames: 0x102 0x104` 的 warning 表示当前周期缺少对应的 `0x101..0x140` 数据帧，不等同于雷达 header 自身报错。

`stamp_policy` 默认是 `vendor_timestamp`，即优先使用 USBCAN-FD SDK 给出的接收时间戳。结构化目标数组消息中也会保留从 ARS620 header 报文解码出的雷达全局时间戳和本地时间戳。

## 验证

编译和单元测试：

```bash
catkin_make
catkin_make run_tests_ars620_driver
source devel/setup.zsh && roslaunch --dump-params src/ars620_driver/launch/ars620_driver.launch
```

硬件冒烟测试：

```bash
roslaunch ars620_driver ars620_driver.launch debug_raw_frames:=true
rostopic hz /ars620/rdi_points
rostopic hz /ars620/od_points
rostopic echo /ars620/config_state
rostopic echo /ars620/system_status
```

全量报文保存冒烟测试：

```bash
roslaunch ars620_driver ars620_driver.launch \
  save_all_canfd_frames:=true \
  save_all_canfd_path:=/tmp/ars620_canfd_test
```

节点运行时确认目录下出现 `.asc` 和 `.raw.csv`。Ctrl-C 正常退出后，确认生成 `.mf4`，或日志中明确提示缺少 `asammdf`。如果进程被 `SIGKILL` 杀死或异常断电，已 flush 的 ASC/raw CSV 通常会保留，但不保证生成 MF4。

如果原始帧日志中能看到 `0x100..0x140`，说明 RDI/cluster 输出已经进入驱动。如果只看到 `0x200..0x219`，说明当前雷达更像是 OD/object 输出模式。若完全没有原始帧，应先检查硬件链路和 CAN-FD 参数。

## 限制

本包只解析客户 DBC 和信号说明中公开的 CAN-FD 广播报文。UDS/DID 配置读写、雷达模式切换、安全解锁、配置掉电保存等功能没有实现，因为当前公开资料中没有提供对应 DID、服务流程和解锁方法。
