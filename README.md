# ars620_driver

ROS1 Noetic driver for the ARS620 radar public CAN-FD output using the vendor USBCAN-FD SDK shipped in this package.

This package reads CAN-FD frames through `libcontrolcanfd.so`, decodes the ARS620 DBC V1.7 public output, and publishes RDI cluster targets and OD object targets as both structured ROS messages and `sensor_msgs/PointCloud2` topics. It does not use SocketCAN and does not require a `can0` Linux network interface.

Chinese documentation is available in [README_CN.md](README_CN.md).

## Supported Interface

The driver decodes these ARS620 public CAN-FD frames:

- `0x50` `ARS_CONFIG_STATE`
- `0x52` `ARS_SYS_PERFORM`
- `0x100..0x140` RDI header and cluster target frames
- `0x200..0x219` OD header and object target frames

The DBC marks the bus as `CAN FD` and the ARS620 output frames as `StandardCAN_FD`. Extended, RTR, and error frames are dropped by the receiver wrapper. Standard data frames with unsupported IDs or unexpected lengths are ignored.

## Hardware And CAN-FD Settings

Use the radar public CAN pins from the wiring document:

- Public CAN low: `Public CAN_L`, Pin 9
- Public CAN high: `Public CAN_H`, Pin 10
- Pin 9 and Pin 10 should be a twisted pair

Default USBCAN-FD settings:

- `device_type`: `41` (`USBCANFD_200U`)
- `device_index`: `0`
- `channel_index`: `0`
- `abit_baud`: `500000` nominal/arbitration bitrate, 500 kbit/s
- `dbit_baud`: `2000000` data phase bitrate, 2 Mbit/s
- `canfd_standard`: `0`
- `termination_enable`: `false`

Set `termination_enable:=true` only when the test bench needs the adapter-side termination resistor. If the radar is connected to an already terminated vehicle or bench CAN bus, keep it `false`.

## SDK Library

The node dynamically loads the vendor library from the `library_path` ROS parameter. The default x86_64 path is:

```text
$(find ars620_driver)/lib/libcontrolcanfd/x86_64/libcontrolcanfd.so
```

Package-local SDK libraries are included under:

```text
src/ars620_driver/lib/libcontrolcanfd/
```

Available architecture folders include `x86_64`, `aarch64`, `aarch32`, and `riscv64`. Override `library_path` when deploying on another architecture or when using an externally installed SDK.

## Build

Build from the catkin workspace root:

```bash
cd /home/glory/Code/ARS620
catkin_make
source devel/setup.zsh
```

## Run

Typical launch command for the current ARS620 CAN-FD setup:

```bash
roslaunch ars620_driver ars620_driver.launch \
  channel_index:=0 \
  abit_baud:=500000 \
  dbit_baud:=2000000
```

If the harness is connected to the second channel of a dual-channel adapter, use:

```bash
roslaunch ars620_driver ars620_driver.launch channel_index:=1
```

To inspect raw received CAN-FD frame IDs before decoding, enable the throttled debug log:

```bash
roslaunch ars620_driver ars620_driver.launch debug_raw_frames:=true
```

When raw frames are received, the node prints lines like:

```text
received 12 raw CAN-FD frames: 0x100=1 0x101=1 ...
```

## Launch Parameters

- `library_path`: path to `libcontrolcanfd.so`
- `device_type`: vendor device type, default `41`
- `device_index`: vendor device index, default `0`
- `channel_index`: CAN channel index, default `0`
- `abit_baud`: arbitration bitrate, default `500000`
- `dbit_baud`: data phase bitrate, default `2000000`
- `canfd_standard`: vendor CAN-FD standard selector, default `0`
- `termination_enable`: adapter termination resistor enable, default `false`
- `frame_id`: ROS frame ID for point clouds and target arrays, default `ars620`
- `stamp_policy`: `vendor_timestamp` or `ros_time`, default `vendor_timestamp`
- `publish_partial`: publish incomplete RDI/OD cycles after timeout, default `false`
- `partial_timeout`: partial cycle timeout in seconds, default `0.1`
- `receive_wait_ms`: SDK receive wait time in milliseconds, default `20`
- `debug_raw_frames`: print throttled raw CAN-FD ID counts, default `false`

## Topics

- `/ars620/rdi_points` (`sensor_msgs/PointCloud2`)
- `/ars620/od_points` (`sensor_msgs/PointCloud2`)
- `/ars620/config_state` (`ars620_driver/Ars620ConfigState`)
- `/ars620/system_status` (`ars620_driver/Ars620SystemStatus`)
- `/ars620/rdi_targets` (`ars620_driver/Ars620RdiTargetArray`)
- `/ars620/od_targets` (`ars620_driver/Ars620OdTargetArray`)

RDI `PointCloud2` fields:

```text
x, y, z, range, azimuth, elevation, vrad_rel, rcs, snr, dyn_prop, quality, cluster_index
```

OD `PointCloud2` fields:

```text
x, y, z, vx, vy, ax, ay, rcs, length, width, orientation, yaw_rate,
object_id, classification, dyn_prop, prob_of_exist, maintenance_state
```

## Cycle Handling

RDI cycles start with `0x100` and can contain up to 512 clusters, with 8 clusters per data frame. OD cycles start with `0x200` and can contain up to 50 objects, with 2 objects per data frame.

By default, RDI and OD outputs are published only after a complete cycle has been assembled. Set `publish_partial:=true` to publish partial cycles after `partial_timeout` seconds.

`stamp_policy` defaults to `vendor_timestamp`, using the USBCAN-FD receive timestamp when available. The structured target array messages also preserve radar global and local timestamps decoded from the ARS620 header frames.

## Verification

Run the build and tests:

```bash
catkin_make
catkin_make run_tests_ars620_driver
source devel/setup.zsh && roslaunch --dump-params src/ars620_driver/launch/ars620_driver.launch
```

Hardware smoke test:

```bash
roslaunch ars620_driver ars620_driver.launch debug_raw_frames:=true
rostopic hz /ars620/rdi_points
rostopic hz /ars620/od_points
rostopic echo /ars620/config_state
rostopic echo /ars620/system_status
```

If `debug_raw_frames:=true` shows no raw frame counts, check radar power, Public CAN wiring, adapter channel, termination, and CAN-FD bitrate settings before debugging the decoder.

## Limitations

This package decodes the public CAN-FD broadcast frames described by the customer DBC and signal documents. UDS/DID configuration read/write, radar mode switching, security unlock, and persistent radar configuration are not implemented because the required supplier DID IDs and unlock procedure are not included in the public documentation.
