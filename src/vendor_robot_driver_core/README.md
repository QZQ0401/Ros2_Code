# vendor_robot_driver_core

## 1. 功能包作用

本包是与 ROS 解耦的 RTDE 通信核心，封装连接、状态接收、关节透传、数字 IO、速度比例、watchdog、错误统计和退避重连。



## 2. 目录结构

```text
vendor_robot_driver_core/
├── include/vendor_robot_driver_core/
│   ├── rtde_client.hpp
│   ├── cdrtdeapi.h
│   └── cdrtdebasedefine.h
├── src/rtde_client.cpp
├── lib/libcdrtdeapi.so
├── lib/libcdrtdebase.so
├── test/test_rtde_client.cpp
├── CMakeLists.txt
└── package.xml
```

## 3. 核心 API

| 方法 | 作用 |
|---|---|
| `start()` / `stop()` | 启停唯一 RTDE worker |
| `enable_servo(bool)` | 允许或停止关节透传 |
| `submit_joint_position()` | 提交最新 6 关节 rad 目标 |
| `submit_speed_scaling()` | 提交 0～1 速度比例 |
| `submit_digital_output()` | 提交有界 IO 队列 |
| `joint_state()` / `io_state()` | 读取线程安全状态快照 |
| `statistics()` | 读取连接、周期、错误、命令新鲜度和 `servo_skipped` |
| `request_safe_stop()` | 请求 worker 执行 `servo_stop` |

状态机：

```text
DISCONNECTED → CONNECTING → SYNCHRONIZING → RUNNING
                                      ↘ DEGRADED → RECONNECTING
```

连续读/写错误达到阈值后请求停止并重连。重连时清除旧关节、IO 和速度命令。

## 4. 输入输出

RTDE 状态 recipe：

```text
actualJointPosition
actualJointVelocity
actualJointAcceleration
```

6 关节 degree、degree/s、degree/s² 转换成 rad、rad/s、rad/s²。非有限值记录为数据类型错误。

关节命令：

```text
position rad
  → submit_joint_position()
  → command_watchdog
  → rad 转 degree
  → cd_rtde_servoj()
```

`servo_enabled=false` 或命令不新鲜时不调用 `servoj`，`servo_skipped=true`，并请求一次 `servo_stop`。

数字 IO：

- 标准输出 0～7；
- 可配置输出 0～7；
- 工具输出 0～9；
- 队列最多 64 条；
- 每轮最多写一条；
- 默认约 10 Hz 回读。

## 5. 配置

| 参数 | 默认值 |
|---|---:|
| `frequency_hz` | 250 |
| `io_poll_rate_hz` | 10 |
| `protocol_version` | 3 |
| `servoj_cutoff_rad_s` | 50 |
| `stop_deceleration_deg_s2` | 30 |
| `command_watchdog` | 100 ms |
| `reconnect_initial` | 250 ms |
| `reconnect_max` | 5000 ms |
| `max_consecutive_errors` | 3 |

## 6. 使用与诊断

本包不能直接用 ROS CLI 调用。启动真机后通过：

```bash
ros2 topic echo /driver_status_broadcaster/driver_status
```

检查 `state_name`、`control_cycle_count`、`last_cycle_ms`、`overrun_count`、`command_fresh`、`servo_skipped`、连续读写错误和最后 RTDE 错误。


