# vendor_robot_hardware

## 1. 功能包作用

本包实现 ros2_control `SystemInterface` 插件 `vendor_robot_hardware/RtdeSystem`，负责硬件生命周期、资源接口、RTDE 快照交换、命令整形和安全停止。

它不创建 ROS node、service、topic、action 或额外 executor；ROS 接口由 controller manager 中的控制器提供。

## 2. 目录结构

```text
vendor_robot_hardware/
├── include/vendor_robot_hardware/rtde_system.hpp
├── src/rtde_system.cpp
├── vendor_robot_hardware.xml
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 3. 生命周期

| 回调 | 处理 |
|---|---|
| `on_init()` | 校验 6 关节、GPIO、driver interface 和参数 |
| `on_configure()` | 启动 RTDE client，等待首帧 |
| `on_activate()` | 实测位置同步到 command，清空旧事件 |
| `prepare/perform_command_mode_switch()` | 一次性切换六关节并启停 servo |
| `read()` | 更新 state interface |
| `write()` | 整形关节命令、转交 IO/速度 |
| `on_deactivate()` | 关闭运动透传、保留连接 |
| `on_cleanup()` / `on_error()` | 停止并释放 client |

`G4System` 为 `unconfigured` 时接口均为 `unavailable`；configure 成功后才可被 controller claim。

## 4. 导出接口

每个 `joint1`～`joint6`：

```text
command: position
state:   position, velocity, acceleration
```

Speed scaling：

```text
speed_scaling/speed_scaling_factor  command + state
```

GPIO：

- 标准数字输入/输出 0～7；
- 可配置数字输入/输出 0～7；
- 工具数字输入/输出 0～9；
- 每个输出还有 `<name>_write_sequence` command。

Driver state：

```text
connection_state, state_age_ms, overrun_count, reconnect_count,
control_cycle_count, last_cycle_ms, max_cycle_ms, command_fresh,
last_rtde_error, read_error_count, write_error_count,
safety_stop_latched, rejected_command_count, servo_skipped
```

## 5. 关节命令处理

`write()` 对 JTC 命令执行有限值检查、加速度限幅、速度限幅和单周期位移限幅，再把整形后的 position 交给 RTDE client。

| 参数 | 默认值 |
|---|---:|
| `max_velocity_rad_s` | 6.5 |
| `max_acceleration_rad_s2` | 40 |
| `max_cycle_delta_rad` | 0.05 |
| `max_position_error_rad` | 1.0 |
| `position_error_is_fatal` | false |
| `state_watchdog_ms` | 200 |
| `max_stale_state_cycles` | 10 |

普通超限 saturation 并告警；NaN/Inf 才锁存运动故障。

## 6. 使用示例

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control set_hardware_component_state G4System inactive

ros2 control switch_controllers --activate arm_controller
ros2 control list_hardware_interfaces
```

激活后应看到：

```text
joint1/position [available] [claimed]
...
joint6/position [available] [claimed]
```

不能手动 claim；claim 由 controller lifecycle 完成。故障后可停用再激活以从当前实测位置重新同步：

```bash
ros2 control switch_controllers --deactivate arm_controller
ros2 control switch_controllers --activate arm_controller
```

## 7. 注意事项

- 硬件层在 JTC 时间参数化后再次限制速度和加速度，可能导致 setpoint 落后并触发 goal tolerance；

