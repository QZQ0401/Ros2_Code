# vendor_robot_controllers

## 1. 功能包作用

本包包含三类 ros2_control controller 插件和三个普通 ROS 2 节点：

- `IOAndStatusController`：速度比例、数字 IO 写入与 IO 状态发布；
- `DriverStatusBroadcaster`：驱动健康状态；
- `SpeedScalingBroadcaster`：速度比例缓存状态；
- `sdk_manager_node`：低频 SDK 管理与 RobotMode；
- `controller_stopper_node`：根据模式和 watchdog 自动停/恢复运动 controller；
- `diagnostics_bridge_node`：转换为标准 `/diagnostics`。



## 2. 目录结构

```text
vendor_robot_controllers/
├── include/vendor_robot_controllers/
│   ├── io_and_status_controller.hpp
│   ├── driver_status_broadcaster.hpp
│   └── speed_scaling_broadcaster.hpp
├── src/
│   ├── io_and_status_controller.cpp
│   ├── driver_status_broadcaster.cpp
│   ├── speed_scaling_broadcaster.cpp
│   ├── sdk_manager_node.cpp
|   ├── controller_bootstrapper_node.cpp
│   ├── controller_stopper_node.cpp
│   └── diagnostics_bridge_node.cpp
├── vendor_robot_controllers.xml
├── CMakeLists.txt
└── package.xml
```

## 3. Topic

### `/io_and_status_controller/io_states`

- 类型：`vendor_robot_msgs/msg/IOStates`
- 发布者：`IOAndStatusController`
- 说明：数字输入和输出按 bit 编码。标准通道使用 bit 0～7，可配置通道使用 bit 8～15，工具通道使用 bit 16～25。

```bash
ros2 topic echo /io_and_status_controller/io_states
ros2 topic hz /io_and_status_controller/io_states
```

### `/driver_status_broadcaster/driver_status`

- 类型：`vendor_robot_msgs/msg/DriverStatus`
- 发布者：`DriverStatusBroadcaster`
- 说明：RTDE 连接、周期、读写错误、命令新鲜度、故障锁存和 `servo_skipped`。

```bash
ros2 topic echo /driver_status_broadcaster/driver_status
```

执行轨迹时重点检查：

```text
state_name: RUNNING
command_fresh: true
servo_skipped: false
consecutive_read_errors: 0
consecutive_write_errors: 0
```

### `/speed_scaling_broadcaster/speed_scaling_factor`

- 类型：`std_msgs/msg/Float64`
- 发布者：`SpeedScalingBroadcaster`
- 说明：当前 ROS command interface 缓存，不是控制柜独立回读。

```bash
ros2 topic echo /speed_scaling_broadcaster/speed_scaling_factor
```

### `/robot_mode`

- 类型：`vendor_robot_msgs/msg/RobotMode`
- 发布者：`sdk_manager_node`
- 说明：SDK `cr_get_robotMode()` 的结果。

```bash
ros2 topic echo /robot_mode
```

主要字段：

| 字段 | 含义 |
|---|---|
| `mode`、`name` | 原始模式值与可读名称 |
| `motion_allowed` | 代码推导的运动允许状态 (忽略)|
| `program_running` | 控制器程序运行状态 (忽略)|
| `servo_enabled` | 是否接受 ROS 运动命令(忽略) |

  - mode：为103之外的状态可以接受控制器命令实现robot运动

### `/safety_mode`

- 类型：`vendor_robot_msgs/msg/SafetyMode`
- 发布者：`sdk_manager_node`


```bash
ros2 topic echo /safety_mode
```

该 topic 不能替代控制柜安全系统。

### `/diagnostics`

- 类型：`diagnostic_msgs/msg/DiagnosticArray`
- 发布者：`diagnostics_bridge_node`
- 输入：订阅 `/driver_status_broadcaster/driver_status`

```bash
ros2 topic echo /diagnostics
```

## 4. Service

### 数字输出 `/io_and_status_controller/set_io`(RTDE设置IO暂未使用)

- 类型：`vendor_robot_msgs/srv/SetIO`
- 可用条件：`io_and_status_controller` 必须 active。

```bash
# 标准输出 0～7，domain=0
ros2 service call /io_and_status_controller/set_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 0, channel: 0, value: true}"

# 可配置输出 0～7，domain=1
ros2 service call /io_and_status_controller/set_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 1, channel: 3, value: false}"

# 工具输出 0～2，domain=2
ros2 service call /io_and_status_controller/set_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 2, channel: 9, value: true}"
```

### 数字输出 `/io_and_status_controller/set_io`(SDK设置IO)

- 类型：`vendor_robot_msgs/srv/SetIO`

```bash
# 标准输出 0～7，domain=0
ros2 service call /set_sdk_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 0, channel: 0, value: true}"

# 可配置输出 0～7，domain=1
ros2 service call /set_sdk_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 1, channel: 3, value: false}"

# 工具输出 0～2，domain=2
ros2 service call /set_sdk_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 2, channel: 0, value: true}"
```



### 速度比例 `/io_and_status_controller/set_speed_scaling`

- 类型：`vendor_robot_msgs/srv/SetSpeedScaling`
- 范围：`0.0～1.0`

```bash
ros2 service call /io_and_status_controller/set_speed_scaling \
  vendor_robot_msgs/srv/SetSpeedScaling \
  "{scaling: 0.10}"
```

### `/power_on`

```bash
ros2 service call /power_on std_srvs/srv/Trigger "{}"
```

### `/enable_robot`

```bash
ros2 service call /enable_robot std_srvs/srv/Trigger "{}"
```

### `/disable_robot`

```bash
ros2 service call /disable_robot std_srvs/srv/Trigger "{}"
```

### `/clear_error`

```bash
ros2 service call /clear_error std_srvs/srv/Trigger "{}"
```

### `/stop_program`

```bash
ros2 service call /stop_program std_srvs/srv/Trigger "{}"
```

调用前应先停用 `arm_controller`，避免 ROS 与控制器程序争夺运动权。

### `/set_tool_power`

- 类型：`std_srvs/srv/SetBool`

```bash
#std_srvs为ros2control提供的interface，查看：ros2 interface list
ros2 service call /set_tool_power std_srvs/srv/SetBool "{data: true}"
ros2 service call /set_tool_power std_srvs/srv/SetBool "{data: false}"
```

### `/set_payload`

- 类型：`vendor_robot_msgs/srv/SetPayload`
- 单位：质量 kg，重心 m；SDK 边界转换为 mm。

```bash
ros2 service call /set_payload vendor_robot_msgs/srv/SetPayload \
  "{name: 'gripper', mass: 1.20, center_of_gravity: {x: 0.0, y: 0.0, z: 0.08}}"
```

节点会按名称更新或新增 Payload，然后激活对应配置。

## 5. ControllerStopper

订阅：

```text
/robot_mode
/safety_mode
```

调用：

```text
/controller_manager/switch_controller
```

参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `enabled` | false；bringup 传 true | 是否启用 |
| `mode_timeout_ms` | 500 | 状态超时 |
| `startup_grace_ms` | 3000 | 启动宽限 |
| `treat_sdk_motion_as_external` | false | 是否把 SDK_MOVING 当外部运动 |
| `auto_restart` | true | 条件恢复后重新激活 |
| `stop_on_watchdog` | true | 状态不发布时是否停控 |
| `controllers` | `[arm_controller]` | 管理的控制器 |

点动、示教、反驱、控制器程序运行或安全状态不允许时会停用 JTC。SDK_MOVING 默认视为本 RTDE 驱动自身运动。


