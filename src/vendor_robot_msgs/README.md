# vendor_robot_msgs

## 1. 功能包作用

本包集中定义 G4 驱动的 ROS 2 Message、Service 和 Action 类型。功能包的主要作用为为机械臂在ROS2的框架下运行提供必要的消息文件。该功能包并没有可执行的使用命令，其主要作用为为其他功能包提供必须的消息文件。
单位约定：

- 关节角：rad；
- 长度：m；
- 质量：kg；
- 速度和加速度字段按对应接口说明使用。

## 2. 目录结构

```text
vendor_robot_msgs/
├── msg/
│   ├── IOStates.msg
│   ├── RobotMode.msg
│   ├── SafetyMode.msg
│   └── DriverStatus.msg
├── srv/
│   ├── SetIO.srv
│   ├── SetSpeedScaling.srv
│   └── SetPayload.srv
├── action/
│   ├── MoveJ.action
│   ├── MoveL.action
│   ├── MoveC.action
│   └── RecoverRobot.action
├── CMakeLists.txt
└── package.xml
```

查看定义：

```bash
ros2 interface show vendor_robot_msgs/msg/DriverStatus
ros2 interface show vendor_robot_msgs/srv/SetIO
ros2 interface show vendor_robot_msgs/action/MoveL
```

## 3. Message

### `IOStates`

| 字段 | 含义 |
|---|---|
| `digital_inputs` | bit 0～7 标准、8～15 可配置、16～25 工具输入 |
| `digital_outputs` | 同样的输出 bit 映射 |
| `analog_inputs/outputs` | 已定义，当前 controller 未填充 |

```bash
ros2 topic echo /io_and_status_controller/io_states
```

### `RobotMode`

```bash
ros2 topic echo /robot_mode
```

字段包含原始模式、名称、运动允许、程序运行和 servo 使能。

### `SafetyMode`

```bash
ros2 topic echo /safety_mode
```

字段包含保护停、急停、降速模式和运动允许。当前由 RobotMode 推导，`reduced_mode` 固定为 false。

### `DriverStatus`

```bash
ros2 topic echo /driver_status_broadcaster/driver_status
```

包含 RTDE 状态、命令新鲜度、连续错误、周期统计、故障锁存、拒绝计数和 `servo_skipped`。`sdk_connected`、`last_sdk_error` 当前没有发布数据源。

## 4. Service

### `SetIO`

```bash
#(暂未使用)
ros2 service call /io_and_status_controller/set_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 0, channel: 0, value: true}"


ros2 service call set_sdk_io \
  vendor_robot_msgs/srv/SetIO \
  "{domain: 0, channel: 0, value: true}"
```

domain：

| 值 | 含义 | 通道 | 状态 |
|---:|---|---|---:|
| 0 | Standard Digital Out | 0～7 |（暂停使用）|
| 1 | Configurable Digital Out | 0～7 |     |
| 2 | Tool Digital Out | 0～9 |（0-1有效）|

### `SetSpeedScaling`

```bash
ros2 service call /io_and_status_controller/set_speed_scaling \
  vendor_robot_msgs/srv/SetSpeedScaling "{scaling: 0.10}"
```

范围 `[0,1]`。

### `SetPayload`

```bash
ros2 service call /set_payload vendor_robot_msgs/srv/SetPayload \
  "{name: 'gripper', mass: 1.2, center_of_gravity: {x: 0.0, y: 0.0, z: 0.08}}"
```

## 5. Action

### `MoveL`：已有 server

```bash
ros2 action send_goal --feedback /move_l vendor_robot_msgs/action/MoveL \
  "{target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.0, z: 0.40}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity: 0.05, acceleration: 0.05, blend_radius: 0.0}"
```

必须先启动 `move_l_action_server`。

### `MoveJ`：仅定义，无 server

Goal 字段：

```yaml
joint_positions: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0]
velocity: 0.1
acceleration: 0.1
blend_radius: 0.0
```

当前执行：

```bash
ros2 action list -t | grep MoveJ
```

不会找到本工程提供的 server。若未来实现，可使用：

```bash
ros2 action send_goal --feedback /move_j vendor_robot_msgs/action/MoveJ \
  "{joint_positions: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0], velocity: 0.1, acceleration: 0.1, blend_radius: 0.0}"
```

### `MoveC`：仅定义，无 server

```yaml
via:    {header: {frame_id: base_link}, pose: {...}}
target: {header: {frame_id: base_link}, pose: {...}}
velocity: 0.1
acceleration: 0.1
blend_radius: 0.0
fixed_orientation: false
```

未来实现 server 后的请求形式：

```bash
ros2 action send_goal --feedback /move_c vendor_robot_msgs/action/MoveC \
  "{via: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.05, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: 0.00, z: 0.40}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity: 0.1, acceleration: 0.1, blend_radius: 0.0, fixed_orientation: false}"
```

### `RecoverRobot`：仅定义，无 server

```yaml
power_on: true
enable: true
```

当前应显式调用：

```bash
ros2 service call /power_on std_srvs/srv/Trigger "{}"
ros2 service call /enable_robot std_srvs/srv/Trigger "{}"
```

未来实现 action server 后的请求形式：

```bash
ros2 action send_goal --feedback /recover_robot \
  vendor_robot_msgs/action/RecoverRobot \
  "{power_on: true, enable: true}"
```

## 6. 实现状态

| 接口 | 状态 |
|---|---|
| IOStates、RobotMode、SafetyMode、DriverStatus | 已有发布者 |
| SetIO、SetSpeedScaling、SetPayload | 已有 service server |
| MoveL | 已有 action server，但需 MoveIt 启动 |
| MoveJ、MoveC、RecoverRobot | 仅定义 |

