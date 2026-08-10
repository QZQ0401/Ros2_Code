# vendor_robot_bringup

## 1. 功能包作用

本包是 G4 驱动的统一启动入口。`robot.launch.py` 根据 `mode` 选择真实 RTDE 硬件或 `mock_components/GenericSystem`，动态生成 ros2_control 参数，启动控制器、SDK 管理和诊断节点。

本包自身不实现消息回调或网络通信，主要负责启动顺序、参数传递、namespace/prefix 和控制器生命周期。

## 2. 目录结构

```text
vendor_robot_bringup/
├── config/diagnostics.yaml
├── launch/robot.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

Humble 中 controller `type` 属于 controller manager，而 `joints`、command/state interface 和约束属于具体 controller 节点。启动文件运行时生成临时 YAML，避免 prefix 与静态参数不一致。

## 3. 启动的节点与接口

| 节点/控制器 | 模式 | 主要接口 |
|---|---|---|
| `robot_state_publisher` | real/fake | `/tf`、`/tf_static`，订阅 `/joint_states` |
| `ros2_control_node` | real/fake | `/controller_manager/*` 标准管理服务 |
| `joint_state_broadcaster` | real/fake | `/joint_states` |
| `arm_controller` | real/fake | `/arm_controller/follow_joint_trajectory` |
| `driver_status_broadcaster` | real/fake | `/driver_status_broadcaster/driver_status` |
| `speed_scaling_broadcaster` | real/fake | `/speed_scaling_broadcaster/speed_scaling_factor` |
| `io_and_status_controller` | real/fake | `~/set_io`、`~/set_speed_scaling`、`~/io_states` |
| `sdk_manager_node` | real | `/robot_mode`、`/safety_mode` 和管理 services |
| `controller_stopper_node` | real | 订阅模式 topic，调用 `/controller_manager/switch_controller` |
| `diagnostics_bridge_node` | real/fake | `/diagnostics` |

控制器私有名 `~` 默认展开为：

```text
/io_and_status_controller/set_io
/io_and_status_controller/set_speed_scaling
/io_and_status_controller/io_states
/driver_status_broadcaster/driver_status
/speed_scaling_broadcaster/speed_scaling_factor
```

## 4. Launch 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `mode` | `real` | `real` 或 `fake` |
| `robot_ip` | `192.168.6.6` | 控制器 IP |
| `network_interface` | 空 | RTDE 绑定网卡 |
| `rtde_frequency` | `250` | RTDE 与 controller manager 更新频率 |
| `rtde_protocol_version` | `3` | RTDE 协议版本 |
| `initial_state_timeout_ms` | `10000` | 等待 RTDE 首帧超时 |
| `sdk_port` | `2323` | 真实控制柜 SDK 端口；虚拟控制柜通常为 2325 |
| `sdk_password` | 空 | SDK 密码 |
| `start_arm_controller` | `true` | 是否直接激活运动控制器 |
| `enable_controller_stopper` | `true` | 是否按机器人/安全状态自动停控 |
| `trajectory_tolerance` | `0.35` | JTC path tolerance，rad |
| `goal_tolerance` | `0.05` | JTC goal tolerance，rad |
| `goal_time` | `2.0` | 到位附加时间，s |
| `stopped_velocity_tolerance` | `0.05` | 终点速度阈值，rad/s |
| `namespace` | 空 | ROS namespace |
| `prefix` | 空 | 关节和 link 前缀 |

## 5. 使用方法

```bash
# Fake Hardware
ros2 launch vendor_robot_bringup robot.launch.py mode:=fake

# 真机
ros2 launch vendor_robot_bringup robot.launch.py \
  mode:=real robot_ip:=192.168.6.16 network_interface:=ens33 \
  rtde_frequency:=250 sdk_port:=2323
# robot_ip为连接robot的真实ip，network_interface为连接robot的网卡
# 只读状态和管理接口，不占用关节运动资源
ros2 launch vendor_robot_bringup robot.launch.py \
  mode:=real robot_ip:=192.168.6.16 network_interface:=ens33 \
  start_arm_controller:=false

# 多机
ros2 launch vendor_robot_bringup robot.launch.py \
  mode:=real namespace:=left prefix:=left_ robot_ip:=192.168.6.16
```

控制器与硬件检查：

```bash
ros2 control list_hardware_components --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 control list_controllers --controller-manager /controller_manager

ros2 control switch_controllers --activate arm_controller \
  --controller-manager /controller_manager
ros2 control switch_controllers --deactivate arm_controller \
  --controller-manager /controller_manager
```

轨迹 action：

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.05, -0.10, 0.10, 0.0, 0.05, 0.0], time_from_start: {sec: 5}}]}}"
```

目标必须替换为当前姿态附近、经过现场安全检查的位置。

## 6. 控制链路

```text
robot.launch.py
  ├─ robot_state_publisher
  ├─ controller_manager
  │   ├─ joint_state_broadcaster
  │   ├─ arm_controller
  │   ├─ IOAndStatusController
  │   └─ Driver/Speed broadcasters
  ├─ sdk_manager_node
  ├─ controller_stopper_node
  └─ diagnostics_bridge_node
```

`arm_controller` 激活后自动 claim 六个 position command interface。硬件为 `unconfigured` 时必须先排查 RTDE 首帧、参数或接口校验错误，不能手动 claim。


