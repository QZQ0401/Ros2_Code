# vendor_robot_description

## 1. 功能包作用

本包提供 G4 的 URDF/Xacro、网格、初始关节位置和统一 ros2_control 描述。Real、Fake 和 Gazebo 使用同一模型入口：

```text
urdf/g4.urdf.xacro
```

运行时接口由使用该模型的 `robot_state_publisher`、controller manager、MoveIt 或 Gazebo 创建。

## 2. 目录结构

```text
vendor_robot_description/
├── urdf/
│   ├── g4.urdf.xacro
│   ├── g4.model.xacro
│   └── g4.ros2_control.xacro
├── meshes/
│   └── *.STL
├── config/initial_positions.yaml
├── CMakeLists.txt
└── package.xml
```

文件作用：

| 文件 | 作用 |
|---|---|
| `g4.model.xacro` | link、joint、惯量、visual、collision |
| `g4.ros2_control.xacro` | Real/Fake/Simulation 硬件插件和 command/state interface |
| `g4.urdf.xacro` | 统一参数入口；仿真时加载 gazebo_ros2_control |
| `initial_positions.yaml` | Fake/初始 position state |

## 3. Xacro 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `prefix` | 空 | link、joint、system 名前缀 |
| `use_fake_hardware` | false | 使用 GenericSystem |
| `use_simulation` | false | 使用 GazeboSystem |
| `robot_ip` | `192.168.6.6` | RTDE 地址 |
| `network_interface` | 空 | RTDE 网卡 |
| `rtde_frequency` | 250 | 更新频率 |
| `rtde_protocol_version` | 3 | RTDE 协议 |
| `initial_state_timeout_ms` | 10000 | 首帧超时 |
| `initial_positions_file` | 包内 YAML | 初始关节位置 |

## 4. 声明的 ros2_control 接口

六关节：

```text
joint1 ... joint6
  command_interface: position
  state_interface: position, velocity, acceleration
```

其他接口：

```text
speed_scaling/speed_scaling_factor
gpio/standard_digital_input_0 ... 7
gpio/standard_digital_output_0 ... 7
gpio/configurable_digital_input_0 ... 7
gpio/configurable_digital_output_0 ... 7
gpio/tool_digital_input_0 ... 9
gpio/tool_digital_output_0 ... 9
driver/<14 个诊断 state interface>
```

每个数字输出还声明 `<output>_write_sequence` command interface。

## 5. 使用示例

展开真机模型：

```bash
xacro $(ros2 pkg prefix vendor_robot_description)/share/vendor_robot_description/urdf/g4.urdf.xacro \
  use_fake_hardware:=false use_simulation:=false \
  robot_ip:=192.168.6.16 network_interface:=ens33 \
  > /tmp/g4_real.urdf

check_urdf /tmp/g4_real.urdf
```

展开 Fake：

```bash
xacro $(ros2 pkg prefix vendor_robot_description)/share/vendor_robot_description/urdf/g4.urdf.xacro \
  use_fake_hardware:=true prefix:=test_ \
  > /tmp/g4_fake.urdf
```

运行时检查：

```bash
ros2 param get /robot_state_publisher robot_description
ros2 control list_hardware_interfaces
ros2 topic echo /tf
```

## 6. 信息流

```text
URDF/Xacro
  ├─ robot_state_publisher → /tf, /tf_static
  ├─ controller_manager → hardware plugin + interfaces
  ├─ MoveIt → RobotModel / collision model
  └─ Gazebo → model + gazebo_ros2_control
```

