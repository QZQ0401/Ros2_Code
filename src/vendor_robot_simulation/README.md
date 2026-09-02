# vendor_robot_simulation

## 1. 功能包作用

本包使用 Gazebo Classic 和 `gazebo_ros2_control/GazeboSystem` 启动多机型仿真，通过 `robot_type` 选择模型，控制器接口保持统一。

仿真验证 ROS 图、轨迹控制和模型，无法验证 RTDE、SDK、网络重连、控制柜安全状态或 `servoj` 实时行为。

## 2. 目录结构

```text
vendor_robot_simulation/
├── config/
│   ├── g4/controllers.yaml
│   ├── g6/controllers.yaml
│   └── <model>/controllers.yaml
├── launch/
│   ├── moveit_gazebo_in_simulation.launch.py
│   └── simulation.launch.py
├── urdf/gazebo.urdf.xacro       # 按 robot_type 选择模型的 Gazebo 通用入口
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 3. 启动

本节点包的启动无需管理员权限。

```bash
# 启动gazebo仿真
ros2 launch vendor_robot_simulation simulation.launch.py robot_type:=g9 
```
启动成功之后弹出界面：
![alt text](doc/gazebo1.png)

之后可以测试相关的运动功能:
```bash
# 示例
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.2, -0.3, 0.3, 0.0, 0.1, 0.0], time_from_start: {sec: 5}}]}}"
```

```bash
# 启动moveit2+gazebo仿真
ros2 launch vendor_robot_simulation moveit_gazebo_in_simulation.launch.py robot_type:=g9
```
启动成功之后弹出界面：
![alt text](doc/gazebo2.png)

之后可以进行moveit2和gazebo的仿真控制

```bash
# 示例
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.2, -0.3, 0.3, 0.0, 0.1, 0.0], time_from_start: {sec: 5}}]}}"

#movel
ros2 action send_goal /move_l vendor_robot_msgs/action/MoveL "{target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}, velocity: 0.05, acceleration: 0.05, blend_radius: 0.0}"
```

当前 `prefix` 必须为空：

```bash
ros2 launch vendor_robot_simulation simulation.launch.py prefix:=
```

传入非空 prefix 会直接报错，因为 controller YAML 使用未加前缀的 joint 名。

