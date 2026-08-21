# vendor_robot_sdk_vendor

## 1. 功能包作用

本包封装厂商提供的 SDK C 头文件和 x86_64 动态库，使其他 ROS 2 包可以通过 CMake target `vendor_robot_sdk_vendor::cr_sdk` 链接管理 SDK。


## 2. 目录结构

```text
vendor_robot_sdk_vendor/
├── include/
│   ├── robotapi.h
│   └── basestruct.h
├── lib/
│   ├── libcr_sdk.so*
│   ├── libRobotConfigDll.so*
│   ├── libtpAlgApp.so*
│   ├── libLogStorageDll.so*
│   ├── libCGXIZip.so*
│   ├── libprotobuf-c.so*
│   ├── libprotobuf-rpc.so*
│   └── libz.so*
├── vendor_robot_sdk_vendor-extras.cmake
├── CMakeLists.txt
└── package.xml
```

## 3. 当前使用的 SDK 接口

`vendor_robot_controllers/sdk_manager_node.cpp` 使用：

| SDK 接口 | ROS 映射 |
|---|---|
| `cr_create_robot` / `cr_destroy_robot` | SDK 连接生命周期 |
| `cr_get_robotMode` | `/robot_mode`、推导 `/safety_mode` |
| `cr_poweron` | `/power_on` |
| `cr_enable` | `/enable_robot` |
| `cr_disable` | `/disable_robot` |
| `cr_FaultReset` | `/clear_error` |
| `cr_stop` | `/stop_program` 和当前自动 ProgramStop 恢复 |
| `cr_set_ToolOutputVoltage` | `/set_tool_power` |
| `cr_cfg_payload_*` | `/set_payload` |

MoveJ/L/C 和 Movex 等 SDK 运动 API 不接入 ROS Action server。SDK 运动和 RTDE
`servoj` 互斥；必须完全停止 ROS 2 控制链路（包括 `ros2_control`、MoveIt 和
`sdk_manager_node`）后，才可运行独立 SDK 程序，禁止并发控制同一机器人。

## 4. 构建与检查

本包只允许 x86_64：

```bash
uname -m
file src/vendor_robot_sdk_vendor/lib/libcr_sdk.so.1.0.0
ldd src/vendor_robot_sdk_vendor/lib/libcr_sdk.so.1.0.0
```

构建：

```bash
colcon build --symlink-install --packages-select vendor_robot_sdk_vendor
source install/setup.bash
```

## 4.1 Movex C++ 示例

`src/movex.cpp` 已整理为可独立运行的 SDK 示例：读取当前关节和 TCP 位姿，下载 5 个点组成的 Movex 轨迹，启动后 5 秒停止。它会控制真实机器人，运行前请确认现场安全、机器人已上电使能，且 ROS 2 控制链路已完全停止。

```bash
ros2 run vendor_robot_sdk_vendor movex [robot_ip] [sdk_port] [password]
```

默认值为 `192.168.6.6`、`2323` 和空密码。

检查头文件版本：

```bash
grep CR_ROBOT_SDK_VERSION \
  src/vendor_robot_sdk_vendor/include/robotapi.h
```

## 5. 与 RTDE 的边界

```text
vendor_robot_sdk_vendor
  └─ 低频管理：上电、使能、模式、Payload、工具电源

vendor_robot_driver_core/lib/libcdrtde*.so
  └─ 高频状态、servoj、数字 IO、速度比例
```

两套库、连接和线程相互独立。
