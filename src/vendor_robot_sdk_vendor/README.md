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

MoveJ/L/C 等 SDK API 虽在头文件中存在，当前没有接入 ROS Action server。

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

