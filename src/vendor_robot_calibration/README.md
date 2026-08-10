# vendor_robot_calibration

## 1. 功能包作用

本包保存机器人序列号、运动学文件哈希和人工验证标志，并提供命令行校验脚本，避免把通用 URDF 或其他机器的标定参数误用于当前机器人。

本包不创建 ROS topic、service 或 action。

本包功能目前暂未实现。

## 2. 目录结构

```text
vendor_robot_calibration/
├── config/calibration.yaml
├── scripts/validate_calibration.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 3. 配置字段

默认文件包含：

```yaml
schema_version: 1
robot_serial: UNSET
kinematic_parameters:
  verified: false
  sha256: ""
```

正式使用前必须设置真实序列号、已审核运动学文件的 SHA-256 和 `verified: true`。

## 4. 使用示例

```bash
ros2 run vendor_robot_calibration validate_calibration.py \
  /path/to/calibration.yaml \
  --robot-serial ACTUAL_SERIAL \
  --kinematics-file /path/to/kinematics.yaml
```

成功输出：

```text
calibration identity and checksum are valid
```

失败时退出码为 2，并列出：

- schema 版本不支持；
- 序列号未绑定或不匹配；
- 运动学未验证；
- SHA-256 不匹配。

生成哈希：

```bash
sha256sum /path/to/kinematics.yaml
```

## 5. 与控制链路的关系

当前校验脚本是独立工具，没有接入 `robot.launch.py` 或 `real_moveit.launch.py`，因此校验失败不会阻止真机启动。上线流程应在 launch 前显式执行，或后续增加启动前置检查。

