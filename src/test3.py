#!/usr/bin/env python3
"""笛卡尔 RPY 路点 -> MoveIt IK -> 按速度/加速度上限规划时间 -> JTC（不平滑）。"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ============================================================================
# 用户配置区：每行 [x, y, z, rx, ry, rz]
# x/y/z 单位：米；rx/ry/rz 单位：度
# ============================================================================
WAYPOINTS_INPUT = [
        [357.103, 357.075, 627.019, 89.7819, 41.1293, 126.3351],
        [361.463, 356.234, 625.640, 89.6988, 41.7363, 125.8933],
        [365.832, 355.384, 624.293, 89.6000, 42.3466, 125.4445],
        [370.209, 354.527, 622.976, 89.4848, 42.9599, 124.9880],
        [374.594, 353.664, 621.692, 89.3526, 43.5764, 124.5236],
        [378.988, 352.794, 620.441, 89.2023, 44.1962, 124.0506],
        [383.392, 351.921, 619.232, 89.0311, 44.8201, 123.5683],
        [387.805, 351.045, 618.054, 88.8407, 45.4471, 123.0766],
        [392.226, 350.168, 616.908, 88.6302, 46.0770, 122.5748],
        [396.655, 349.292, 615.795, 88.3989, 46.7098, 122.0625],
        [401.093, 348.418, 614.715, 88.1457, 47.3455, 121.5389],
        [405.540, 347.548, 613.669, 87.8689, 47.9844, 121.0033],
        [409.996, 346.684, 612.661, 87.5667, 48.6266, 120.4549],
        [414.461, 345.828, 611.684, 87.2397, 49.2714, 119.8933],
        [418.935, 344.980, 610.739, 86.8867, 49.9186, 119.3175],
        [423.417, 344.144, 609.825, 86.5065, 50.5682, 118.7266],
        [427.908, 343.321, 608.943, 86.0977, 51.2201, 118.1199],
        [432.408, 342.513, 608.092, 85.6589, 51.8741, 117.4962],
        [436.916, 341.722, 607.271, 85.1888, 52.5300, 116.8546],
        [441.433, 340.951, 606.481, 84.6855, 53.1877, 116.1938],
        [445.960, 340.204, 605.722, 84.1464, 53.8474, 115.5129],
        [450.495, 339.481, 604.991, 83.5712, 54.5083, 114.8104],
        [455.039, 338.786, 604.287, 82.9578, 55.1702, 114.0852],
        [459.591, 338.121, 603.610, 82.3045, 55.8328, 113.3356],
        [464.152, 337.488, 602.957, 81.6090, 56.4957, 112.5602],
        [468.721, 336.891, 602.328, 80.8692, 57.1586, 111.7574],
        [473.297, 336.332, 601.722, 80.0829, 57.8211, 110.9255],
        [477.881, 335.815, 601.136, 79.2474, 58.4827, 110.0625],
        [482.473, 335.344, 600.569, 78.3604, 59.1430, 109.1667],
        [487.071, 334.920, 600.020, 77.4191, 59.8014, 108.2359],
        [491.675, 334.549, 599.485, 76.4206, 60.4573, 107.2681],
        [496.285, 334.233, 598.963, 75.3621, 61.1100, 106.2609],
        [500.900, 333.976, 598.451, 74.2405, 61.7589, 105.2122],
        [505.519, 333.782, 597.947, 73.0527, 62.4032, 104.1194],
        [510.140, 333.655, 597.446, 71.7954, 63.0419, 102.9800],
        [514.763, 333.601, 596.947, 70.4648, 63.6745, 101.7920],
        [519.386, 333.626, 596.444, 69.0574, 64.2999, 100.5531],
        [524.007, 333.731, 595.935, 67.5708, 64.9169, 99.2605],
        [528.624, 333.919, 595.415, 66.0017, 65.5243, 97.9118],
        [533.235, 334.195, 594.879, 64.3471, 66.1209, 96.5047],
        [537.837, 334.562, 594.324, 62.6044, 66.7055, 95.0372],
        [542.428, 335.024, 593.743, 60.7711, 67.2766, 93.5075],
        [547.002, 335.596, 593.129, 58.8442, 67.8336, 91.9170],
        [551.556, 336.274, 592.478, 56.8234, 68.3746, 90.2635],
        [556.087, 337.057, 591.785, 54.7083, 68.8981, 88.5460],
        [560.590, 337.948, 591.045, 52.4987, 69.4026, 86.7650],
        [565.058, 338.961, 590.246, 50.1964, 69.8875, 84.9256],
        [569.484, 340.094, 589.382, 47.8041, 70.3513, 83.0303],
        [573.866, 341.341, 588.452, 45.3248, 70.7927, 81.0795],
]
WAYPOINTS = []
for point in WAYPOINTS_INPUT:
    WAYPOINTS.append({
        'x': point[0] / 1000.0,
        'y': point[1] / 1000.0,
        'z': point[2] / 1000.0,
        'rx': math.radians(point[3]),
        'ry': math.radians(point[4]),
        'rz': math.radians(point[5]),
    })

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
FRAME_ID = 'base_link'
MOVE_GROUP = 'arm'
IK_SERVICE = '/compute_ik'
JTC_ACTION = '/arm_controller/follow_joint_trajectory'

AVOID_COLLISIONS = True
IK_TIMEOUT = 1.0
START_DELAY = 0.20
MIN_SEGMENT_DURATION = 0.05
APPROACH_TIME_SCALE = 2.0  # 第一段时间放大倍率,防止机械臂当前位置与120个点第一个点相距过大导致速度过快，出现过冲。


# joint_limits.yaml
MAX_VELOCITIES = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]      # rad/s
MAX_ACCELERATIONS = [10.0, 10.0, 10.0, 20.0, 20.0, 20.0]    # rad/s²

# 速度/加速度缩放，范围 (0, 1]，实机建议从 0.1 起步
VELOCITY_SCALING = 1.0
ACCELERATION_SCALING = 1.0


def seconds_to_duration(seconds):
    nanoseconds = max(0, int(round(seconds * 1.0e9)))
    return Duration(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def duration_to_seconds(duration):
    return duration.sec + duration.nanosec * 1.0e-9


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    quaternion = Quaternion()
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    quaternion.w = cr * cp * cy + sr * sp * sy
    return quaternion


class DirectCartesianNode(Node):
    def __init__(self):
        super().__init__('direct_cartesian')
        self.callback_group = ReentrantCallbackGroup()
        self.current_joint_state = None
        self.started = False
        self.done = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            20,
            callback_group=self.callback_group,
        )
        self.ik_client = self.create_client(
            GetPositionIK,
            IK_SERVICE,
            callback_group=self.callback_group,
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            JTC_ACTION,
            callback_group=self.callback_group,
        )
        self.start_timer = self.create_timer(
            0.5,
            self.try_start,
            callback_group=self.callback_group,
        )
        self.get_logger().info('等待 /joint_states、MoveIt IK 和 JTC...')

    def joint_state_callback(self, message):
        self.current_joint_state = message

    def try_start(self):
        if self.started or self.current_joint_state is None:
            return
        self.started = True
        self.start_timer.cancel()
        try:
            self.validate_configuration()
            self.convert_and_execute()
        except (RuntimeError, ValueError) as error:
            self.get_logger().error(str(error))
        finally:
            self.done = True

    @staticmethod
    def validate_configuration():
        if not WAYPOINTS:
            raise ValueError('WAYPOINTS 不能为空。')
        for index, waypoint in enumerate(WAYPOINTS):
            if len(waypoint) != 6:
                raise ValueError(f'WAYPOINTS[{index}] 必须包含 6 个数字。')
            if not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in waypoint.values()):
                raise ValueError(f'WAYPOINTS[{index}] 含有无效数字。')
        if len(JOINT_NAMES) != 6:
            raise ValueError('JOINT_NAMES 必须包含 6 个关节名。')

    def current_arm_positions(self):
        state = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ))
        missing = [n for n in JOINT_NAMES if n not in state]
        if missing:
            raise RuntimeError('/joint_states 缺少关节: ' + ', '.join(missing))
        return [state[n] for n in JOINT_NAMES]

    # ------------------------------------------------------------------
    # 每段时间计算：按 max 速度/加速度，用梯形速度剖面
    # ------------------------------------------------------------------
    def segment_times(self, positions, vmax, amax):
        durations = []
        for seg_idx, (q0, q1) in enumerate(
                zip(positions[:-1], positions[1:])):
            seg_time = MIN_SEGMENT_DURATION
            for ji in range(len(JOINT_NAMES)):
                delta = abs(q1[ji] - q0[ji])
                if delta <= 0.0:
                    continue
                # 三角形剖面（加速→减速，未达 vmax）
                t_triangle = 2.0 * math.sqrt(delta / amax[ji])
                # 梯形剖面（加速→匀速→减速，达到 vmax）
                t_trapezoid = delta / vmax[ji] + vmax[ji] / amax[ji]
                # 位移足够大才能达到 vmax，否则只能用三角形
                if delta > vmax[ji] ** 2 / amax[ji]:
                    t_joint = t_trapezoid
                else:
                    t_joint = t_triangle
                seg_time = max(seg_time, t_joint)
            if seg_idx == 0:
                seg_time *= APPROACH_TIME_SCALE
            durations.append(seg_time)
        return durations

    # ------------------------------------------------------------------
    # IK + 构建轨迹 + 发送
    # ------------------------------------------------------------------
    def convert_and_execute(self):
        if not self.ik_client.service_is_ready():
            self.get_logger().info(f'等待 IK 服务 {IK_SERVICE}...')
            if not self.ik_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f'IK 服务 {IK_SERVICE} 5 秒内未就绪。')

        vmax = [v * VELOCITY_SCALING for v in MAX_VELOCITIES]
        amax = [a * ACCELERATION_SCALING for a in MAX_ACCELERATIONS]

        # 第一个点是当前位置
        joint_points = [self.current_arm_positions()]
        seed_state = self.current_joint_state

        self.get_logger().info(f'开始转换 {len(WAYPOINTS)} 个 waypoints。')
        for index, values in enumerate(WAYPOINTS):
            request = GetPositionIK.Request()
            request.ik_request.group_name = MOVE_GROUP
            request.ik_request.avoid_collisions = AVOID_COLLISIONS
            request.ik_request.timeout = seconds_to_duration(IK_TIMEOUT)
            request.ik_request.robot_state.joint_state = seed_state

            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(values['x'])
            pose.pose.position.y = float(values['y'])
            pose.pose.position.z = float(values['z'])
            pose.pose.orientation = rpy_to_quaternion(
                values['rx'], values['ry'], values['rz'])
            request.ik_request.pose_stamped = pose

            response = self.ik_client.call(request)
            if response.error_code.val != response.error_code.SUCCESS:
                raise RuntimeError(
                    f'第 {index + 1} 个点 IK 失败，错误码: '
                    f'{response.error_code.val}。轨迹未发送。')

            solution = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            missing = [n for n in JOINT_NAMES if n not in solution]
            if missing:
                raise RuntimeError(
                    f'第 {index + 1} 个 IK 结果缺少关节: {missing}')
            positions = [solution[n] for n in JOINT_NAMES]
            joint_points.append(positions)
            seed_state = response.solution.joint_state

            formatted = ', '.join(f'{v:.5f}' for v in positions)
            self.get_logger().info(f'IK {index + 1}: [{formatted}]')

        # 按速度/加速度上限计算每段时间
        durations = self.segment_times(joint_points, vmax, amax)

        # 构建 JointTrajectory（由 JTC 插值）
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        elapsed = START_DELAY
        for idx, pos in enumerate(joint_points):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in pos]
            point.time_from_start = seconds_to_duration(elapsed)
            trajectory.points.append(point)
            if idx < len(durations):
                elapsed += durations[idx]

        total_time = duration_to_seconds(
            trajectory.points[-1].time_from_start)
        self.get_logger().info(
            f'规划完成，{len(joint_points)} 个点，'
            f'总时间 {total_time:.3f} 秒。')
        self.send_and_wait(trajectory)

    def send_and_wait(self, trajectory):
        if not self.action_client.server_is_ready():
            self.get_logger().info(f'等待 JTC Action {JTC_ACTION}...')
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f'JTC Action {JTC_ACTION} 5 秒内未就绪。')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('轨迹被 JTC 拒绝。')

        self.get_logger().info('JTC 已接受，等待执行完成...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != result.SUCCESSFUL:
            raise RuntimeError(
                f'执行失败: error_code={result.error_code}, '
                f'error_string={result.error_string!r}')
        self.get_logger().info('轨迹执行成功。')


def main(args=None):
    rclpy.init(args=args)
    node = DirectCartesianNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            try:
                executor.spin_once(timeout_sec=0.1)
            except rclpy._rclpy_pybind11.RCLError:
                if node.done:
                    break
                raise
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
