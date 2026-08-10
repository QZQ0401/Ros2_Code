#!/usr/bin/env python3
"""运行：笛卡尔 RPY 路点 -> MoveIt IK -> 平滑关节轨迹 -> JTC。"""

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
# 每行依次为 [x, y, z, rx, ry, rz]
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
    [578.197, 342.703, 587.449, 42.7630, 71.2108, 79.0782],
    [582.462, 344.199, 586.354, 40.1292, 71.6058, 77.0418],
    [586.663, 345.807, 585.176, 37.4269, 71.9764, 74.9676],
    [590.794, 347.525, 583.908, 34.6641, 72.3225, 72.8630],
    [594.833, 349.369, 582.530, 31.8580, 72.6453, 70.7497],
    [598.788, 351.318, 581.052, 29.0124, 72.9445, 68.6253],
    [602.653, 353.365, 579.473, 26.1366, 73.2206, 66.4977],
    [606.404, 355.521, 577.771, 23.2528, 73.4763, 64.3919],
    [610.052, 357.765, 575.960, 20.3616, 73.7118, 62.3029],
    [613.594, 360.088, 574.042, 17.4718, 73.9283, 60.2372],
    [617.006, 362.496, 571.996, 14.6060, 74.1297, 58.2191],
    [620.301, 364.968, 569.840, 11.7611, 74.3162, 56.2404],
    [623.479, 367.497, 567.576, 8.9426, 74.4896, 54.3045],
    [626.523, 370.081, 565.193, 6.1665, 74.6535, 52.4271],
    [629.440, 372.706, 562.699, 3.4328, 74.8092, 50.6053],
    [632.236, 375.365, 560.103, 0.7396, 74.9577, 48.8347],
    [634.910, 378.049, 557.408, -1.9108, 75.1008, 47.1159],
    [637.446, 380.754, 554.602, -4.5047, 75.2426, 45.4621],
    [639.862, 383.470, 551.702, -7.0535, 75.3824, 43.8591],
    [642.159, 386.192, 548.713, -9.5589, 75.5215, 42.3038],
    [644.341, 388.912, 545.637, -12.0223, 75.6609, 40.7937],
    [646.408, 391.627, 542.478, -14.4454, 75.8018, 39.3258],
    [648.351, 394.330, 539.231, -16.8226, 75.9470, 37.9047],
    [650.186, 397.017, 535.909, -19.1652, 76.0954, 36.5179],
    [651.914, 399.684, 532.514, -21.4773, 76.2475, 35.1605],
    [653.540, 402.327, 529.052, -23.7623, 76.4036, 33.8284],
    [655.066, 404.944, 525.524, -26.0242, 76.5642, 32.5169],
    [656.494, 407.532, 521.934, -28.2669, 76.7296, 31.2215],
    [657.827, 410.087, 518.285, -30.4948, 76.8997, 29.9372],
    [659.069, 412.608, 514.580, -32.7124, 77.0748, 28.6593],
    [660.222, 415.092, 510.822, -34.9242, 77.2546, 27.3825],
    [661.289, 417.538, 507.014, -37.1351, 77.4391, 26.1019],
    [662.272, 419.944, 503.158, -39.3500, 77.6281, 24.8121],
    [663.174, 422.308, 499.257, -41.5741, 77.8212, 23.5079],
    [663.999, 424.630, 495.313, -43.8124, 78.0180, 22.1838],
    [664.748, 426.909, 491.329, -46.0705, 78.2181, 20.8343],
    [665.423, 429.143, 487.308, -48.3539, 78.4211, 19.4538],
    [666.029, 431.333, 483.250, -50.6681, 78.6262, 18.0365],
    [666.565, 433.477, 479.159, -53.0190, 78.8329, 16.5765],
    [667.036, 435.575, 475.036, -55.4125, 79.0404, 15.0680],
    [667.443, 437.628, 470.883, -57.8546, 79.2480, 13.5049],
    [667.788, 439.634, 466.703, -60.3513, 79.4548, 11.8811],
    [668.074, 441.595, 462.496, -62.9087, 79.6598, 10.1907],
    [668.301, 443.509, 458.264, -65.5327, 79.8621, 8.4278],
    [668.467, 445.373, 454.007, -68.2279, 80.0617, 6.5872],
    [668.574, 447.186, 449.726, -71.0006, 80.2572, 4.6628],
    [668.628, 448.953, 445.425, -73.8572, 80.4465, 2.6486],
    [668.631, 450.674, 441.105, -76.8024, 80.6284, 0.5401],
    [668.585, 452.350, 436.768, 100.1600, 99.1985, 178.3336],
    [668.490, 453.980, 432.414, 97.0273, 99.0358, 176.0264],
    [668.345, 455.562, 428.043, 93.7981, 98.8843, 173.6168],
    [668.145, 457.089, 423.655, 90.4710, 98.7443, 171.1029],
    [667.901, 458.572, 419.255, 87.0484, 98.6193, 168.4881],
    [667.615, 460.010, 414.842, 83.5347, 98.5109, 165.7770],
    [667.287, 461.406, 410.418, 79.9366, 98.4206, 162.9766],
    [666.914, 462.753, 405.983, 76.2619, 98.3491, 160.0939],
    [666.492, 464.046, 401.536, 72.5188, 98.2970, 157.1367],
    [666.031, 465.296, 397.081, 68.7251, 98.2672, 154.1241],
    [665.534, 466.504, 392.618, 64.8975, 98.2609, 151.0730],
    [665.000, 467.671, 388.149, 61.0541, 98.2789, 148.0017],
    [664.416, 468.779, 383.670, 57.2031, 98.3197, 144.9162],
    [663.797, 469.845, 379.186, 53.3737, 98.3859, 141.8480],
    [663.145, 470.872, 374.698, 49.5853, 98.4778, 138.8167],
    [662.460, 471.858, 370.205, 45.8553, 98.5951, 135.8396],
    [661.728, 472.784, 365.708, 42.1884, 98.7359, 132.9195],
    [660.965, 473.672, 361.207, 38.6115, 98.9015, 130.0855],
    [660.172, 474.523, 356.705, 35.1368, 99.0911, 127.3501],
    [659.343, 475.326, 352.200, 31.7674, 99.3033, 124.7153],
    [658.475, 476.079, 347.694, 28.5086, 99.5369, 122.1863],
    [657.580, 476.795, 343.188, 25.3733, 99.7919, 119.7774],
    [656.659, 477.476, 338.681, 22.3638, 100.0668, 117.4911],
]
WAYPOINTS = []
for point in WAYPOINTS_INPUT:
        WAYPOINTS.append({
        'x': point[0] / 1000.0,
        'y': point[1] / 1000.0,
        'z': point[2] / 1000.0,
        'rx': math.radians(point[3]),
        'ry': math.radians(point[4]),
        'rz': math.radians(point[5])
        })

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
FRAME_ID = 'base_link'
MOVE_GROUP = 'arm'
IK_SERVICE = '/compute_ik'
JTC_ACTION = '/arm_controller/follow_joint_trajectory'

AVOID_COLLISIONS = True
IK_TIMEOUT = 1.0
START_DELAY = 0.10
MIN_SEGMENT_DURATION = 0.05  #相邻两点之间最小时间（秒），避免过短导致 JTC 采样过冲，视相邻路点joint改变量而定。
APPROACH_TIME_SCALE = 3.0  # 第一段（当前位置→120点第一个waypoint）时间放大倍率，
                            #防止机械臂当前位置与120个点第一个点相距过大导致速度过快，出现过冲。

# joint_limits.yaml；
MAX_VELOCITIES = [3.65, 3.65, 3.65, 6.28, 6.28, 6.28]
MAX_ACCELERATIONS = [10.0, 10.0, 10.0, 20.0, 20.0, 20.0]

# 调节120点轨迹速度：下方两个参数用于调节轨迹速度，单位为倍数。
#  实机第一次建议保持 0.1，验证后再逐步提高，范围 (0, 1]。
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


class CartesianTrajectoryNode(Node):
    def __init__(self):
        super().__init__('cartesian_trajectory_direct')
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
                raise ValueError(
                    f'WAYPOINTS[{index}] 必须包含 6 个数字。')
            if not all(isinstance(value, (int, float)) and math.isfinite(value)
                       for value in waypoint.values()):
                raise ValueError(
                    f'WAYPOINTS[{index}] 含有无效数字。')
        if len(JOINT_NAMES) != 6:
            raise ValueError('JOINT_NAMES 必须包含 6 个关节名。')
        if len(MAX_VELOCITIES) != 6 or len(MAX_ACCELERATIONS) != 6:
            raise ValueError('速度和加速度上限必须各包含 6 个值。')
        if not 0.0 < VELOCITY_SCALING <= 1.0:
            raise ValueError('VELOCITY_SCALING 必须在 (0, 1]。')
        if not 0.0 < ACCELERATION_SCALING <= 1.0:
            raise ValueError('ACCELERATION_SCALING 必须在 (0, 1]。')
        if MIN_SEGMENT_DURATION <= 0.0:
            raise ValueError('MIN_SEGMENT_DURATION 必须大于 0。')

    def current_arm_positions(self):
        state = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ))
        missing = [name for name in JOINT_NAMES if name not in state]
        if missing:
            raise RuntimeError('/joint_states 缺少关节: ' + ', '.join(missing))
        return [state[name] for name in JOINT_NAMES]

    def convert_and_execute(self):
        if not self.ik_client.service_is_ready():
            self.get_logger().info(f'等待 IK 服务 {IK_SERVICE}...')
            if not self.ik_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f'IK 服务 {IK_SERVICE} 5 秒内未就绪。')

        joint_waypoints = [self.current_arm_positions()]
        seed_state = self.current_joint_state
        self.get_logger().info(
            f'开始转换 {len(WAYPOINTS)} 个 waypoints。')

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
            pose.pose.orientation = rpy_to_quaternion(values['rx'], values['ry'], values['rz'])
            request.ik_request.pose_stamped = pose

            response = self.ik_client.call(request)
            if response.error_code.val != response.error_code.SUCCESS:
                raise RuntimeError(
                    f'第 {index + 1} 个点 IK 失败，MoveIt 错误码: '
                    f'{response.error_code.val}。轨迹未发送。')

            solution = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            missing = [name for name in JOINT_NAMES if name not in solution]
            if missing:
                raise RuntimeError(
                    f'第 {index + 1} 个 IK 结果缺少关节: {missing}')
            positions = [solution[name] for name in JOINT_NAMES]
            joint_waypoints.append(positions)
            seed_state = response.solution.joint_state
            formatted = ', '.join(f'{value:.5f}' for value in positions)
            self.get_logger().info(f'IK {index + 1}: [{formatted}]')

        trajectory = self.make_smooth_trajectory(joint_waypoints)
        total_time = duration_to_seconds(
            trajectory.points[-1].time_from_start)
        self.get_logger().info(
            f'IK 和时间参数化完成，总时间 {total_time:.3f} 秒。')
        self.send_and_wait(trajectory)

    @staticmethod
    def effective_limits():
        vmax = [value * VELOCITY_SCALING for value in MAX_VELOCITIES]
        amax = [value * ACCELERATION_SCALING
                for value in MAX_ACCELERATIONS]
        if min(vmax + amax) <= 0.0:
            raise ValueError('速度和加速度上限必须为正数。')
        return vmax, amax

    def make_smooth_trajectory(self, positions):
        vmax, amax = self.effective_limits()

        # 根据每段最大关节位移给出保守时间
        durations = []
        for seg_idx, (start, end) in enumerate(zip(positions[:-1], positions[1:])):
            segment_time = MIN_SEGMENT_DURATION
            for joint_index, (q0, q1) in enumerate(zip(start, end)):
                delta = abs(q1 - q0)
                velocity_time = 2.0 * delta / vmax[joint_index]
                acceleration_time = (
                    math.sqrt(6.0 * delta / amax[joint_index])
                    if delta > 0.0 else 0.0
                )
                segment_time = max(
                    segment_time, velocity_time, acceleration_time)
            # 第一段（当前位置→首个waypoint）额外放慢，避免远距离猛冲
            if seg_idx == 0:
                segment_time *= APPROACH_TIME_SCALE
            durations.append(segment_time)

        # 计算路点连续速度；再按 JTC 实际使用的五次曲线采样检查。
        # 若任何关节超限，整体延长时间并重新计算速度。
        velocities = None
        for _ in range(4):
            velocities = self.waypoint_velocities(
                positions, durations, vmax)
            ratio = self.sample_limit_ratio(
                positions, velocities, durations, vmax, amax)
            if ratio <= 1.0 + 1.0e-6:
                break
            time_scale = 1.02 * ratio
            durations = [value * time_scale for value in durations]
        else:
            raise RuntimeError('时间缩放后仍不能满足速度/加速度限制。')

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        elapsed = max(0.01, START_DELAY)
        for index, positions_at_point in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = list(positions_at_point)
            point.velocities = list(velocities[index])
            point.accelerations = [0.0] * len(JOINT_NAMES)
            point.time_from_start = seconds_to_duration(elapsed)
            trajectory.points.append(point)
            if index < len(durations):
                elapsed += durations[index]
        return trajectory

    @staticmethod
    def waypoint_velocities(positions, durations, vmax):
        velocities = [[0.0] * len(JOINT_NAMES) for _ in positions]
        for point_index in range(1, len(positions) - 1):
            time_before = durations[point_index - 1]
            time_after = durations[point_index]
            for joint_index in range(len(JOINT_NAMES)):
                slope_before = (
                    positions[point_index][joint_index]
                    - positions[point_index - 1][joint_index]
                ) / time_before
                slope_after = (
                    positions[point_index + 1][joint_index]
                    - positions[point_index][joint_index]
                ) / time_after

                # 发生运动方向反转时在该路点停下，避免样条过冲。
                if slope_before * slope_after <= 0.0:
                    velocity = 0.0
                else:
                    velocity = (
                        time_after * slope_before
                        + time_before * slope_after
                    ) / (time_before + time_after)
                velocities[point_index][joint_index] = max(
                    -vmax[joint_index], min(vmax[joint_index], velocity))
        # 起点和终点速度保持为零，实现平滑起停。
        return velocities

    @staticmethod
    def sample_limit_ratio(positions, velocities, durations, vmax, amax):
        """采样 JTC 五次插值，返回所需的时间放大倍率。"""
        worst_ratio = 0.0
        for segment_index, segment_time in enumerate(durations):
            for joint_index in range(len(JOINT_NAMES)):
                p0 = positions[segment_index][joint_index]
                p1 = positions[segment_index + 1][joint_index]
                v0 = velocities[segment_index][joint_index]
                v1 = velocities[segment_index + 1][joint_index]

                # 归一化时间 u 下的五次多项式系数；两端加速度为零。
                displacement = p1 - p0 - v0 * segment_time
                velocity_change = (v1 - v0) * segment_time
                c1 = v0 * segment_time
                c2 = 0.0
                c3 = 10.0 * displacement - 4.0 * velocity_change
                c4 = -15.0 * displacement + 7.0 * velocity_change
                c5 = 6.0 * displacement - 3.0 * velocity_change

                for sample_index in range(101):
                    u = sample_index / 100.0
                    velocity = (
                        c1 + 2.0*c2*u + 3.0*c3*u**2
                        + 4.0*c4*u**3 + 5.0*c5*u**4
                    ) / segment_time
                    acceleration = (
                        2.0*c2 + 6.0*c3*u + 12.0*c4*u**2
                        + 20.0*c5*u**3
                    ) / (segment_time * segment_time)
                    worst_ratio = max(
                        worst_ratio,
                        abs(velocity) / vmax[joint_index],
                        math.sqrt(
                            abs(acceleration) / amax[joint_index]),
                    )
        return worst_ratio

    def send_and_wait(self, trajectory):
        if not self.action_client.server_is_ready():
            self.get_logger().info(f'等待 JTC Action {JTC_ACTION}...')
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f'JTC Action {JTC_ACTION} 5 秒内未就绪。')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        # 同步等待 send_goal 完成
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('轨迹被 JTC 拒绝，请检查控制器状态和约束。')

        self.get_logger().info('JTC 已接受轨迹，等待执行完成...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        result = wrapped_result.result
        if result.error_code != result.SUCCESSFUL:
            raise RuntimeError(
                f'执行失败: status={wrapped_result.status}, '
                f'error_code={result.error_code}, '
                f'error_string={result.error_string!r}')
        self.get_logger().info('轨迹执行成功，程序即将退出。')

    @staticmethod
    def feedback_callback(feedback_message):
        _ = feedback_message


def main(args=None):
    rclpy.init(args=args)
    node = CartesianTrajectoryNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            try:
                executor.spin_once(timeout_sec=0.1)
            except rclpy._rclpy_pybind11.RCLError:
                # 轨迹完成后 action client 可能状态异常（Humble 已知问题），安全退出
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