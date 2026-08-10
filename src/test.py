import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
import math
import numpy as np
from scipy.interpolate import CubicSpline

class ReactionNode(Node):
    def __init__(self):
        super().__init__('path_to_joint_relay')
        self.cb_group = ReentrantCallbackGroup()

        self.current_joint_state = None
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_group)

        # IK客户端
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        
        # Action客户端
        self.action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory', 
            callback_group=self.cb_group)

        # 轨迹规划参数
        self.TOTAL_TIME = 15.0
        self.NUM_INTERPOLATION_POINTS = 150
        
        # 速度/加速度约束
        self.MAX_VELOCITY = 0.3
        self.MAX_ACCELERATION = 0.2
        
        # 存储笛卡尔点
        self.cartesian_points = []
        
        # 加载测试数据
        self.load_test_points()
        
        # 定时处理
        self.timer = self.create_timer(5.0, self.process_cartesian_points_smooth, 
                                      callback_group=self.cb_group)
        
        self.get_logger().info('>>> 节点已启动，等待关节状态...')

    def load_test_points(self):
        """加载测试数据"""
        raw_data = [
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
            [573.866, 341.341, 588.452, 45.3248, 70.7927, 81.0795]
        ]
        
        test_points = []
        for point in raw_data:
            test_points.append({
                'x': point[0] / 1000.0,
                'y': point[1] / 1000.0,
                'z': point[2] / 1000.0,
                'rx': math.radians(point[3]),
                'ry': math.radians(point[4]),
                'rz': math.radians(point[5])
            })
        
        self.cartesian_points = test_points
        self.get_logger().info(f"已加载 {len(test_points)} 个笛卡尔点")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Pose()
        q.orientation.x = sr * cp * cy - cr * sp * sy
        q.orientation.y = cr * sp * cy + sr * cp * sy
        q.orientation.z = cr * cp * sy - sr * sp * cy
        q.orientation.w = cr * cp * cy + sr * sp * sy
        return q

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def get_current_joint_angles(self):
        """获取当前关节角度"""
        if self.current_joint_state is None:
            return None
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        angles = []
        full_names = self.current_joint_state.name
        full_positions = self.current_joint_state.position
        
        for name in joint_names:
            try:
                idx = full_names.index(name)
                angles.append(full_positions[idx])
            except ValueError:
                self.get_logger().error(f"找不到关节 {name}")
                return None
        return angles

    async def call_ik(self, pose_stamped, seed_state):
        """调用IK服务"""
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.timeout = rclpy.duration.Duration(seconds=2.0).to_msg()
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.robot_state.joint_state = seed_state
        
        response = await self.ik_client.call_async(ik_request)
        return response

    def create_pose_stamped(self, point_data):
        """从点数据创建PoseStamped"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = point_data['x']
        pose_stamped.pose.position.y = point_data['y']
        pose_stamped.pose.position.z = point_data['z']
        
        quat = self.euler_to_quaternion(
            point_data['rx'], 
            point_data['ry'], 
            point_data['rz']
        )
        pose_stamped.pose.orientation = quat.orientation
        
        return pose_stamped

    def diagnose_trajectory(self, trajectory):
        """诊断轨迹问题"""
        if len(trajectory.points) < 2:
            self.get_logger().error("轨迹点数太少")
            return False
        
        # 检查时间是否递增（从第一个点开始）
        prev_time = -1.0  # 初始化为负数
        for i, point in enumerate(trajectory.points):
            curr_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if curr_time <= prev_time:
                self.get_logger().error(f"时间戳不递增: 点{i}时间={curr_time:.6f}, 前一点时间={prev_time:.6f}")
                return False
            prev_time = curr_time
        
        self.get_logger().info(f"时间戳检查通过: 从 {trajectory.points[0].time_from_start.sec}.{trajectory.points[0].time_from_start.nanosec:09d} 到 {trajectory.points[-1].time_from_start.sec}.{trajectory.points[-1].time_from_start.nanosec:09d}")
        
        # 检查速度和加速度
        max_vel = 0
        max_acc = 0
        for point in trajectory.points:
            if hasattr(point, 'velocities') and point.velocities:
                vel_max = max(abs(v) for v in point.velocities)
                max_vel = max(max_vel, vel_max)
            if hasattr(point, 'accelerations') and point.accelerations:
                acc_max = max(abs(a) for a in point.accelerations)
                max_acc = max(max_acc, acc_max)
        
        self.get_logger().info(f"最大速度: {max_vel:.3f} rad/s")
        self.get_logger().info(f"最大加速度: {max_acc:.3f} rad/s²")
        
        if max_vel > self.MAX_VELOCITY * 1.5:
            self.get_logger().warn(f"速度超过限制: {max_vel:.3f} > {self.MAX_VELOCITY:.3f}")
        if max_acc > self.MAX_ACCELERATION * 1.5:
            self.get_logger().warn(f"加速度超过限制: {max_acc:.3f} > {self.MAX_ACCELERATION:.3f}")
        
        return True

    def generate_smooth_trajectory(self, waypoints_joint_angles):
        """生成平滑轨迹"""
        if len(waypoints_joint_angles) < 2:
            return None
        
        n_joints = len(waypoints_joint_angles[0])
        n_waypoints = len(waypoints_joint_angles)
        
        # 使用均匀时间分布
        time_nodes = np.linspace(0, self.TOTAL_TIME, n_waypoints)
        time_interp = np.linspace(0, self.TOTAL_TIME, self.NUM_INTERPOLATION_POINTS)
        
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 为每个关节生成轨迹
        for joint_idx in range(n_joints):
            joint_values = [wp[joint_idx] for wp in waypoints_joint_angles]
            
            # 创建样条插值
            cs = CubicSpline(time_nodes, joint_values, bc_type='clamped')
            
            positions = cs(time_interp)
            velocities = cs.derivative()(time_interp)
            accelerations = cs.derivative(2)(time_interp)
            
            # 限制速度和加速度
            velocities = np.clip(velocities, -self.MAX_VELOCITY, self.MAX_VELOCITY)
            accelerations = np.clip(accelerations, -self.MAX_ACCELERATION, self.MAX_ACCELERATION)
            
            # 添加到轨迹点
            for i in range(len(time_interp)):
                # 如果当前索引还没有点，创建一个新点
                if i >= len(joint_trajectory.points):
                    point = JointTrajectoryPoint()
                    point.positions = []
                    point.velocities = []
                    point.accelerations = []
                    total_nanoseconds = int(time_interp[i] * 1e9)
                    point.time_from_start.sec = total_nanoseconds // 1000000000
                    point.time_from_start.nanosec = total_nanoseconds % 1000000000
                    joint_trajectory.points.append(point)
                
                # 添加关节数据
                joint_trajectory.points[i].positions.append(positions[i])
                joint_trajectory.points[i].velocities.append(velocities[i])
                joint_trajectory.points[i].accelerations.append(accelerations[i])
        
        return joint_trajectory

    async def process_cartesian_points_smooth(self):
        """处理笛卡尔点"""
        if not self.cartesian_points:
            return

        if self.current_joint_state is None:
            self.get_logger().warn("等待关节状态...")
            return

        if not self.ik_client.service_is_ready():
            if not self.ik_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("IK服务未就绪")
                return

        self.get_logger().info(f"开始处理 {len(self.cartesian_points)} 个笛卡尔点...")
        
        current_angles = self.get_current_joint_angles()
        if current_angles is None:
            return
        
        joint_waypoints = [current_angles]
        seed_state = self.current_joint_state
        
        for i, point_data in enumerate(self.cartesian_points):
            pose_stamped = self.create_pose_stamped(point_data)
            response = await self.call_ik(pose_stamped, seed_state)
            
            if response.error_code.val == response.error_code.SUCCESS:
                full_names = response.solution.joint_state.name
                full_positions = response.solution.joint_state.position
                joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                
                angles = []
                for name in joint_names:
                    idx = full_names.index(name)
                    angles.append(full_positions[idx])
                
                joint_waypoints.append(angles)
                seed_state = response.solution.joint_state
                
                if (i + 1) % 10 == 0:
                    self.get_logger().info(f"已处理 {i+1}/{len(self.cartesian_points)} 个点")
            else:
                self.get_logger().error(f"点 {i} IK 失败，错误码: {response.error_code.val}")
                return
        
        self.get_logger().info(f"IK转换完成，共{len(joint_waypoints)}个点")
        
        # 生成平滑轨迹
        smooth_trajectory = self.generate_smooth_trajectory(joint_waypoints)
        
        if smooth_trajectory is None:
            self.get_logger().error("生成平滑轨迹失败")
            return
        
        self.get_logger().info(f"生成轨迹包含 {len(smooth_trajectory.points)} 个插值点")
        
        # 诊断轨迹
        if not self.diagnose_trajectory(smooth_trajectory):
            self.get_logger().error("轨迹诊断失败，不发送")
            return
        
        # 发送轨迹
        self.send_goal(smooth_trajectory)

    def send_goal(self, trajectory):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("控制器 Action 不在线")
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # 添加轨迹容差
        from control_msgs.msg import JointTolerance
        for joint_name in trajectory.joint_names:
            tolerance = JointTolerance()
            tolerance.name = joint_name
            tolerance.position = 0.01
            tolerance.velocity = 0.01
            tolerance.acceleration = 0.01
            goal_msg.goal_tolerance.append(tolerance)
        
        # 设置目标时间容差
        goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        self.get_logger().info(f"发送轨迹到Action：{len(trajectory.points)} 个点，总时间 {self.TOTAL_TIME}s")
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝")
            return
        self.get_logger().info("轨迹目标已被接受")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(
            f"轨迹执行完成，status={status}, error_code={result.error_code}, "
            f"error_string='{result.error_string}'"
        )

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ReactionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()