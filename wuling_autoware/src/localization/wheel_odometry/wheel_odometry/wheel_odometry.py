import rclpy
from rclpy.node import Node
import message_filters
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseStamped
import math
from datetime import datetime
from builtin_interfaces.msg import Time

# 使用 math.pi 获取 pi 的值
pi_value = math.pi
C5_DISTANCE_WHEELS = 1.45  # 轮距，单位：米
robot_liner_y_ = 0.0  # 假设没有侧向速度
current_yaw_ = 0.0  # 初始朝向

robot_status = {
    'position_x_': 0.0,  # 初始位置
    'position_y_': 0.0,
    'position_yaw_': current_yaw_
}

class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')

        self.last_left_v = 0
        self.last_right_v = 0
        self.forward = 0
        # self.back = 0



        # 订阅左右轮速和左右脉冲信息
        left_velocity_sub = message_filters.Subscriber(self, VelocityReport, '~/input/driven_left_velocity')
        right_velocity_sub = message_filters.Subscriber(self, VelocityReport, '~/input/driven_right_velocity')
        left_pulse_sub = message_filters.Subscriber(self, VelocityReport, '~/input/driven_left_pulse')
        right_pulse_sub = message_filters.Subscriber(self, VelocityReport, '~/input/driven_right_pulse')

        # 使用 ApproximateTimeSynchronizer 来同步消息
        ts = message_filters.ApproximateTimeSynchronizer(
            [left_velocity_sub, right_velocity_sub, left_pulse_sub, right_pulse_sub], 
            queue_size=10, slop=0.02)  # 设置容忍度为 20 毫秒
        ts.registerCallback(self.synchronized_callback)


        # 创建Publisher来发布PoseStamped消息
        self.pose_publisher = self.create_publisher(PoseStamped, '~/output/position', 10)

        self.previous_timestamp = None

    def synchronized_callback(self, left_velocity_msg, right_velocity_msg, left_pulse_msg, right_pulse_msg):
        # 提取左右轮的时间戳
        timestamp = left_velocity_msg.header.stamp.sec + left_velocity_msg.header.stamp.nanosec / 1e9  # 使用左轮时间戳
        # 提取左右脉冲信息（如果需要）
        left_pulse = left_pulse_msg.longitudinal_velocity  # 假设脉冲与轮速相同，具体依赖消息定义
        right_pulse = right_pulse_msg.longitudinal_velocity  # 同上
        # 提取左右轮的速度
        left_velocity = left_velocity_msg.longitudinal_velocity
        right_velocity = right_velocity_msg.longitudinal_velocity
        
        # if abs(left_pulse) != 9999 or abs(left_pulse) != 8888:
        #     if left_pulse < 0:
        #         left_velocity = -left_velocity
        # if abs(right_pulse) != 9999 or abs(right_pulse) != 8888:
        #     if right_pulse < 0:
        #         right_velocity = -right_velocity


        # if abs(left_pulse) != 9999 and abs(left_pulse) != 8888:
        #     if left_pulse < 0:
        #         left_velocity = -left_velocity
        #     self.last_left_v = left_velocity
        # else:
        #     left_velocity = self.last_left_v

        # if abs(right_pulse) != 9999 or abs(right_pulse) != 8888:
        #     if right_pulse < 0 :
        #         right_velocity = -right_velocity
        #     self.last_right_v = right_velocity
        # else:
        #     right_velocity = self.last_right_v

        if abs(left_pulse) != 9999 and abs(left_pulse) != 8888:
            if left_pulse <= 0 :
                self.forward = 0
            else:
                self.forward = 1
        #print(self.forward)
        if self.forward == 0:
            left_velocity = -left_velocity
            right_velocity = -right_velocity


        # self.last_right_v = 0

        # 计算里程计数据
        self.calculate_odometry(timestamp, left_velocity, right_velocity)

    def calculate_odometry(self, timestamp, left_velocity, right_velocity):
        global robot_status

        # 计算时间步长 dt
        if self.previous_timestamp is None:
            self.previous_timestamp = timestamp
            return
        dt = timestamp - self.previous_timestamp
        self.previous_timestamp = timestamp

        # 计算左右轮的平均线速度
        robot_liner_x_ = (left_velocity + right_velocity) / 2.0

        # 计算偏航角变化（delta_yaw）
        delta_yaw = (right_velocity - left_velocity) * dt / C5_DISTANCE_WHEELS

        # 计算位置变化（delta_x, delta_y）
        delta_x = (robot_liner_x_ * math.cos(robot_status['position_yaw_']) - robot_liner_y_ * math.sin(robot_status['position_yaw_'])) * dt
        delta_y = (robot_liner_x_ * math.sin(robot_status['position_yaw_']) + robot_liner_y_ * math.cos(robot_status['position_yaw_'])) * dt

        # 更新机器人位置
        robot_status['position_x_'] += delta_x
        robot_status['position_y_'] += delta_y
        robot_status['position_yaw_'] += delta_yaw

        if robot_status['position_yaw_']>pi_value:
            robot_status['position_yaw_']=robot_status['position_yaw_']-2*pi_value
        if robot_status['position_yaw_']<-pi_value:
            robot_status['position_yaw_']=robot_status['position_yaw_']+2*pi_value

        # 计算四元数表示旋转
        qx, qy, qz, qw = self.euler_to_quaternion(robot_status['position_yaw_'])

        # 将里程计数据保存为TUM格式
        timestamp_sec = int(timestamp)
        timestamp_nsec = int((timestamp - timestamp_sec) * 1e9)
        timestamp_nsec_str = f"{timestamp_nsec:09d}"
        #self.odometry_file.write(f"{timestamp_sec}.{timestamp_nsec_str} {robot_status['position_x_']} {robot_status['position_y_']} 0.0 {qx} {qy} {qz} {qw}\n")
        #self.odometry_file.flush()

        # 发布机器人位姿
        self.publish_pose(robot_status['position_x_'], robot_status['position_y_'], robot_status['position_yaw_'], timestamp)
        #print(f"轮速里程计输出实时位置为: {robot_status['position_x_']:.2f}, {robot_status['position_y_']:.2f} {robot_status['position_yaw_']:.2f}")

    @staticmethod
    def euler_to_quaternion(yaw):
        """
        将欧拉角（yaw）转换为四元数。
        假设只有绕z轴的旋转（2D平面旋转）。
        """
        qw = math.cos(yaw / 2.0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        return qx, qy, qz, qw

    def publish_pose(self, x, y, yaw, timestamp):
        """
        发布机器人的位姿信息（Position + Quaternion）。
        """
        # 创建 PoseStamped 消息
        pose_msg = PoseStamped()

        # 创建时间戳消息
        time_msg = Time()
        time_msg.sec = int(timestamp)  # 时间戳的秒部分
        time_msg.nanosec = int((timestamp - int(timestamp)) * 1e9)  # 时间戳的小数部分转纳秒

        # 设置时间戳
        pose_msg.header.stamp = time_msg

        pose_msg.header.frame_id = 'map'  # 可以根据需要修改坐标系

        # 设置位置信息
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0  # 假设机器人只在平面上运动

        # 设置四元数表示的朝向
        qx, qy, qz, qw = self.euler_to_quaternion(yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # 发布消息
        self.pose_publisher.publish(pose_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    velocity_logger = VelocityLogger()

    velocity_logger.get_logger().info("Velocity logger node with odometry computation and pose publishing has started.")
    
    try:
        rclpy.spin(velocity_logger)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
