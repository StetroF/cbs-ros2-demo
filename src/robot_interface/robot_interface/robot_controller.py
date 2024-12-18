"""
机器人统一控制接口，订阅5个odom主题，并根据位置差控制机器人运动。(PID控制)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist
from functools import partial  # 用于创建带参数的回调函数
import sys
import os
import math


from robot_interface.transform import euler_from_quaternion
from robot_interface.pid_controller import PIDController
from robot_interface.base_class import RobotState,Pose,TaskState
from cbs_ros2_msgs.srv import PathRequest
import threading
from backend.app import RobotAPI
from datetime import datetime



class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.rate = self.create_rate(5)
        self.robots:dict[str,RobotState] = {}
        num_of_robots = 5
        self.vel_publishers = []
        
        
        for i in range (num_of_robots):
            self.robots[f'tb0_{i}'] = RobotState(f'tb0_{i}')
            # 订阅5个odom主题，并传递robot_id作为参数
            self.create_subscription(
                Odometry,
                f'/tb0_{i}/odom',
                partial(self.odom_callback, robot_id=f'tb0_{i}'),  # 使用partial传递robot_id
                10
            )
            self.vel_publishers.append(
                self.create_publisher(Twist, f'/tb0_{i}/cmd_vel', 10)
            )
        # PID 控制器参数
        self.angular_pid = PIDController(1.0, 0.0, 0.1)  # 旋转控制器
        self.linear_pid = PIDController(0.5, 0.0, 0.05)  # 平移控制器
        self.path_request_server = self.create_service(PathRequest, 'path_request', self.path_request_callback)


    def get_robot_poses(self)->dict[str,RobotState]:
        return self.robots
    def path_request_callback(self, request, response):
        move_thread = threading.Thread(target=self.move_to_goal, args=(request,))
        move_thread.start()
        
        response.success = True
        return response

    def odom_callback(self, msg: Odometry, robot_id: str):
        # 获取位置并存储
        position = msg.pose.pose.position
        r, p, yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.robots[robot_id].pose = [position.x, position.y, yaw]
        
        # 转换时间戳为 datetime 对象
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.robots[robot_id].timestamp = timestamp
        
        # 创建 datetime 对象并格式化输出
        # dt = datetime.fromtimestamp(timestamp)
        # time_str = dt.strftime("%H:%M:%S")  # 只显示小时、分钟和秒

        # self.info(f"Received odom for robot {robot_id}: {time_str}")
    def info(self,msg):
        self.get_logger().info(f'{msg}')
    def error(self,msg):
        self.get_logger().error(f'{msg}')
    def move_to_goal(self, path_request=None):
        robot_id = path_request.robot_id if path_request else 'tb0_3'  # 测试用默认值
        if robot_id not in self.robots:
            self.get_logger().error(f"Robot {robot_id} not found in positions")
            return

        vel_publisher = self.vel_publishers[int(robot_id[-1])]

        # 获取初始和目标位姿
        start_pos = self.robots.get(robot_id).pose
        while start_pos is None:
            rclpy.spin_once(self)
            start_pos = self.robots.get(robot_id).pose
        
        end_pos = (path_request.goal.x, path_request.goal.y) if path_request else (-2,5)

        # 计算初始角度和距离
        heding_angle = math.atan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        distance = math.sqrt((end_pos[1] - start_pos[1])**2 + (end_pos[0] - start_pos[0])**2)

        self.get_logger().info(f"Start position: {start_pos}, End position: {end_pos}, Heading angle: {heding_angle}, Distance: {distance}")

        twist = Twist()
        prev_time = self.get_clock().now()

        while True:
            current_time = self.get_clock().now()
            dt = (current_time - prev_time).nanoseconds / 1e9
            prev_time = current_time

            # 更新机器人当前位置
            current_pos = self.robots.get(robot_id).pose


            # 计算实时角度和距离差
            current_heading = current_pos[2]
            angle_diff = heding_angle - current_heading
            distance_diff = math.sqrt((end_pos[1] - current_pos[1])**2 + (end_pos[0] - current_pos[0])**2)

            # PID 控制角速度和线速度
            angular_vel = self.angular_pid.compute(angle_diff, dt)
            linear_vel = self.linear_pid.compute(distance_diff, dt)

            # 防止过快旋转和平移
            angular_vel = max(-1.0, min(1.0, angular_vel))  # 限制角速度在 [-1.0, 1.0]
            linear_vel = max(0.0, min(0.5, linear_vel))  # 限制线速度在 [0.0, 0.5]

            # 如果距离差足够小，停止运动
            if distance_diff < 0.05:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                vel_publisher.publish(twist)
                self.get_logger().info(f"Robot {robot_id} reached the goal!")
                break

            # 如果角度差较大，优先旋转调整角度
            if abs(angle_diff) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = angular_vel
            else:
                twist.linear.x = linear_vel
                twist.angular.z = angular_vel

            vel_publisher.publish(twist)
            self.get_logger().info(f"Moving robot {robot_id}: Angle diff {angle_diff:.4f}, Distance diff {distance_diff:.4f}")
            self.rate.sleep()

def main():
    # 初始化rclpy
    rclpy.init()

    # 创建节点
    robot_controller = RobotController()
    app_thread = threading.Thread(target=RobotAPI, args=(robot_controller,))
    app_thread.start()
    # 加载地图


    while rclpy.ok():
        # 处理ROS回调
        rclpy.spin_once(robot_controller)
        


if __name__ == '__main__':
    main()
