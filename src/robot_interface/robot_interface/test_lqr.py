from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from lqr_controller import LQRController
import numpy as np
import rclpy
from line_generator import generate_reference_path, LineType, generate_bspline
from geometry_msgs.msg import Pose, Twist, Vector3
from threading import Thread
import time
import pygame
from pid_controller import PIDController
import math
from threading import Lock

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.odom_sub = self.create_subscription(Odometry, '/tb0_0/odom', self.odom_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/tb0_0/cmd_vel', 10)
        self.controller: LQRController = LQRController(self.get_logger())
        self.robot_state = None
        self.angular_pid = PIDController(1.0, 0.0, 0.1)  # 旋转控制器
        self.dest_path_pub = self.create_publisher(Path, '/dest_path', 10)  # 新增：发布目标路径
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)  # 新增：发布机器人路径
        self.robot_path = []  # 用于存储机器人的历史路径点

    def odom_callback(self, msg: Odometry):
        robot_state = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion(msg.pose.pose.orientation)[2]])
        self.robot_state = robot_state
        robot_velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        self.controller.update_param(robot_state, robot_velocity)
        self.robot_path.append(robot_state)  # 存储机器人的历史路径点

    def info(self, msg):
        self.get_logger().info(msg)

    def heading_to_target(self, target_pose):
        prev_time = self.get_clock().now()

        while True:
            current_time = self.get_clock().now()
            dt = (current_time - prev_time).nanoseconds / 1e9
            prev_time = current_time

            # 更新机器人当前位置
            current_pos = self.robot_state
            heding_angle = math.atan2(target_pose[1] - current_pos[1], target_pose[0] - current_pos[0])
            # 计算实时角度和距离差
            current_heading = current_pos[2]
            angle_diff = target_pose[2] - current_heading
            if abs(angle_diff) < 0.05:
                self.vel_pub.publish(Twist(linear=Vector3(x=0., y=0., z=0.), angular=Vector3(x=0., y=0., z=0.)))
                break
            # PID 控制角速度和线速度
            angular_vel = self.angular_pid.compute(angle_diff, dt)
            self.vel_pub.publish(Twist(linear=Vector3(x=0., y=0., z=0.), angular=Vector3(x=0., y=0., z=angular_vel)))
            time.sleep(0.05)

    def control(self, path):
        dist_thresh = 0.07
        while self.robot_state is None:
            self.get_logger().info("Waiting for robot state")
            time.sleep(1)
        dist_to_end_point = np.linalg.norm(path[-1][:2] - self.robot_state[:2])
        # self.info(f"Distance to end point: {dist_to_end_point}")
        target_vel = [0.0, 0.0]
        
        def lqr_control_loop(path):
            index = 0

            while dist_to_end_point >= dist_thresh:
                if index >= len(path):
                    break
                dist_to_point = np.linalg.norm(path[index][:2] - self.robot_state[:2])
                if dist_to_point <= dist_thresh:
                    self.info(f'已到达点{index}，距离:{dist_to_point}')
                    index += 1
                    continue
                control_input = self.controller.control(self.robot_state, path[index])
                self.info(f'跟踪index: [{index}] 实时控制速度:{control_input} 点距离:{dist_to_point}')
                # self.get_logger().info(f"Control input: {control_input}")
                self.vel_pub.publish(Twist(linear=Vector3(x=control_input[0], y=0., z=0.), angular=Vector3(x=0., y=0., z=control_input[1])))
                time.sleep(0.05)
            self.vel_pub.publish(Twist(linear=Vector3(x=0., y=0., z=0.), angular=Vector3(x=0., y=0., z=0.)))
            self.info("End of path")
        
        first_pose = path[0]
        reaching_path = self.adjust_path_if_needed(path)
        self.info(f'目标角度: {reaching_path[0]},机器人当前角度: {self.robot_state[2]}')
        self.heading_to_target(reaching_path[0])

        lqr_control_loop(reaching_path)
        self.heading_to_target(first_pose)
        lqr_control_loop(path)
        # self.publish_dest_path(reaching_path, path)  # 发布目标路径

    def adjust_path_if_needed(self, path):
        # 确保 path 是一个 numpy 数组
        path = np.array(path)
        
        start_point = path[0][:2]
        current_point = self.robot_state[:2]
        dist_to_start = np.linalg.norm(start_point - current_point)
        
        if dist_to_start > 0.1:
            direction = (start_point - current_point) / dist_to_start
            new_points = []
            step = 0.02
            distance = dist_to_start
            
            while distance > step:
                new_point = current_point + direction * step
                yaw = np.arctan2(direction[1], direction[0])
                new_points.append([new_point[0], new_point[1], yaw])
                current_point = new_point
                distance -= step
        
            # 插入新点到路径前面
            new_points = np.array(new_points)  # 将列表转换为 numpy 数组
            path = np.vstack((new_points))  # 使用 vstack 合并数组
        return path

    def publish_dest_path(self, reaching_path, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for point in np.vstack((reaching_path, path)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.orientation.w = 1.0  # 默认朝向
            path_msg.poses.append(pose_stamped)
        self.dest_path_pub.publish(path_msg)

    def publish_robot_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for point in self.robot_path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.orientation.w = 1.0  # 默认朝向
            path_msg.poses.append(pose_stamped)
        self.robot_path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = robot_controller()
    
    # Generate reference path
    start_p = np.array([2.0, 10.0])
    end_p = np.array([10.0, 18.0])
    control_points = np.array([[4, 10], [6, 13.5], [8, 15.5]])

    path_points = generate_bspline(start_p, end_p, control_points, step_size=0.02)
    logger = rclpy.logging.get_logger('robot_controller')
    logger.info(f"Generated path with {len(path_points)} points")
    for p in path_points:
        logger.info(f"Point: {p}")
        
    compute_path_thread = Thread(target=node.control, args=(path_points,))
    compute_path_thread.start()
    # 新增：启动可视化线程
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()