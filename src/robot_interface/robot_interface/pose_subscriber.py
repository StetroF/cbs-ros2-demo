import pygame
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist
from functools import partial  # 用于创建带参数的回调函数
import sys
import os
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
util_path = os.path.abspath(os.path.join(current_dir, '../../util'))
cbs_ros2_msgs_path = os.path.abspath(os.path.join(current_dir, '../../cbs_ros2_msgs'))
sys.path.append(util_path)
sys.path.append(cbs_ros2_msgs_path)
from transform import euler_from_quaternion
from pid_controller import PIDController
from cbs_ros2_msgs.srv import PathRequest
import threading
# 定义全局常量
MAP_PATH = '/home/x/map.png'  # 地图文件路径
MAP_WIDTH, MAP_HEIGHT = 800, 600  # 地图显示的宽度和高度

# 设置缩放比例，调整位置显示的大小
SCALE_X = 50  # 水平缩放比例
SCALE_Y = 50   # 垂直缩放比例

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        
        self.robots = [f'tb0_{i}' for i in range(5)]
        # 订阅5个odom主题，并传递robot_id作为参数
        self.subscribers = [
            self.create_subscription(
                Odometry,
                f'/tb0_{i}/odom',
                partial(self.odom_callback, robot_id=f'tb0_{i}'),  # 使用partial传递robot_id
                10
            )
            for i in range(5)
        ]
        
        self.vel_publishers = [
            self.create_publisher(Twist, f'/tb0_{i}/cmd_vel', 10)
            for i in range(5)
        ]
        # 用于存储位置数据
        self.positions = dict()
        # PID 控制器参数
        self.angular_pid = PIDController(1.0, 0.0, 0.1)  # 旋转控制器
        self.linear_pid = PIDController(0.5, 0.0, 0.05)  # 平移控制器
        self.path_server = self.create_service(PathRequest, 'path_request', self.path_request_callback)
        
    def path_request_callback(self, request, response):
        move_thread = threading.Thread(target=self.move_to_goal, args=(request,))
        move_thread.start()
        
        # move_thread.join()
        response.success = True
        return response

    def odom_callback(self, msg: Odometry, robot_id: str):
        # 获取位置并存储
        position = msg.pose.pose.position
        r,p,yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.positions[robot_id] = (position.x, position.y,yaw)
    def info(self,msg):
        self.get_logger().info(f'{msg}')
    def move_to_goal(self, path_request=None):
        robot_id = path_request.robot_id if path_request else 'tb0_3'  # 测试用默认值
        if robot_id not in self.robots:
            self.get_logger().error(f"Robot {robot_id} not found in positions")
            return

        vel_publisher = self.vel_publishers[int(robot_id[-1])]

        # 获取初始和目标位姿
        start_pos = self.positions.get(robot_id)
        while start_pos is None:
            rclpy.spin_once(self)
            start_pos = self.positions.get(robot_id)
        
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
            current_pos = self.positions.get(robot_id)


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
def draw_map(screen, map_surface, positions):
    # 显示地图
    screen.blit(map_surface, (0, 0))
    
    # 设置机器人标记的颜色和大小
    color = (255, 0, 0)  # 红色
    radius = 8  # 调整机器人标记的大小
    # print(positions)
    # 在地图上显示每个机器人的位置
    for i, pos in positions.items():
        if pos is not None:
            x, y,yaw = pos
            # 将机器人的x, y坐标映射到屏幕像素坐标
            pixel_x = int(x * SCALE_X)  # 缩放比例，可调整
            pixel_y = -int(y * SCALE_Y)  # 缩放比例，可调整
            
            # 确保机器人标记的位置在屏幕范围内
            if 0 <= pixel_x < MAP_WIDTH and 0 <= pixel_y < MAP_HEIGHT:
                pygame.draw.circle(screen, color, (pixel_x, pixel_y), radius)

def main():
    # 初始化rclpy
    rclpy.init()

    # 创建节点
    odom_subscriber = OdomSubscriber()

    # 初始化pygame
    # pygame.init()

    # 设置屏幕
    # screen = pygame.display.set_mode((MAP_WIDTH, MAP_HEIGHT))
    # pygame.display.set_caption('Map Display with Robot Odom')

    # 加载地图
    map_surface = pygame.image.load(MAP_PATH)
    map_surface = pygame.transform.scale(map_surface, (MAP_WIDTH, MAP_HEIGHT))

    # 用于存储机器人的位置
    positions = [None] * 5
    # odom_subscriber.move_to_goal()
    # 主循环
    clock = pygame.time.Clock()
    while rclpy.ok():
        # 处理ROS回调
        rclpy.spin_once(odom_subscriber)
        
        # # 事件处理
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         rclpy.shutdown()
        #         pygame.quit()
        #         return
        
        # # 清空屏幕
        # screen.fill((255, 255, 255))
        
        # # 绘制地图和机器人的位置
        # draw_map(screen, map_surface, odom_subscriber.positions)
        
        # # 更新屏幕
        # pygame.display.flip()
        
        # # 设置帧率
        # clock.tick(30)

if __name__ == '__main__':
    main()
