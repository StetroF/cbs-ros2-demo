import pygame
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from functools import partial  # 用于创建带参数的回调函数

# 定义全局常量
MAP_PATH = '/home/x/map.png'  # 地图文件路径
MAP_WIDTH, MAP_HEIGHT = 800, 600  # 地图显示的宽度和高度

# 设置缩放比例，调整位置显示的大小
SCALE_X = 50  # 水平缩放比例
SCALE_Y = 50   # 垂直缩放比例

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        
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
        
        # 用于存储位置数据
        self.positions = dict()

    def odom_callback(self, msg: Odometry, robot_id: str):
        # 获取位置并存储
        position = msg.pose.pose.position
        self.positions[robot_id] = (position.x, position.y)

def draw_map(screen, map_surface, positions):
    # 显示地图
    screen.blit(map_surface, (0, 0))
    
    # 设置机器人标记的颜色和大小
    color = (255, 0, 0)  # 红色
    radius = 8  # 调整机器人标记的大小
    print(positions)
    # 在地图上显示每个机器人的位置
    for i, pos in positions.items():
        if pos is not None:
            x, y = pos
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
    pygame.init()

    # 设置屏幕
    screen = pygame.display.set_mode((MAP_WIDTH, MAP_HEIGHT))
    pygame.display.set_caption('Map Display with Robot Odom')

    # 加载地图
    map_surface = pygame.image.load(MAP_PATH)
    map_surface = pygame.transform.scale(map_surface, (MAP_WIDTH, MAP_HEIGHT))

    # 用于存储机器人的位置
    positions = [None] * 5

    # 主循环
    clock = pygame.time.Clock()
    while rclpy.ok():
        # 处理ROS回调
        rclpy.spin_once(odom_subscriber)
        
        # 事件处理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                pygame.quit()
                return
        
        # 清空屏幕
        screen.fill((255, 255, 255))
        
        # 绘制地图和机器人的位置
        draw_map(screen, map_surface, odom_subscriber.positions)
        
        # 更新屏幕
        pygame.display.flip()
        
        # 设置帧率
        clock.tick(30)

if __name__ == '__main__':
    main()
