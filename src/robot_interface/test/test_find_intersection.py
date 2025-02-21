import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from threading import Thread, Lock
import time
import sys

def calculate_yaw(dx, dy):
    """计算偏航角（yaw）"""
    return np.arctan2(dy, dx)

def generate_bspline(start, end, control_points, step_size=0.02):
    """
    生成 B 样条曲线
    :param start: 起始点 [x, y]
    :param end: 终点 [x, y]
    :param control_points: 控制点列表 [[x1, y1], [x2, y2], ...]
    :param step_size: 步长，默认 0.02
    :return: 曲线点列表，每个点包含 [x, y, yaw]
    """
    points = np.vstack([start, control_points, end]).T

    # 计算 B 样条曲线
    tck, u = splprep(points, u=None, s=0.0, per=0)
    u_new = np.arange(0, 1.0 + step_size, step_size)
    x, y = splev(u_new, tck, der=0)

    # 计算导数（用于计算 yaw）
    dx, dy = splev(u_new, tck, der=1)
    yaw = calculate_yaw(dx, dy)

    curve_points = np.vstack([x, y, yaw]).T
    return curve_points

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')
        # 订阅 odometry 话题
        self.odom_sub = self.create_subscription(
            Odometry, '/tb0_0/odom', self.odom_callback, 10)
        
        # 初始化数据存储
        self.history_positions = []  # 历史位置
        self.current_position = None  # 当前位置
        self.lock = Lock()  # 线程锁

        # 生成 B 样条曲线
        self.start = np.array([2.0, 10.0])
        self.end = np.array([10,18.0])
        self.control_points = np.array([[4,10], [6,13.5], [8,15.5]])

        self.reference_path = generate_bspline(self.start, self.end, self.control_points)

    def odom_callback(self, msg: Odometry):
        """处理 odometry 数据"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 更新当前位置和历史轨迹
        with self.lock:
            self.current_position = (x, y)
            self.history_positions.append((x, y))
            if len(self.history_positions) > 500:
                self.history_positions.pop(0)

    def key_press(self, event):
        """处理键盘事件"""
        if event.key == 'enter':
            with self.lock:
                self.history_positions.clear()
                self.current_position = None

    def visualize(self):
        """实时可视化"""
        plt.ion()  # 开启交互模式
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Robot Trajectory and Reference Path')
        ax.grid(True)
        ax.axis('equal')

        # 绘制参考路径
        ref_path_x = self.reference_path[:, 0]
        ref_path_y = self.reference_path[:, 1]
        ax.plot(ref_path_x, ref_path_y, 'b-', label='Reference Path')

        # 初始化机器人轨迹
        robot_line, = ax.plot([], [], 'r-', label='Robot Trajectory')
        robot_point, = ax.plot([], [], 'ro', label='Current Position')
        ax.legend()

        # 连接键盘事件
        fig.canvas.mpl_connect('key_press_event', self.key_press)

        while rclpy.ok():
            # 获取当前数据
            with self.lock:
                if self.current_position is None:
                    time.sleep(0.1)
                    continue
                history_x = [p[0] for p in self.history_positions]
                history_y = [p[1] for p in self.history_positions]
                current_x, current_y = self.current_position

            # 更新机器人轨迹
            robot_line.set_xdata(history_x)
            robot_line.set_ydata(history_y)

            # 更新当前位置
            robot_point.set_xdata([current_x])
            robot_point.set_ydata([current_y])

            # 调整视图范围
            ax.relim()
            ax.autoscale_view()

            # 重绘图
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = OdomVisualizer()

    # 启动可视化线程
    viz_thread = Thread(target=node.visualize)
    viz_thread.start()

    # 运行 ROS 2 节点
    rclpy.spin(node)

    # 清理
    node.destroy_node()
    rclpy.shutdown()
    viz_thread.join()

if __name__ == '__main__':
    main()
