import rclpy
import rclpy.logging
from robot_interface.robot_controller import RobotController
import threading
from backend.app import RobotAPI

def main():
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
