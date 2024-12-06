import os,sys
current_dir = os.path.dirname(os.path.abspath(__file__))
util_path = os.path.abspath(os.path.join(current_dir, 'util'))
print(f'utilPath: {util_path}')
cbs_ros2_msgs_path = os.path.abspath(os.path.join(current_dir, '../../cbs_ros2_msgs'))
robot_interface_path = os.path.abspath(os.path.join(current_dir, '../../robot_interface'))
sys.path.append(util_path)
sys.path.append(cbs_ros2_msgs_path)
sys.path.append(robot_interface_path)
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
