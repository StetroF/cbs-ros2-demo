import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,Twist
from pynput import keyboard

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        # 创建一个 TwistStamped 消息发布者
        self.publisher_ = self.create_publisher(Twist, '/tb0_2/cmd_vel', 10)
        self.get_logger().info("Keyboard control node has started.")

        # 设置键盘监听器
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # 初始化 Twist 消息
        self.twist_msg = Twist()

    def on_press(self, key):
        try:
            if key.char == 'w':  # 向前
                self.twist_msg.linear.x = 0.5

                self.twist_msg.angular.z = 0.0
            elif key.char == 'a':  # 向左
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.5
            elif key.char == 's':  # 向后
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
            elif key.char == 'd':  # 向右
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = -0.5
        except AttributeError:
            # 特殊键（如Ctrl、Alt等）时跳过
            pass

        # 发布 TwistStamped 消息
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
