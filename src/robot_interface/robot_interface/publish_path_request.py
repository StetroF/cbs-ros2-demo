from rclpy.node import Node
from cbs_ros2_msgs.srv import PathRequest
import rclpy
class PublishPathRequest(Node):
    def __init__(self):
        super().__init__('publish_path_request')
        self.client = self.create_client(PathRequest, 'path_request')
        
        
    def test_publish_path_request(self):
        req = PathRequest.Request()
        req.robot_id = 'tb0_2'
        req.goal.x = 6.5
        req.goal.y = 8.0
        
        self.get_logger().info(f"Sending request: {req}")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        future = self.client.call_async(req)
        future.add_done_callback(self.callback_function)
    def callback_function(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
            
def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = PublishPathRequest()
    node.test_publish_path_request()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()