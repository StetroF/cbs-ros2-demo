from fastapi import FastAPI
from rclpy.node import Node
import rclpy
from cbs_ros2_msgs.srv import PathRequest
import os,sys
from fastapi.middleware.cors import CORSMiddleware  # 导入 CORS 中间件
import uvicorn
this_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(this_dir, '..'))

from backend.baseAPI import Point,pathRequest

class RobotAPI():
    def __init__(self,robot_controller):
        self.robot_controller = robot_controller
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # 允许所有域名
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
                
        self.path_req_client = self.robot_controller.create_client(PathRequest, '/path_request')
        self.init_routes()
        uvicorn.run(self.app, host='0.0.0.0', port=5000)
    def info(self,msg):
        self.robot_controller.get_logger().info(f'{msg}')
    def init_routes(self):
        self.app.add_api_route('/move_to_goal', self.move_to_goal, methods=['POST'])

    def move_to_goal(self,path: pathRequest):
        req = PathRequest.Request()
        req.robot_id = path.robot_id
        req.goal.x = path.goal.x
        req.goal.y = path.goal.y

        self.info(f"Sending path request to robot {path.robot_id} to move to {path.goal.x}, {path.goal.y}")
        future = self.path_req_client.call_async(req)
        rclpy.spin_until_future_complete(self.robot_controller, future)
        if future.result() is not None:
            self.info(f"Path request to robot {path.robot_id} to move to {path.goal.x}, {path.goal.y} sent successfully")
            return {"status": "success"}
        else:
            self.info(f"Failed to send path request to robot {path.robot_id} to move to {path.goal.x}, {path.goal.y}")
            return {"status": "failed"}