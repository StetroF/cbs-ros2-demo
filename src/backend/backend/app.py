from __future__ import annotations  # Python 3.7及以上支持推迟注解
from fastapi import FastAPI
from rclpy.node import Node
import rclpy
from cbs_ros2_msgs.srv import PathRequest
import os,sys,typing
from fastapi.middleware.cors import CORSMiddleware  # 导入 CORS 中间件
import uvicorn
from backend.routes.map import MapRouter
from backend.routes.robot import RobotRouter
from backend.routes import *
this_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(this_dir, '..'))
if typing.TYPE_CHECKING:
    from robot_interface.fleet import FleetManager

from backend.baseAPI import Point,pathRequest,BaseResponse

class RobotAPI():
    def __init__(self,fleet_manager:FleetManager):
        self.fleet_manager = fleet_manager
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # 允许所有域名
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
                
        self.path_req_client = self.fleet_manager.create_client(PathRequest, '/path_request')
        self.logger = self.fleet_manager.get_logger()
        self.init_routes()
        uvicorn.run(self.app, host='0.0.0.0', port=5000)
    def info(self,msg):
        self.fleet_manager.get_logger().info(f'{msg}')
    
    

    def init_routes(self):
        self.app.add_api_route('/move_to_goal', self.move_to_goal, methods=['POST'])
        map_router = MapRouter(self.logger)
        self.app.include_router(map_router,tags=["Map"])
        robot_router = RobotRouter(self.fleet_manager)
        self.app.include_router(robot_router,tags=["Robot"])
        map_edit_router = MapEditRouter(self.logger)
        self.app.include_router(map_edit_router,tags=["MapEdit"])
    async def move_to_goal(self,path: pathRequest):
        req = PathRequest.Request()
        req.robot_id = path.robot_id
        req.goal.x = path.goal.x
        req.goal.y = path.goal.y

        self.info(f"Sending path request to robot {path.robot_id} to move to {path.goal.x}, {path.goal.y}")
        future = self.path_req_client.call_async(req)
        # future.add_done_callback(self.on_path_response)
        return BaseResponse(status=True, message="Path request sent successfully")

    # def on_path_response(self, future):
    #     try:
    #         response = future.result()
    #         self.info(f"Path request response received from robot {response.robot_id}: {response.status}")
    #     except Exception as e:
    #         self.info(f"Error while processing path request response: {e}")

