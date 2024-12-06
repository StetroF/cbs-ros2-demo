from __future__ import annotations  # Python 3.7及以上支持推迟注解
from fastapi import APIRouter
import os
import robot_interface
from fastapi import  WebSocket
import asyncio
import typing
from rclpy.logging import get_logger
if typing.TYPE_CHECKING:
    from robot_interface.robot_controller import RobotController
from robot_interface.base_class import RobotState
class RobotRouter(APIRouter):
    def __init__(self,robot_controller:RobotController):
        super().__init__(prefix="/Robot")
        self.robot_controller = robot_controller
        
        self.init_routes()
        self.logger = get_logger('robot_router')
    def init_routes(self):
        self.add_api_websocket_route('/robot_pose_websocket',self.robot_pose_websocket)
    def info(self,msg):
        self.robot_controller.info(msg)
    def error(self,msg):
        self.robot_controller.error(msg)
    async def robot_pose_websocket(self,websocket: WebSocket):
        await websocket.accept()
        
        while True:
            try:
                robot_poses: dict[str, RobotState] = self.robot_controller.get_robot_poses()
                json_robot_poses = {}
                
                for robot in robot_poses:
                    json_robot_poses[robot] = robot_poses[robot].pose
                
                await asyncio.sleep(0.3)
                
                if not robot_poses:
                    self.error("No robot poses available")
                    continue

                await websocket.send_json(json_robot_poses)
            except Exception as e:
                self.error(f"An error occurred: {e}")
            except RuntimeError as e:
                self.error(f'连接超时，错误原因: {e}')
                break
            except Exception as e:
                self.error(f'订阅机器人位置时发生位置错误: {e}')
                break