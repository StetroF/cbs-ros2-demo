from __future__ import annotations  # Python 3.7及以上支持推迟注解
from fastapi import APIRouter
import os
import robot_interface
from fastapi import  WebSocket
import asyncio
import typing
if typing.TYPE_CHECKING:
    from robot_interface.robot_controller import RobotController
class RobotRouter(APIRouter):
    def __init__(self,robot_controller:RobotController):
        super().__init__(prefix="/Robot")
        self.robot_controller = robot_controller
        
        self.init_routes()
        
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
                robot_poses = self.robot_controller.get_robot_poses()
                await asyncio.sleep(0.3)
                if not robot_poses:
                    self.error("No robot poses available")
                    continue
                await websocket.send_json(robot_poses)
                
            except RuntimeError as e:
                self.error(f'连接超时，错误原因: {e}')
                break
            except Exception as e:
                self.error(f'订阅机器人位置时发生位置错误: {e}')
                break