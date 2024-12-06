from fastapi import APIRouter,Request
import os,typing
from rclpy.logging import get_logger
from backend.baseAPI import BaseResponse,SaveMapRequest
import json
from pydantic import ValidationError

class MapEditRouter(APIRouter):
    def __init__(self,logger):
        super().__init__(prefix="/MapEdit")
        
        self.init_routes()
        self.logger = get_logger('MapEditRouter')
    def init_routes(self):
        self.add_api_route('/GetMapList',self.get_map_list,methods=['GET'])
        self.add_api_route('/SaveMap',self.save_map,methods=['POST'])
    
    async def save_map(self,request:SaveMapRequest):
        try:
            
            self.logger.info(f"Saving map: {request}")
            map_name = request.map_name
            edges = request.edges
            nodes = request.nodes
            ###1.修改写入edge
            _path = f"/home/x/map/{map_name}"  # 只需要文件夹路径
            math_edges_path = os.path.join(_path, "map_edges.json")
            # 保存 map.json 文件
            save_dict_data = {"map_edges":[]}
            for edge in edges:
                edge_dict = edge.model_dump()
                save_dict_data["map_edges"].append(edge_dict)
                # self.logger.info(f'edge dict: {edge_dict}')        
            self.logger.info(f"Saving map: {map_name}")
            # self.logger.info(f"Saving map: {save_dict_data}")
            # self.logger.info(f"Edges: {edges}")
            
            with open(math_edges_path,'w') as f:
                json.dump(save_dict_data,f)
                
            self.logger.info(f'写入地图数据...')

            ###2.修改写入node
            node_dict_data = {"node":[]}
            for node in nodes:
                node_dict_data.get("node").append(node.model_dump())
            
            with open(os.path.join(_path, "node.json"),'w') as f:
                json.dump(node_dict_data,f)
            self.logger.info(f'写入地图数据完成...')

            return BaseResponse(status=True, message="Save map success")
        except ValidationError as e:
            self.logger.error(f"Error while saving map: {e}")
            return BaseResponse(status=False, message="Save map failed")
    def get_map_list(self):
        map_list = []
        map_file_path = '/home/x/map'
        
        try:
            for entry in os.listdir(map_file_path):
                full_path = os.path.join(map_file_path, entry)
                if os.path.isdir(full_path):
                    # 将文件名和创建时间一起存储在元组中
                    map_list.append((entry, os.path.getctime(full_path)))
            
            # 按创建时间进行排序，新的在前
            map_list.sort(key=lambda x: x[1], reverse=True)
            
            # 提取排序后的文件名列表
            sorted_map_list = [entry[0] for entry in map_list]
            
            self.logger.info(f"Map list: {sorted_map_list}")
        except Exception as e:
            self.logger.error(f"Error while getting map list: {e}")
            return []
        
        return sorted_map_list
    
    def get_map_by_id(self):
        pass