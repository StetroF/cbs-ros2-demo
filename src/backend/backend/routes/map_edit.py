from __future__ import annotations  # Python 3.7及以上支持推迟注解
from fastapi import APIRouter,Request
import os,typing
from rclpy.logging import get_logger
from backend.baseAPI import BaseResponse,SaveMapRequest
import json
if typing.TYPE_CHECKING:
    from robot_interface.robot_controller import RobotController

class MapEditRouter(APIRouter):
    def __init__(self,logger):
        super().__init__(prefix="/MapEdit")
        
        self.init_routes()
        self.logger = get_logger('MapEditRouter')
    def init_routes(self):
        self.add_api_route('/GetMapList',self.get_map_list,methods=['GET'])
        self.add_api_route('/SaveMap',self.save_map,methods=['POST'])
    def save_map(self, save_map_request: SaveMapRequest) -> BaseResponse:
        """保存地图数据，将节点信息和边信息写入文件"""
        # 记录请求信息，方便调试和跟踪
        self.logger.info(f"Save map request: {save_map_request}")

        # 定义保存节点和边数据的文件路径
        map_path_nodes = f"/home/x/map/{save_map_request.map_name}/node.json"
        map_path_edges = f"/home/x/map/{save_map_request.map_name}/map_edges.json"
        
        self.logger.info(f"Nodes map path: {map_path_nodes}")
        self.logger.info(f"Edges map path: {map_path_edges}")

        # 确保目录存在，如果没有则创建
        os.makedirs(os.path.dirname(map_path_nodes), exist_ok=True)

        # 处理节点数据
        if os.path.exists(map_path_nodes):
            # 文件存在，读取当前的节点数据
            with open(map_path_nodes, 'r') as f:
                nodes_data = json.load(f)
        else:
            # 文件不存在，初始化一个空的节点数据列表
            nodes_data = []

        # 将传入的节点数据转换为字典格式，方便后续操作
        incoming_nodes = {node.node_id: dict(node) for node in save_map_request.nodes}
        
        # 遍历传入的节点，检查是否已存在
        for node in save_map_request.nodes:
            existing_node = next((n for n in nodes_data if n['node_id'] == node.node_id), None)
            if existing_node:
                # 更新已存在节点的坐标和拓扑类型
                existing_node['x'] = node.x
                existing_node['y'] = node.y
                existing_node['topo_type'] = node.topo_type
            else:
                # 添加新节点
                nodes_data.append(dict(node))
        
        # 将更新后的节点数据写入 JSON 文件
        with open(map_path_nodes, 'w') as f:
            json.dump(nodes_data, f, indent=4)

        # 处理边数据
        edges_data = []

        # 将传入的边数据转换为字典格式
        if save_map_request.edges:
            edges_data = [dict(edge) for edge in save_map_request.edges]

        # 将边数据写入 JSON 文件
        with open(map_path_edges, 'w') as f:
            json.dump(edges_data, f, indent=4)

        # 返回操作成功的响应
        return BaseResponse(status=True, message="Save map success")
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