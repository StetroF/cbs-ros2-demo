from fastapi import APIRouter
import os
from backend.baseAPI import BaseResponse,MapResponse
from fastapi import APIRouter, HTTPException, Response

class MapRouter(APIRouter):
    def __init__(self,logger):
        super().__init__(prefix="/Map")
        
        self.init_routes()
        self.logger = logger
    def init_routes(self):        
        self.add_api_route('/GetMapImage',self.get_map,methods=['GET'])
        self.add_api_route('/GetMapData',self.get_map_data,methods=['GET'])
    def get_map(self):
        if os.path.exists("/home/x/map/history_map_id.txt"): 
            with open("/home/x/map/history_map_id.txt", "r") as f:
                map_id = f.read().strip()
                self.logger.info(f"Map id found: {map_id}")
        else:
            self.logger.error("No map id found")
            raise HTTPException(status_code=404, detail="No map id found")

        _path = f"/home/x/map/{map_id}/map.png"  # 只需要文件夹路径

            
        return Response(content=open(_path, "rb").read(), media_type="image/png")

    def get_map_data(self):
        if os.path.exists("/home/x/map/history_map_id.txt"): 
            with open("/home/x/map/history_map_id.txt", "r") as f:
                map_id = f.read().strip()
        else:
            self.logger.error("No map id found")
            raise HTTPException(status_code=404, detail="No map id found")

        _path = f"/home/x/map/{map_id}"  # 只需要文件夹路径
        map_contents = {}
        
        # 定义需要读取的文件及其对应的键
        files_to_read = {
            "map_json": os.path.join(_path, "map.json"),
            "map_edges": os.path.join(_path, "map_edges.json"),
            "node": os.path.join(_path, "node.json"),
            "elements_info": os.path.join(_path, "elements_info.json"),
        }
        
        for key, file_path in files_to_read.items():
            try:
                with open(file_path, "r") as f:
                    map_contents[key] = f.read()
            except FileNotFoundError:
                self.logger.error(f"File not found: {file_path}")
                map_contents[key] = None  # 或者根据需求处理缺失的文件

        map_contents["map_name"] = map_id

        # 返回 JSON 数据，不包括图像
        return map_contents