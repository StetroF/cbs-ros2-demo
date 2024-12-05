from pydantic import BaseModel
from typing import List,Any

class Point(BaseModel):
    x: float
    y: float
    z: float = None
class pathRequest(BaseModel):
    robot_id: str
    goal: Point
    
class BaseResponse(BaseModel):
    status: bool
    message: str = None
    
class MapResponse(BaseModel):
    map_name:str
    map_data:Any