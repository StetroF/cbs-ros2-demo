from pydantic import BaseModel
from typing import List,Any,Literal,Optional

class Point(BaseModel):
    x: float
    y: float
    node_id: str = None
class pathRequest(BaseModel):
    robot_id: str
    goal: Point
    
class BaseResponse(BaseModel):
    status: bool
    message: str = None
    
class MapResponse(BaseModel):
    map_name:str
    map_data:Any
    
    
class Edges(BaseModel):
    edgeId: str =None
    allowBackward: bool =None
    requireApply: bool=None
    topoType: int=None
    edgeType: Literal['line','curve','bcurve']=None ##只允许接收这三个值
    width: float| int=None
    dockingType: int =None
    avoidanceMode: int =None
    safetyZoneType: int =None
    navigationMode: int =None
    velocity: int | float =None
    startPoint: Point
    endPoint: Point
    elementId: str =None
    controlPoints: Optional[List[Point]] =None ##非必须参数

class Node(BaseModel):
    node_id :str
    x: float
    y: float
    topo_type: int = None

    
class SaveMapRequest(BaseModel):
    edges: Optional[List[Edges]] = None
    map_name: str
    nodes: Optional[List[Node]] = None
    element_info : Optional[list] = None
