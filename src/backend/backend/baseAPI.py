from pydantic import BaseModel
from typing import List,Any,Literal,Optional

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
    
    
class Edges(BaseModel):
    edgeId: str
    allowBackward: bool
    requireApply: bool
    topoType: int
    edgeType: Literal['line','curve','bcurve'] ##只允许接收这三个值
    width: float| int
    dockingType: int 
    avoidanceMode: int 
    safetyZoneType: int 
    navigationMode: int
    velocity: int | float
    startPoint: Point
    endPoint: Point
    elementId: str
    controlPoints: Optional[List[Point]] =None ##非必须参数

class Node(BaseModel):
    node_id :str
    x: float
    y: float
    topo_type: int

    
class SaveMapRequest(BaseModel):
    edges: List[Edges] = None
    map_name: str
    nodes: Optional[List[Node]] = None
    element_info : Optional[list] = None
