from pydantic import BaseModel
from typing import List
class Point(BaseModel):
    x: float
    y: float
    z: float = None
class pathRequest(BaseModel):
    robot_id: str
    goal: Point