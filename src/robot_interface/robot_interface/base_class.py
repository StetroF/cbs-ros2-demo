class TaskState:
    IDLE = 0
    WORKING = 1
    FINISHED = 2
class Pose:
    x: float
    y: float
    yaw: float

class RobotState:
    def __init__(self,robot_name):
        self.robot_name = robot_name
        self.pose =[]
        self.final_goal : str ###一个大任务的最终目标 c
        self.current_goal: str###大任务的其中某个当前目标点
        self.task_state: TaskState
        self.velocity:list
        self.timestamp: float
        self.robot_radius = 0.1 ##固定0.1
    def set_current_goal(self,goal):
        self.current_goal = goal
    
    