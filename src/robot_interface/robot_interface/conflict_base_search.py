
"""
conflict-base-search 冲突路径搜索算法入口
作用:
    根据接收到的若干条path以及robot，预测在每个时间点的位置，根据预测位置做碰撞检测，然后协商路径
"""
from heapq import heappush, heappop
from robot_interface.space_time_dijk import Planner
from robot_interface.base_class import RobotState
from pprint import pprint
from itertools import combinations
import os,sys
from robot_interface.constrain_tree import CTNode
from robot_interface.constraints import Constraints
import math
import multiprocessing as mp
import numpy as np
class ConflictBaseSearch:
    def __init__(self):
        self.st_planner:Planner  = Planner()##包含时间的dijk搜索算法
        self.agents = []  ###机器人列表，这里用agent表示一个机器人
        self.robots:list[RobotState] = []
        self.max_iter_time = 1000
    def add_robot(self, robot):
        self.robots.append(robot)
    def add_agent(self, agent):
        self.agents.append(agent)
        
        
    def plan(self, max_process=10, debug=False):
        # solutions = []
        # for robot in self.robots:
        #     solutions.append(self.st_planner.plan(robot))  ##调用时间的dijk搜索算法，得到路径和时间戳
        
        _iter = 0
        solutions =dict({robot.robot_name:self.st_planner.plan(robot) for robot in self.robots})
        manager = mp.Manager()
        constrains = Constraints()
        open = []
        node = CTNode(constrains,solution=solutions)
        open.append(node)
        # for robot_name,path in solutions.items():
        #     pprint(f'Path for {robot_name}: {path}')
        results = manager.list([])
        self.search_node(node,results)
        return
        while open and iter_ < self.max_iter_time:
            iter_ += 1

            results = manager.list([])

            processes = []

            # Default to 10 processes maximum
            for _ in range(max_process if len(open) > max_process else len(open)):
                p = mp.Process(target=self.search_node, args=[heappop(open), results])
                processes.append(p)
                p.start()

            for p in processes:
                p.join()

            for result in results:
                if len(result) == 1:
                    if debug:
                        print('CBS_MAPF: Paths found after about {0} iterations'.format(4 * iter_))
                    return result[0]
                if result[0]:
                    heappush(open, result[0])
                if result[1]:
                    heappush(open, result[1])

        if debug:
            print('CBS-MAPF: Open set is empty, no paths found.')
        return np.array([])
    def search_node(self,best:CTNode,results):
        # agent_i, agent_j, time_of_conflict = self.validate_paths(best)
        self.validate_paths(best)
        # If there is not conflict, validate_paths returns (None, None, -1)
        # if agent_i is None:
        #     results.append((self.reformat(self.agents, best.solution),))
        #     return
    def validate_paths(self,node:CTNode):
        for robot1,robot2 in combinations(node.solution,2):
            pprint(f'Robot1: {node.solution[robot1]}, Robot2: {node.solution[robot2]}')
            time_of_conflict = self.safe_distance(node.solution[robot1],node.solution[robot2],robot1,robot2)
            if time_of_conflict == -1:
                continue
            
            return robot1,robot2,time_of_conflict
        return None,None,-1
    def safe_distance(self, solution1: list[str, float], solution2: list[str, float], robot1, robot2):
        downsample_path1 = self.st_planner.downsample_path(robot1, solution1)
        downsample_path2 = self.st_planner.downsample_path(robot2, solution2)

        def find_time_intersection(path1, path2):
            # 提取时间
            times1 = [point[1] for point in path1]  
            times2 = [point[1] for point in path2]

            # 计算交集
            intersection = set(times1) & set(times2)
            return sorted(intersection)  # 返回排序后的交集结果

        intersect_time = find_time_intersection(downsample_path1, downsample_path2)
        
        if not intersect_time:
            return -1  # 如果没有交集，返回-1

        for time in intersect_time:
            # 找到交集时间对应的点
            point1 = next(point for point in downsample_path1 if point[1] == time)[0]  # 获取对应的点
            point2 = next(point for point in downsample_path2 if point[1] == time)[0]  # 获取对应的点

            # 计算两点之间的距离
            distance = math.hypot(point1[0] - point2[0], point1[1] - point2[1])
            
            if distance < 0.5:
                return time  # 返回小于0.5的距离，可以根据需求定制返回值

        return -1  # 如果没有找到小于0.5的距离，返回-1
    
def main():
    cbs = ConflictBaseSearch()
    robot1 = RobotState('robot1')
    robot1.pose = [3.0, 3.25, 1.57]  # 初始位置：[x, y, theta]
    robot1.velocity = [0.5, 0.5]  # 线速度和角速度
    robot1.current_goal = "P15"
    robot1.timestamp = 0.0  # 初始时间戳

    robot2 = RobotState('robot2')
    robot2.pose = [13.0,7.0, 0.00]  # 初始位置：[x, y, theta]
    robot2.velocity = [0.5, 0.5]  # 线速度和角速度
    robot2.current_goal = "P5"
    robot2.timestamp = 0.0  # 初始时间戳

    cbs.add_robot(robot1)
    cbs.add_robot(robot2)
    solutions = cbs.plan()
    
    
if __name__ == '__main__':
    main()