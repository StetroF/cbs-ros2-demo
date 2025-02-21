
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
import json
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
        robot1,robot2,conflict_point = self.validate_paths(best)
        if conflict_point == -1:
            print('No conflict found')
        print(f'机器人1: {robot1}, 机器人2: {robot2}, 冲突点: {conflict_point}')
        
        robot1_path = self.st_planner.plan(self.robots[self.robots.index(robot1)],conflict_point)
        robot2_path = self.st_planner.plan(self.robots[self.robots.index(robot2)],conflict_point)
        
        # If there is not conflict, validate_paths returns (None, None, -1)
        # if agent_i is None:
        #     results.append((self.reformat(self.agents, best.solution),))
        #     return
    def validate_paths(self,node:CTNode):
        for robot1,robot2 in combinations(node.solution,2):
            pprint(f'Robot1: {node.solution[robot1]}, Robot2: {node.solution[robot2]}')
            ###TODO 不应该计算time_of_conflict，而是返回点，时间
            conflict_point = self.conflict_find(node.solution[robot1],node.solution[robot2],self.robots[self.robots.index(robot1)],self.robots[self.robots.index(robot2)])
            print(f'conflict_point: {conflict_point}')
            if conflict_point == -1:
                continue
            
            return robot1,robot2,conflict_point
        return None,None,-1
    ####冲突搜索函数，输入两个路径，返回发生冲突时，距离最近的那个点
    def conflict_find(self, solution1: list[str, float], solution2: list[str, float], robot1:RobotState, robot2:RobotState):
        
        origin_path1 = self.st_planner.get_path_by_solution(solution1)
        print(f'origin_path1: {origin_path1}')
        origin_path2 = self.st_planner.get_path_by_solution(solution2)
        downsample_path1 = self.st_planner.downsample_path(robot1, solution1)
        downsample_path2 = self.st_planner.downsample_path(robot2, solution2)
        

        ####找到两个path中，时间上有交集的点
        def find_time_intersection(path1:list[list,float], path2:list[list,float]):
            time1 = set(map(lambda point: point[1], path1))###时间集
            time2 = set(map(lambda point: point[1], path2))
            intersect_time = time1.intersection(time2)
            return intersect_time
        intersect_time = find_time_intersection(downsample_path1, downsample_path2)
        print(f'intersect_time: {intersect_time}')  
        if not intersect_time:
            return -1  # 如果没有交集，返回-1
        

        for time in intersect_time:
            point1 = downsample_path1[downsample_path1.index(next(filter(lambda point: point[1] == time, downsample_path1)))]
            point2 = downsample_path2[downsample_path2.index(next(filter(lambda point: point[1] == time, downsample_path2)))]
            distance = math.sqrt((point1[0][0]-point2[0][0])**2 + (point1[0][1]-point2[0][1])**2)
            ##TODO distance的阈值应该是两个robot_radius的和
            if distance < 0.5:
                ###当距离小于阈值，说明发生了碰撞，则找到path中距离robot最近的点，返回
                print(f'似乎发生了碰撞，此时poin1: {point1}, point2: {point2}')
                nearest_point = min(origin_path1,key = lambda point: math.sqrt((point[0][0]-point1[0][0])**2 + (point[0][1]-point1[0][1])**2))
                node = solution1[origin_path1.index(nearest_point)]
                return node  # 如果找到小于0.5的距离，返回时间戳
        return None
    
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