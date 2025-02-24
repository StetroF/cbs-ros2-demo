"""
这是一个pathSearch,用于读取node和edge,获取路径以及用于规划的类
"""
import os
import json
import networkx as nx
import matplotlib.pyplot as plt
from pprint import pprint
from typing import List, Tuple,Dict
import heapq
import math
import sys
from itertools import combinations
curr_file_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(curr_file_path, '..'))
from robot_interface.base_class import RobotState
class Node:
    def __init__(self, node_id, point=[0, 0]):
        self.node_id = node_id
        self.neighbours: List[Tuple['Node', int]] = []  # Store (neighbour, cost) pairs
        self.point = point

    def add_neighbour(self, neighbour: 'Node', cost):
        self.neighbours.append((neighbour, cost))
    def __eq__(self, obj):
        if isinstance(obj, Node):
            return self.node_id == obj.node_id
        elif isinstance(obj, str):  # Allow comparison with node_id string
            return self.node_id == obj
        return False

    def __hash__(self):
        return hash(self.node_id)

    def __repr__(self):
        return str(self.node_id)
    def __lt__(self, other):
        return self.node_id < other.node_id
    
class Planner:
    def __init__(self):
        self.nodes:list[Node] = []
        self.init_nodes()
        self.build_graph()
    def get_node_pose(self,node_id):
        for node in self.nodes:
            if node.node_id == node_id:
                return node.point
    def init_nodes(self):
        map_id = ""
        if os.path.exists("/home/x/map/history_map_id.txt"):
            with open("/home/x/map/history_map_id.txt", "r") as f:
                map_id = f.read().strip()
        _path = f"/home/x/map/{map_id}"

        nodes_path = os.path.join(_path, 'node.json')
        edge_path = os.path.join(_path, 'map_edges.json')

        # Load all nodes
        with open(nodes_path, 'r') as f:
            nodes_data = json.load(f)
        nodes = nodes_data['node']
        for node in nodes:
            self.nodes.append(Node(node_id=node.get('node_id'), point=[node.get('x'), node.get('y')]))
            # self.nodes.append(Node(node_id=node.get('node_id'), point=[round(node.get('x'), 8), round(node.get('y'), 8)]))
        # Find neighbours from edges
        with open(edge_path, 'r') as f:
            edges_data = json.load(f)
        import time
        before = time.time()
        for node in self.nodes:
            for edge in edges_data['map_edges']:
                edge_start_point = list(edge.get('startPoint').values())
                edge_end_point = list(edge.get('endPoint').values())

                edge_type = edge.get('edgeType')
                if edge_type == 'bcurve':
                    print(f'曲线类型: {edge_type}')

                    print(f'b样条曲线数据:{edge}')
                #     print(f'起始点: {edge_start_point}, 终止点: {edge_end_point}')
                dist_to_start = ((node.point[0] - edge_start_point[0]) ** 2 + (node.point[1] - edge_start_point[1]) ** 2) ** 0.5
                dist_to_end = ((node.point[0] - edge_end_point[0]) ** 2 + (node.point[1] - edge_end_point[1]) ** 2) ** 0.5
                edge_cost = math.sqrt(pow(edge_start_point[0] - edge_end_point[0],2)+ pow(edge_start_point[1] - edge_end_point[1],2))
                edge_cost = round(edge_cost, 4)
                if dist_to_start<0.05:
                    for neighbour in self.nodes:
                        neighbour_to_end = ((neighbour.point[0] - edge_end_point[0]) ** 2 + (neighbour.point[1] - edge_end_point[1]) ** 2) ** 0.5
                        if neighbour_to_end < 0.05:
                            if edge_type == 'bcurve':
                                print('找到匹配b样条曲线的')
                            node.add_neighbour(neighbour, edge_cost)  # Assuming cost is 1 for simplicity
                            break
                if dist_to_end<0.05:
                    for neighbour in self.nodes:
                        neighbour_to_start = ((neighbour.point[0] - edge_start_point[0]) ** 2 + (neighbour.point[1] - edge_start_point[1]) ** 2) ** 0.5
                        if neighbour_to_start<0.05:
                            if edge_type == 'bcurve':
                                print('找到匹配b样条曲线的')
                            node.add_neighbour(neighbour, edge_cost)  # Assuming cost is 1 for simplicity
                            break
        using_time = time.time() - before
    def build_graph(self):
        """Build a NetworkX graph using the nodes."""
        self.G = nx.Graph()
        for node in self.nodes:
            self.G.add_node(node.node_id, pos=node.point)
            for neighbour, cost in node.neighbours:
                self.G.add_edge(node.node_id, neighbour.node_id, weight=cost)
    ###下采样path，根据输入的path(point,time),返回根据时间步长下采样后的path
    """
    作用：下采样path，根据输入的path(point,time),返回根据时间步长下采样后的path
    参数：
        robot:提供机器人一开始的角度信息
        path:输入的path(point,time),格式为[(node_id,time),(node_id,time),...]
        time_step:时间步长，默认为1.0s
    返回：
        results:下采样后的path,格式为[(point,1.0),(point,2.0),...]
    """
    def downsample_path(self,robot:RobotState, path: list[tuple], time_step=1.0):
        
        results = []
        last_target_angle=0.0
        for point_start, point_end in zip(path, path[1:]):
            # 计算时间数组
            time_diff = abs(point_end[1] - point_start[1])
            time_step_num = math.floor(time_diff / time_step)
            time_array = [point_start[1] + i * time_step for i in range(time_step_num + 1)]
            # time_array.append(round(point_end[1]))

            # 根据时间数组预测机器人在整条路径中，每隔1s的位置
            start_node = self.nodes[self.nodes.index(point_start[0])]
            end_node = self.nodes[self.nodes.index(point_end[0])]

            dist = math.hypot(end_node.point[0] - start_node.point[0], end_node.point[1] - start_node.point[1])
            x_dist = end_node.point[0] - start_node.point[0]
            y_dist = end_node.point[1] - start_node.point[1]
            # 计算机器人的角度变化
            target_angle = math.atan2(end_node.point[1] - start_node.point[1], end_node.point[0] - start_node.point[0])

            print(f'起始点: {start_node.point}, 终止点: {end_node.point}, 距离: {dist}, 目标角度: {target_angle},')
            # rotate_time_cost = 0
            for time in time_array:
                delta_x = x_dist * time_array.index(time) *time_step /time_diff
                delta_y = y_dist * time_array.index(time) *time_step /time_diff
                x = start_node.point[0] + delta_x
                y = start_node.point[1] + delta_y
                results.append(([round(x, 4), round(y, 4)], time))
                # rotate_time_cost += abs(angle_diff_to_start) / robot.velocity[1]
                # num_step= int(time_diff/time_step)
                # for i in range(num_step):
                #     t = i*time_step
                #     x = start_node.point[0] + t / time_diff * x_dist
                #     y = start_node.point[1] + t / time_diff * y_dist
                #     results.append(([round(x, 4), round(y, 4)], time+t))


    
        return results
    ###根据输入的solution(包含n个node_id),输出坐标
    def get_path_by_solution(self,solution:list[str,float]):
        
        path = [[self.nodes[self.nodes.index(node_id)].point,time] for [node_id,time] in solution]
        return path
    """
    space-time-star 算法
    1.相对于astar,会根据输入robot的速度，计算机器人到达每个节点的预期时间
    TODO 2.根据dynamic_obstacle，以及semi_dynamic_obstacle(其他robot到达目标点后的约束)
    args:
        dynamic_obstacle: 其他机器人到达目标点后的约束，格式为[(node_id,time),(node_id,time),...]
        leavt_time: 机器人离开目标点的时间，默认为3s
    """
    def plan(self, robot: RobotState = None, dynamic_obstacle: list[str,float] = None,max_iter_time=500,leave_time=3):
        if robot is None:
            robot = RobotState("tb0_1")
            robot.pose = [3.0, 3.25, 1.57]  # 初始位置：[x, y, theta]
            robot.velocity = [0.5, 0.5]  # 线速度和角速度
            robot.current_goal = "P15"
            robot.timestamp = 0.0  # 初始时间戳
        # print(f'当前机器人位置：{robot.pose}')
        # 找到距离机器人当前位置最近的节点
        start_node = min(self.nodes, key=lambda x: ((x.point[0] - robot.pose[0]) ** 2 + (x.point[1] - robot.pose[1]) ** 2) ** 0.5)
        target_node_id = robot.current_goal
        end_node = self.nodes[self.nodes.index(target_node_id)]
        
        # 计算机器人到达起始节点的时间成本
        dist_to_start = ((start_node.point[0] - robot.pose[0]) ** 2 + (start_node.point[1] - robot.pose[1]) ** 2) ** 0.5
        heading_angle = math.atan2(start_node.point[1] - robot.pose[1], start_node.point[0] - robot.pose[0])
        angle_diff_to_start = heading_angle - robot.pose[2]

        linear_time_cost = dist_to_start / robot.velocity[0]
        angular_time_cost = abs(angle_diff_to_start) / robot.velocity[1]
        start_time_cost = round(linear_time_cost + angular_time_cost)
        

        start_node_reach_time = robot.timestamp + start_time_cost
        
        # Dijkstra算法，优先队列中加入时间成本
        queue = []  # 优先队列（最小堆）
        heapq.heappush(queue, (start_node_reach_time, start_node, 0, heading_angle))  # (成本, 节点, 到达时间，到达这个点的预期角度)
        visited = set()  # 用于避免重复访问节点
        came_from = {}  # 用于路径重构
        costs = {node: float('inf') for node in self.nodes}
        costs[start_node] = 0
        reach_times = {node: float('inf') for node in self.nodes}  # 跟踪每个节点的到达时间
        reach_times[start_node] = start_node_reach_time  # 


        _iter = 0
        while queue and _iter<max_iter_time:
            _iter+=1
            current_time, current_node, current_cost, current_angle = heapq.heappop(queue)
            # print(f'当前迭代次数：{_iter},当前节点：{current_node},当前时间：{current_time},当前角度：{current_angle}')
            ###把当前点放入到探索队列，用于多机器人规划的约束计算
            heapq.heappush(queue, (current_time+3, current_node, current_cost, current_angle))  # 重新放回队列，以便下次访问


            # 如果目标节点被到达，重构路径
            if current_node == end_node:
                path = []
                while current_node:
                    path.append((current_node.node_id, reach_times[current_node]))  # 添加节点ID和到达时间
                    current_node = came_from.get(current_node)
                print(f'路径规划完成，路径长度：{len(path)},迭代次数：{_iter}')
                return path[::-1]  # 返回正确顺序的路径

            # 更新成本，并将邻居节点加入队列
            # 最小堆利用时间成本排序
            for neighbour, cost in current_node.neighbours:
                dist_to_neighbour = ((neighbour.point[0] - current_node.point[0]) ** 2 + (neighbour.point[1] - current_node.point[1]) ** 2) ** 0.5
                heading_angle_to_neighbour = math.atan2(neighbour.point[1] - current_node.point[1], neighbour.point[0] - current_node.point[0])
                angle_diff_to_neighbour = heading_angle_to_neighbour - current_angle
    
                linear_time_cost = dist_to_neighbour / robot.velocity[0]
                angular_time_cost = abs(angle_diff_to_neighbour) / robot.velocity[1]
                total_time_cost = round(linear_time_cost + angular_time_cost)

                new_time = current_time + total_time_cost
                new_cost = current_cost + cost
                if dynamic_obstacle:
                    # print(f'neighbour: {neighbour.node_id}, time: {new_time}, cost: {new_cost}')
                    if neighbour.node_id == dynamic_obstacle[0] and new_time == dynamic_obstacle[1]:
                        # costs[neighbour] = new_cost
                        came_from[neighbour] = neighbour
                        # reach_times[neighbour] = current_time+3  # 记录到达邻居节点的时间
                        heapq.heappush(queue, (current_time+3, neighbour, 0, heading_angle_to_neighbour))
                        continue

                if new_time < reach_times[neighbour]:
                    costs[neighbour] = new_cost
                    came_from[neighbour] = current_node
                    reach_times[neighbour] = new_time  # 记录到达邻居节点的时间
                    heapq.heappush(queue, (new_time, neighbour, 0, heading_angle_to_neighbour))
        print(f'没有找到路径!')
        return []  # 如果没有找到路径，返回空路径
    
    def draw_graph(self):
        """绘制图形，仅显示前四个字符的节点ID."""
        pos = nx.get_node_attributes(self.G, 'pos')  # 获取节点位置
        weights = nx.get_edge_attributes(self.G, 'weight')  # 获取边的权重

        # 获取节点ID并取前四个字符
        labels = {node_id: node_id[:4] for node_id in self.G.nodes()}

        # 绘制节点
        nx.draw(self.G, pos, labels=labels, node_color='lightblue', node_size=500, font_size=10)

        # 绘制边的权重
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=weights)

        plt.title("图的展示")
        plt.show()

    def get_graph(self):
        return self.G

def path_search(start_nodes: List[Node], end_node: Node, has_search: List[Node], 
                search_result: List[List[Node]]):
    """Find all paths between start nodes and the end node."""
    for node in start_nodes:
        if node in has_search:
            continue
        has_search.append(node)

        if node == end_node:
            search_result.append(list(has_search))  # Save found path
        else:
            for neighbour, _ in node.neighbours:  # Explore neighbours
                path_search([neighbour], end_node, has_search, search_result)

        has_search.pop()  # Backtrack to explore other paths
    return search_result

def find_shortest_path(G, start_node_id, end_node_id):
    """Find the shortest path using Dijkstra's algorithm."""
    return nx.shortest_path(G, source=start_node_id, target=end_node_id, weight='weight')

def visualize_paths(G, search_result, shortest_path):
    """Visualize the graph and highlight paths."""
    pos = nx.spring_layout(G)  # Compute layout positions

    # Draw the basic graph
    nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=800, font_size=10, font_weight='bold')

    # Highlight all found paths in red
    for path in search_result:
        edges = [(path[i].node_id, path[i + 1].node_id) for i in range(len(path) - 1)]
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='red', width=1)

    # Highlight the shortest path in green
    shortest_edges = [(shortest_path[i], shortest_path[i + 1]) for i in range(len(shortest_path) - 1)]
    nx.draw_networkx_edges(G, pos, edgelist=shortest_edges, edge_color='green', width=3)

    plt.title("Path Search Visualization")
    plt.show()

# Sample usage
if __name__ == "__main__":
    path_tool = Planner()
    # path_tool.draw_graph()
    # Display available nodes
    print("Available nodes:")
    # for node in path_tool.nodes:
    #     print(f"{node.node_id} at {node.point}")

    path = path_tool.plan(dynamic_obstacle=['P5',20])
    print(f"Path: {path}")  
    robot = RobotState("robot1")
    robot.pose = [3.0, 3.25, 1.57]  # 初始位置：[x, y, theta]
    # dp_path = path_tool.downsample_path(robot, path)
    # print(f"DP Path: {dp_path}")
    # # Get user input for start and end nodes
    # start_node_id = 'P5'
    # end_node_id = 'P15'

    # # Find corresponding Node objects
    # start_nodes = [node for node in path_tool.nodes if node.node_id == start_node_id]
    # end_node = next((node for node in path_tool.nodes if node.node_id == end_node_id), None)

    # if not start_nodes:
    #     print(f"Start node '{start_node_id}' not found.")
    # elif end_node is None:
    #     print(f"End node '{end_node_id}' not found.")
    # else:
    #     search_result = path_search(start_nodes, end_node, [], [])
    #     print('Path search completed from {} to {}.'.format(start_node_id, end_node_id))
    #     print("All paths:", search_result)

    #     # Get the graph and find the shortest path
    #     G = path_tool.get_graph()
    #     shortest_path = find_shortest_path(G, start_nodes[0].node_id, end_node.node_id)
    #     print("Shortest path:", shortest_path)

    #     # Visualize the paths and highlight the shortest path
    #     visualize_paths(G, search_result, shortest_path)
