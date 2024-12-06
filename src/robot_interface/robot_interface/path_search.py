"""
这是一个pathSearch,用于读取node和edge,获取路径以及用于规划的类
"""
import os
import json
import networkx as nx
import matplotlib.pyplot as plt
from pprint import pprint
from typing import List, Tuple
import math
class Node:
    def __init__(self, node_id, point=[0, 0]):
        self.node_id = node_id
        self.neighbours: List[Tuple['Node', int]] = []  # Store (neighbour, cost) pairs
        self.point = point

    def add_neighbour(self, neighbour: 'Node', cost):
        self.neighbours.append((neighbour, cost))

    def __eq__(self, obj):
        return self.node_id == obj.node_id

    def __hash__(self):
        return hash(self.node_id)

    def __repr__(self):
        return str(self.node_id)

class PathPlanTool:
    def __init__(self):
        self.nodes:list[Node] = []
        self.init_nodes()
        self.build_graph()

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
    path_tool = PathPlanTool()
    path_tool.draw_graph()
    # Display available nodes
    print("Available nodes:")
    for node in path_tool.nodes:
        print(f"{node.node_id} at {node.point}")

    # # Get user input for start and end nodes
    start_node_id = 'Q36EJK0YUB5YOO'
    end_node_id = 'D1QV6YV9AMCQGK'

    # Find corresponding Node objects
    start_nodes = [node for node in path_tool.nodes if node.node_id == start_node_id]
    end_node = next((node for node in path_tool.nodes if node.node_id == end_node_id), None)

    if not start_nodes:
        print(f"Start node '{start_node_id}' not found.")
    elif end_node is None:
        print(f"End node '{end_node_id}' not found.")
    else:
        search_result = path_search(start_nodes, end_node, [], [])
        print('Path search completed from {} to {}.'.format(start_node_id, end_node_id))
        print("All paths:", search_result)

        # Get the graph and find the shortest path
        G = path_tool.get_graph()
        shortest_path = find_shortest_path(G, start_nodes[0].node_id, end_node.node_id)
        print("Shortest path:", shortest_path)

        # Visualize the paths and highlight the shortest path
        visualize_paths(G, search_result, shortest_path)
