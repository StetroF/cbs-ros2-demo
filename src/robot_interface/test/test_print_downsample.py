import os
import json
import matplotlib.pyplot as plt

def plot_paths():
    # 获取用户主目录
    home_path = os.path.expanduser('~')
    path1_file = os.path.join(home_path, 'path1.json')
    path2_file = os.path.join(home_path, 'path2.json')

    # 读取 JSON 文件
    try:
        with open(path1_file, 'r') as f1, open(path2_file, 'r') as f2:
            path1 = json.load(f1)
            path2 = json.load(f2)

        # 提取 x, y 坐标
        path1_x = [point['x'] for point in path1]
        path1_y = [point['y'] for point in path1]
        path2_x = [point['x'] for point in path2]
        path2_y = [point['y'] for point in path2]

        # 绘制散点图
        plt.figure(figsize=(8, 6))
        plt.scatter(path1_x, path1_y, c='blue', label='Path 1', alpha=0.7)
        plt.scatter(path2_x, path2_y, c='red', label='Path 2', alpha=0.7)

        # 设置图例和标题
        plt.legend()
        plt.title('Paths Visualization')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)

        # 显示图形
        plt.show()

    except FileNotFoundError:
        print("JSON 文件未找到，请确保路径正确。")
    except Exception as e:
        print(f"发生错误: {e}")

# 调用函数绘图
plot_paths()
