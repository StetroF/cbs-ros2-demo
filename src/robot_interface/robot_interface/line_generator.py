import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
def calculate_yaw(dx, dy):
    """计算偏航角（yaw）"""
    return np.arctan2(dy, dx)

class LineType:
    LINE = 0
    BEZIER = 1
    BSPLINE = 2

def generate_reference_path(start_p, end_p, type=LineType.LINE, step=0.1, control_points=None):
    """生成参考路径点，附带朝向信息"""
    if type == LineType.LINE:
        return generate_line_path(start_p, end_p, step)
    elif type == LineType.BEZIER:
        if control_points is None or len(control_points) != 2:
            raise ValueError("For Bezier path, two control points are required")
        return generate_bezier_path(start_p, end_p, control_points, step)
    else:
        raise ValueError("Unsupported path type")

def generate_line_path(start, end, step):
    """直线路径生成"""
    dist = np.linalg.norm(end - start)
    num_points = int(dist / step) + 1
    t = np.linspace(0, 1, num_points)
    points = start + t[:, None] * (end - start)
    
    # 计算朝向（直线路径的朝向是固定的）
    direction = end - start
    yaw = np.arctan2(direction[1], direction[0])
    yaws = np.full(len(points), yaw)  # 所有点的朝向相同
    
    # 将点和朝向合并
    return np.hstack([points, yaws[:, None]])

def generate_bezier_path(start, end, control_points, step):
    """二次贝塞尔曲线生成"""
    ctrl_p1, ctrl_p2 = control_points
    points = []
    for t in np.arange(0, 1 + step, step):
        point = (1 - t)**2 * start + 2 * (1 - t) * t * ctrl_p1 + t**2 * end
        points.append(point)
    points = np.array(points)
    
    # 计算朝向（贝塞尔曲线的切线方向）
    yaws = []
    for i in range(len(points)):
        if i == 0:
            # 第一个点的朝向是第一个点和第二个点的方向
            yaw = np.arctan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
        elif i == len(points) - 1:
            # 最后一个点的朝向是倒数第二个点和最后一个点的方向
            yaw = np.arctan2(points[-1][1] - points[-2][1], points[-1][0] - points[-2][0])
        else:
            # 中间点的朝向是前后两个点的平均方向
            yaw = np.arctan2(points[i + 1][1] - points[i - 1][1], points[i + 1][0] - points[i - 1][0])
        yaws.append(yaw)
    yaws = np.array(yaws)
    
    # 将点和朝向合并
    return np.hstack([points, yaws[:, None]])

def generate_bspline(start, end, control_points, step_size=0.02):
    """
    生成 B 样条曲线
    :param start: 起始点 [x, y]
    :param end: 终点 [x, y]
    :param control_points: 控制点列表 [[x1, y1], [x2, y2], ...]
    :param step_size: 步长，默认 0.02
    :return: 曲线点列表，每个点包含 [x, y, yaw]
    """
    # 将起始点、控制点和终点合并
    points = np.vstack([start, control_points, end]).T

    # 计算 B 样条曲线
    tck, u = splprep(points, u=None, s=0.0, per=0)
    u_new = np.arange(0, 1.0 + step_size, step_size)
    x, y = splev(u_new, tck, der=0)

    # 计算导数（用于计算 yaw）
    dx, dy = splev(u_new, tck, der=1)
    yaw = calculate_yaw(dx, dy)

    # 组合结果
    curve_points = np.vstack([x, y, yaw]).T
    return curve_points
# 示例用法
if __name__ == "__main__":
    start_p = np.array([2, 10])
    end_p = np.array([3, 14])
    
    # 设定四个控制点
    ctrl_p1 = np.array([1, 10])
    ctrl_p2 = np.array([2.5, 11.8])
    ctrl_p3 = np.array([2.8, 13])
    ctrl_p4 = np.array([4, 20])
    
    # 生成B样条曲线路径
    points = generate_reference_path(start_p, end_p, type=LineType.BEZIER, step=0.02, control_points=[ctrl_p2, ctrl_p3])

    # 使用pyplot显示点的位置和方向
    plt.plot(points[:, 0], points[:, 1], 'o-')
    plt.quiver(points[:, 0], points[:, 1], np.cos(points[:, 2]), np.sin(points[:, 2]), angles='xy', scale_units='xy', scale=1)
    plt.show()

    print("Generated path with orientations:")
    print(points)
