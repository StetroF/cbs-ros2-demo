import numpy as np
from scipy import linalg

"""
LQR轨迹跟踪,基于机器人的状态方程建立
"""
dt = 0.05
max_iter = 150
EPS = 1e-4

linear_max = 0.4
linear_min = -0.4
angular_max = 0.4
angular_min = -0.4

# 引入全局变量 mode
mode = False  # 如果为 True，则将位置误差 dist 引入到整个状态方程中

class LQRController:
    def __init__(self, logger):
        self.velocity = [0.0, 0.0]
        if mode:
            self.Q = np.array([
                            [4, 0, 0, 0],
                            [0, 4, 0, 0],
                            [0, 0, 1e-2, 0],
                            [0, 0, 0, 1e2]])  # 目标误差引入后的状态代价
        else:
            self.Q = np.array([
                            [4, 0, 0],
                            [0, 4, 0],
                            [0, 0, 1e-2]])

        # 控制输入代价矩阵 R
        self.R = np.array([
                        [1e-1, 0],
                        [0, 1e-1]])  # 控制输入代价（保持不变）
        
        if mode:
            self.A = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # 状态转移矩阵
            self.B = np.array([[0, 0], [0, 0], [0, 1], [0, 0]])  # 控制输入矩阵（已更新）
        else:
            self.A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # 状态转移矩阵
            self.B = np.array([[0, 0], [0, 0], [0, 1]])  # 控制输入矩阵（已更新）
        self.logger = logger

    def update_param(self, robot_state, robot_velocity):
        x, y, yaw = robot_state
        vx, vyaw = robot_velocity
        if mode:
            self.A = np.array([[1, 0, -vx * np.sin(yaw) * dt, 0],
                            [0, 1, vx * np.cos(yaw) * dt, 0],
                            [0.0, 0.0, 1, 0],
                            [0.0, 0.0, 0, 1]])  # 目标误差引入后的状态转移矩阵

            self.B = np.array([[np.cos(yaw) * dt, 0],
                            [np.sin(yaw) * dt, 0],
                            [0, dt],
                            [0, 0]])  # 修改后的控制输入矩阵，考虑目标误差
        else:
            self.A = np.array([[1, 0, -vx * np.sin(yaw) * dt],
                            [0, 1, vx * np.cos(yaw) * dt],
                            [0.0, 0.0, 1]])  # 目标误差引入后的状态转移矩阵

            self.B = np.array([[np.cos(yaw) * dt, 0],
                            [np.sin(yaw) * dt, 0],
                            [0, dt]])  # 修改后的控制输入矩阵，考虑目标误差
        # self.logger.info(f"更新矩阵: A = {self.A}, B = {self.B}")

    def solve_riccati(self, A, B, Q, R):
        """解代数里卡提方程

        Args:
            A (_type_): 状态矩阵A
            B (_type_): 状态矩阵B
            Q (_type_): Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
            R (_type_): R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。

        Returns:
            _type_: _description_
        """
        # 设置迭代初始值
        Qf=Q
        P=Qf
        # 循环迭代
        for t in range(max_iter):
            P_=Q+A.T@P@A-A.T@P@B@np.linalg.pinv(R+B.T@P@B)@B.T@P@A
            if(abs(P_-P).max()<EPS):
                break
            P=P_
        return P_

    def norm_vel(self, u):
        linear_scale = u[0] / linear_max
        angular_scale = u[1] / angular_max
        max_scale = max(abs(linear_scale), abs(angular_scale))
        if max_scale > 1:
            u = u / max_scale
        return u

    def control(self, robot_state, point):
        A, B, Q, R = self.A, self.B, self.Q, self.R
        P = self.solve_riccati(A, B, Q, R)
        
        # 计算增益矩阵K
        K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
        
        # 计算最近路径点（只考虑x，y坐标）
        nearest_path_point = point
        # 设定目标状态为当前路径点附近的一小段

        # 计算状态误差（dx, dy）
        dist = np.sqrt((robot_state[0]-nearest_path_point[0])**2+(robot_state[1]-nearest_path_point[1])**2)
        if mode:
            x_error = np.array(robot_state[:3]) - np.array(nearest_path_point[:3])
            x_error = np.array([x_error[0], x_error[1], x_error[2], dist])
        else:
            x_error = np.array(robot_state[:3]) - np.array(nearest_path_point[:3])
        # self.logger.info(f"状态误差: {x_error}")
        # 控制输入（u = -Kx）
        u = -K @ x_error
        
        u = self.norm_vel(u)
        return u