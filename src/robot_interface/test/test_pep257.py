import numpy as np
from scipy.linalg import solve_discrete_are

class DiscreteLQRTrajectoryTracker:
    def __init__(self, dt=0.1):
        """
        离散LQR轨迹跟踪控制器
        状态变量：x_err, y_err, theta_err, v_err, w_err
        控制变量：a (线加速度), alpha (角加速度)
        """
        self.dt = dt
        
        # 系统参数
        self.n_states = 5  # 状态维度
        self.n_controls = 2  # 控制维度
        
        # 权重矩阵配置
        self.Q = np.diag([10, 10, 5, 2, 1])  # 状态误差权重
        self.R = np.diag([0.5, 0.5])        # 控制量权重
        
        # 系统矩阵初始化
        self.Ad = None   # 离散状态矩阵
        self.Bd = None   # 离散控制矩阵
        self.K = None    # 反馈增益矩阵
        
        self._build_system_model()
        
    def _build_system_model(self):
        """构建误差状态空间模型"""
        # 连续时间模型 (在参考轨迹附近线性化)
        Ac = np.array([
            [0, 0, -1, 1, 0],   # dx/dt = -v_ref*sin(theta_ref)*theta_err + v_err
            [0, 0, 0,  0, 1],   # dy/dt = v_ref*cos(theta_ref)*theta_err + w_err
            [0, 0, 0,  0, 0],   # d(theta_err)/dt = w_err
            [0, 0, 0,  0, 0],   # dv/dt = a
            [0, 0, 0,  0, 0]    # dw/dt = alpha
        ])
        
        Bc = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1]
        ])
        
        # 离散化（前向欧拉法）
        self.Ad = np.eye(self.n_states) + Ac * self.dt
        self.Bd = Bc * self.dt
        
    def compute_gains(self):
        """求解离散代数Riccati方程"""
        P = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)
        self.K = -np.linalg.inv(self.R + self.Bd.T @ P @ self.Bd) @ self.Bd.T @ P @ self.Ad
        return self.K
    
    def compute_control(self, state, reference):
        """
        计算控制指令
        :param state: 当前状态 [x, y, theta, v, w]
        :param reference: 参考状态 [x_ref, y_ref, theta_ref, v_ref, w_ref]
        :return: 控制量 [a, alpha]
        """
        # 计算误差状态
        error = self._compute_error_state(state, reference)
        
        # 应用LQR控制律
        u = self.K @ error
        
        # 添加前馈补偿
        u_ff = self._compute_feedforward(reference)
        return u + u_ff
    
    def _compute_error_state(self, state, ref):
        """计算旋转后的误差状态"""
        x, y, theta, v, w = state
        x_ref, y_ref, theta_ref, v_ref, w_ref = ref
        
        # 位置误差（转换到机器人坐标系）
        dx = x_ref - x
        dy = y_ref - y
        rot_matrix = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        pos_err = rot_matrix @ np.array([dx, dy, 0])
        
        # 构造误差向量
        return np.array([
            pos_err[0],         # x方向误差
            pos_err[1],         # y方向误差
            theta_ref - theta,  # 角度误差
            v_ref - v,          # 线速度误差
            w_ref - w           # 角速度误差
        ])
    
    def _compute_feedforward(self, ref):
        """计算前馈项（考虑参考轨迹加速度）"""
        # 此处需要根据参考轨迹的二阶导数计算前馈量
        # 示例假设参考加速度已知，实际需根据轨迹生成模块提供
        a_ref = 0.0   # 参考线加速度
        alpha_ref = 0.0  # 参考角加速度
        return np.array([a_ref, alpha_ref])

# 使用示例
if __name__ == "__main__":
    # 初始化控制器
    lqr = DiscreteLQRTrajectoryTracker(dt=0.1)
    K = lqr.compute_gains()
    print("LQR反馈增益矩阵：\n", K)
    
    # 模拟控制循环
    current_state = np.array([1.0, 2.0, np.pi/4, 0.5, 0.1])
    target_ref = np.array([1.2, 2.3, np.pi/3, 0.6, 0.15])
    
    control = lqr.compute_control(current_state, target_ref)
    print("计算得到控制量：", control)
