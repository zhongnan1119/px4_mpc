#!/usr/bin/env python3
# coding=utf-8

import casadi as ca
import numpy as np

def solve_mpc_3d(current_state, desired_state, desired_vel, dt=0.05, N=10):
    """
    使用 CasADi 构建并求解 3D MPC 优化问题。
    :param current_state: 当前状态 [x, y, z]
    :param desired_state: 期望位置 [x, y, z]
    :param desired_vel:   期望速度 [vx, vy, vz]
    :param dt:           离散时间步长
    :param N:            预测步数
    :return: 第一时刻的最优控制输入 [vx, vy, vz]
    """
    # 定义符号变量（3 维状态与控制输入）
    x = ca.SX.sym('x', 3)   # 状态 [x, y, z]
    u = ca.SX.sym('u', 3)   # 控制输入 [vx, vy, vz]
    
    # 状态更新模型：简单积分器 x_next = x + u * dt
    x_next = x + u * dt
    f = ca.Function('f', [x, u], [x_next])
    
    # 定义决策变量：U 控制序列（3 x N）和 X 状态轨迹（3 x (N+1)）
    U = ca.SX.sym('U', 3, N)  # [vx, vy, vz] over N steps
    X = ca.SX.sym('X', 3, N+1)

    # 权重矩阵 - 调整位置误差权重和控制输入惩罚
    Q = np.diag([40, 30.0, 80])  # 位置跟踪权重，z轴权重更高
    R = np.diag([10, 6, 30])     # 控制输入惩罚，减少速度震荡
    
    cost = 0
    constraints = []
    
    # 初始状态约束
    constraints.append(X[:, 0] - current_state)
    
    # 构造目标函数和动态约束
    for k in range(N):
        # 状态跟踪误差代价
        state_error = X[:, k] - desired_state
        cost += ca.mtimes(ca.mtimes(state_error.T, Q), state_error)
        
        # 控制输入偏差惩罚
        control_error = U[:, k] - desired_vel
        cost += ca.mtimes(ca.mtimes(control_error.T, R), control_error)
        
        # 终端状态代价（增加终端状态权重）
        if k == N - 1:
            terminal_error = X[:, k+1] - desired_state
            cost += 2.0 * ca.mtimes(ca.mtimes(terminal_error.T, Q), terminal_error)

        # 状态更新约束：X[:, k+1] = X[:, k] + U[:, k]*dt
        x_next_pred = f(X[:, k], U[:, k])
        constraints.append(X[:, k+1] - x_next_pred)
    
    # 拼接所有约束
    g = ca.vertcat(*constraints)
    # 将 U 和 X 展平成一个向量
    opt_vars = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
    
    # 构造 NLP 问题
    nlp = {'x': opt_vars, 'f': cost, 'g': g}
    opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.max_iter': 100}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
    
    # 设置决策变量和约束的上下界
    lbx = -ca.inf * np.ones(opt_vars.shape[0])
    ubx = ca.inf * np.ones(opt_vars.shape[0])
    
    # 控制输入约束（简单速度限制）
    for i in range(3*N):
        if i % 3 == 0 or i % 3 == 1:  # x, y 速度限制
            lbx[i] = -1.0
            ubx[i] = 1.0
        else:  # z 速度限制
            lbx[i] = -0.5
            ubx[i] = 0.5
    
    lbg = np.zeros(g.shape)
    ubg = np.zeros(g.shape)
    
    # 初始猜测（全零）
    init_guess = np.zeros(opt_vars.shape)
    
    try:
        # 求解 NLP
        sol = solver(x0=init_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        opt_solution = sol['x'].full().flatten()
        
        # 提取第一时刻的控制输入 [vx, vy, vz]
        u_opt = opt_solution[:3]
        
        # 检查解的有效性
        if np.any(np.isnan(u_opt)) or np.any(np.isinf(u_opt)):
            raise ValueError("MPC 求解器返回 NaN 或无限值")
        
        return u_opt
    except Exception as e:
        print(f"MPC 求解错误: {e}")
        # 返回安全的控制输入
        return np.zeros(3)
