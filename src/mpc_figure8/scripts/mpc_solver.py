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
    # 定义符号变量：状态 x, 控制输入 u (均为 3 维)
    x = ca.SX.sym('x', 3)
    u = ca.SX.sym('u', 3)
    
    # 状态更新模型：x_{k+1} = x_k + u_k * dt
    x_next = x + u * dt
    f = ca.Function('f', [x, u], [x_next])
    
    # 决策变量：U (3xN) & X (3x(N+1))
    U = ca.SX.sym('U', 3, N)
    X = ca.SX.sym('X', 3, N+1)

    cost = 0
    constraints = []
    
    # 初始状态约束
    constraints.append(X[:, 0] - current_state)
    
    # 构造代价函数和动态约束
    for k in range(N):
        # 在末端状态跟踪 desired_state
        if k == N - 1:
            cost += ca.sumsqr(X[:, k+1] - desired_state)
        # 控制输入偏差惩罚
        cost += 0.1 * ca.sumsqr(U[:, k] - desired_vel)
        # 动力学：X[k+1] = X[k] + U[k]*dt
        x_next_pred = f(X[:, k], U[:, k])
        constraints.append(X[:, k+1] - x_next_pred)
    
    g = ca.vertcat(*constraints)
    opt_vars = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
    
    # 构造 NLP
    nlp = {'x': opt_vars, 'f': cost, 'g': g}
    opts = {'ipopt.print_level': 0, 'print_time': 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
    
    # 上下界
    lbx = -ca.inf * np.ones(opt_vars.shape[0])
    ubx = ca.inf * np.ones(opt_vars.shape[0])
    lbg = np.zeros(g.shape)
    ubg = np.zeros(g.shape)
    
    # 初始猜测
    init_guess = np.zeros(opt_vars.shape)
    
    sol = solver(x0=init_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
    opt_solution = sol['x'].full().flatten()
    
    # 提取第一时刻控制输入 [vx, vy, vz]
    u_opt = opt_solution[:3]
    return u_opt
