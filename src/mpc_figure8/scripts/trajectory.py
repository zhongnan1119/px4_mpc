#!/usr/bin/env python3
# coding=utf-8

import numpy as np

def generate_figure8_3d(t, A=1, omega=0.5, z_const=1.0):
    """
    生成平滑的 3D 版 8 字轨迹 (x, y, z)，并给出对应速度 (vx, vy, vz) 和加速度 (ax, ay, az).
    :param t: 当前时间
    :param A: 轨迹幅值 (降低到 0.8，减少大幅度飞行动作)
    :param omega: 角频率 (降低到 0.08，让轨迹更易跟踪)
    :param z_const: z 轴保持恒定的高度
    :return: (pos, vel, acc)
             pos = [x, y, z]
             vel = [vx, vy, vz]
             acc = [ax, ay, az]
    """
    # 使用更平滑的正弦函数生成 8 字轨迹
    # 位置
    x = A * np.sin(omega * t)
    y = A * np.sin(omega * t) * np.cos(omega * t)
    z = z_const
    
    # 速度
    vx = A * omega * np.cos(omega * t)
    vy = A * omega * (np.cos(omega * t)**2 - np.sin(omega * t)**2)
    vz = 0.0
    
    # 加速度
    ax = -A * omega**2 * np.sin(omega * t)
    ay = -A * omega**2 * 4 * np.sin(omega * t) * np.cos(omega * t)
    az = 0.0
    
    # 应用平滑启动函数，避免初始阶段的突变
    if t < 8.0:  # 延长平滑启动时间到 8 秒
        # 使用更平滑的启动函数
        smooth_factor = 0.5 * (1 - np.cos(np.pi * t / 8.0))
        x *= smooth_factor
        y *= smooth_factor
        vx *= smooth_factor
        vy *= smooth_factor
        ax *= smooth_factor
        ay *= smooth_factor
        # 确保 z 高度保持不变
        z = z_const
        vz = 0.0
        az = 0.0
    
    return (x, y, z), (vx, vy, vz), (ax, ay, az)
