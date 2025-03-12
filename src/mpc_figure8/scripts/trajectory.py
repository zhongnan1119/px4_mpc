#!/usr/bin/env python3
# coding=utf-8

import numpy as np

def generate_figure8_3d(t, A=1.0, omega=0.5, z_const=1.0):
    """
    生成 3D 版 8 字轨迹 (x, y, z)，并给出对应速度 (vx, vy, vz).
    :param t: 当前时间
    :param A: 轨迹幅值
    :param omega: 角频率
    :param z_const: z 轴保持恒定的高度
    :return: (pos, vel)
             pos = [x, y, z]
             vel = [vx, vy, vz]
    """
    # 位置
    x = A * np.sin(omega * t)
    y = A * np.sin(omega * t) * np.cos(omega * t)
    z = z_const

    # 速度
    vx = A * omega * np.cos(omega * t)
    vy = A * omega * (np.cos(omega * t)**2 - np.sin(omega * t)**2)
    vz = 0.0

    return (x, y, z), (vx, vy, vz)
