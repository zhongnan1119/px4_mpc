#!/usr/bin/env python3
# coding=utf-8

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_figure8(actual_x, actual_y, desired_x, desired_y, 
                 actual_z=None, desired_z=None, 
                 actual_vx=None, actual_vy=None, desired_vx=None, desired_vy=None,
                 timestamps=None, title="Figure-8 Tracking"):
    """
    绘制轨迹跟踪结果，包括 2D/3D 位置和速度
    """
    # 2D 位置图
    plt.figure(figsize=(12, 10))
    
    # 2D 轨迹图 (俯视图)
    plt.subplot(2, 2, 1)
    plt.plot(desired_x, desired_y, 'r--', label="Desired", linewidth=2.0)
    plt.plot(actual_x, actual_y, 'b-',  label="Actual", linewidth=2.0)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    # 设置更合适的坐标轴范围，确保能显示完整轨迹
    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    plt.title("XY Plane (Top View)")
    
    # 如果有 Z 轴数据，绘制 3D 图
    if actual_z is not None and desired_z is not None:
        ax = plt.subplot(2, 2, 2, projection='3d')
        ax.plot(desired_x, desired_y, desired_z, 'r--', label="Desired", linewidth=2.0)
        ax.plot(actual_x, actual_y, actual_z, 'b-', label="Actual", linewidth=2.0)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()
        # 设置更合适的坐标轴范围
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.set_zlim(0, 3.0)
        # 设置更好的视角
        ax.view_init(elev=30, azim=45)
        ax.set_title("3D Trajectory")
    
    # 如果有速度数据，绘制速度图
    if actual_vx is not None and actual_vy is not None and desired_vx is not None and desired_vy is not None:
        # 时间轴
        t = timestamps if timestamps is not None else np.arange(len(actual_vx))
        
        # X 方向速度
        plt.subplot(2, 2, 3)
        plt.plot(t, desired_vx, 'r--', label="Desired Vx")
        plt.plot(t, actual_vx, 'b-', label="Actual Vx")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity X (m/s)")
        plt.legend()
        plt.grid(True)
        plt.title("X Velocity")
        
        # Y 方向速度
        plt.subplot(2, 2, 4)
        plt.plot(t, desired_vy, 'r--', label="Desired Vy")
        plt.plot(t, actual_vy, 'b-', label="Actual Vy")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity Y (m/s)")
        plt.legend()
        plt.grid(True)
        plt.title("Y Velocity")
    
    plt.tight_layout()
    plt.suptitle(title, fontsize=16)
    plt.subplots_adjust(top=0.9)
    plt.show()
    
    # 如果有 Z 轴数据，单独绘制高度图
    if actual_z is not None and desired_z is not None:
        plt.figure(figsize=(10, 4))
        t = timestamps if timestamps is not None else np.arange(len(actual_z))
        plt.plot(t, desired_z, 'r--', label="Desired Z")
        plt.plot(t, actual_z, 'b-', label="Actual Z")
        plt.xlabel("Time (s)")
        plt.ylabel("Height (m)")
        plt.legend()
        plt.grid(True)
        plt.title("Height Tracking")
        plt.show()
