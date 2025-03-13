#!/usr/bin/env python3
# coding=utf-8

"""
记录实际轨迹与期望轨迹
"""
import rospy

actual_x = []
actual_y = []
actual_z = []
desired_x = []
desired_y = []
desired_z = []
actual_vx = []
actual_vy = []
actual_vz = []
desired_vx = []
desired_vy = []
desired_vz = []
timestamps = []

def record_data(current_pos, desired_pos, current_vel=None, desired_vel=None):
    """
    记录位置和速度数据
    :param current_pos: (x, y, z)
    :param desired_pos: (x, y, z)
    :param current_vel: (vx, vy, vz) 可选
    :param desired_vel: (vx, vy, vz) 可选
    """
    timestamps.append(rospy.Time.now().to_sec())
    actual_x.append(current_pos[0])
    actual_y.append(current_pos[1])
    actual_z.append(current_pos[2])
    desired_x.append(desired_pos[0])
    desired_y.append(desired_pos[1])
    desired_z.append(desired_pos[2])
    
    if current_vel is not None:
        actual_vx.append(current_vel[0])
        actual_vy.append(current_vel[1])
        actual_vz.append(current_vel[2])
    
    if desired_vel is not None:
        desired_vx.append(desired_vel[0])
        desired_vy.append(desired_vel[1])
        desired_vz.append(desired_vel[2])

def get_recorded_data():
    """
    获取记录的数据
    :return: 位置和速度数据
    """
    position_data = (actual_x, actual_y, actual_z, desired_x, desired_y, desired_z)
    velocity_data = (actual_vx, actual_vy, actual_vz, desired_vx, desired_vy, desired_vz)
    return position_data, velocity_data, timestamps

def clear_data():
    """
    清空所有记录的数据
    """
    actual_x.clear()
    actual_y.clear()
    actual_z.clear()
    desired_x.clear()
    desired_y.clear()
    desired_z.clear()
    actual_vx.clear()
    actual_vy.clear()
    actual_vz.clear()
    desired_vx.clear()
    desired_vy.clear()
    desired_vz.clear()
    timestamps.clear()
