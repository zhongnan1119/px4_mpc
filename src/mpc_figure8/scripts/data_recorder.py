#!/usr/bin/env python3
# coding=utf-8

"""
记录实际轨迹与期望轨迹
"""

actual_x = []
actual_y = []
desired_x = []
desired_y = []

def record_data(current_pos, desired_pos):
    """
    记录 x, y
    :param current_pos: (x, y, z)
    :param desired_pos: (x, y, z)
    """
    actual_x.append(current_pos[0])
    actual_y.append(current_pos[1])
    desired_x.append(desired_pos[0])
    desired_y.append(desired_pos[1])

def get_recorded_data():
    return actual_x, actual_y, desired_x, desired_y

def clear_data():
    actual_x.clear()
    actual_y.clear()
    desired_x.clear()
    desired_y.clear()
