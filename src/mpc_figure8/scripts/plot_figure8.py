#!/usr/bin/env python3
# coding=utf-8

import matplotlib.pyplot as plt

def plot_figure8(actual_x, actual_y, desired_x, desired_y, title="Figure-8 Tracking"):
    plt.figure(figsize=(6,6))
    plt.plot(desired_x, desired_y, 'r--', label="Desired")
    plt.plot(actual_x, actual_y, 'b-',  label="Actual")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.title(title)
    plt.show()
