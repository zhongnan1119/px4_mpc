#!/usr/bin/env python3
# coding=utf-8
"""
MAVROS OFFBOARD 模式 + MPC 控制 + 记录&绘图
-------------------------------------------------
流程：
1. 连续发送目标位置 (0,0,2)，建立 OFFBOARD 数据流
2. 切换 OFFBOARD 并解锁 (ARM)
3. 等待 5 秒，让无人机稳定爬升到 2m
4. MPC 8 字轨迹（记录实际/期望）
5. 停止速度
6. 悬停在最后位置（不自动降落，不加锁）
7. 绘图对比实际与期望轨迹
"""

import rospy
import time
import numpy as np

# 消息类型
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
# 服务
from mavros_msgs.srv import CommandBool, SetMode

# MPC & Trajectory
from mpc_solver import solve_mpc_3d
from trajectory import generate_figure8_3d

# 数据记录 & 绘图模块
from data_recorder import record_data, get_recorded_data, clear_data
from plot_figure8 import plot_figure8

current_pose = PoseStamped()
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def mpc_figure8_controller():
    rospy.init_node('mpc_figure8_offboard', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz

    # 订阅状态与位姿
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, pose_cb)

    # 发布者：位置 & 速度
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

    # 等待 MAVROS 连接
    rospy.loginfo("等待 MAVROS 连接...")
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and not current_state.connected:
        if (rospy.Time.now() - t0).to_sec() > 10.0:
            rospy.logwarn("等待连接 10s 超时，检查 MAVROS 是否已启动")
            break
        rate.sleep()
    rospy.loginfo("已连接 MAVROS")

    # 等待服务：ARM & set_mode
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # 目标 (0,0,2)
    takeoff_pose = PoseStamped()
    takeoff_pose.pose.position.x = 0.0
    takeoff_pose.pose.position.y = 0.0
    takeoff_pose.pose.position.z = 2.0

    # 1. 连续发送 100 次位置 setpoint，保证 PX4 获得足够数据流以进入 OFFBOARD
    rospy.loginfo("发送起飞目标(0,0,2)，准备进入 OFFBOARD...")
    for _ in range(100):
        takeoff_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(takeoff_pose)
        rate.sleep()

    # 2. 切换到 OFFBOARD 模式
    rospy.loginfo("切换到 OFFBOARD 模式")
    mode_srv(custom_mode="OFFBOARD")  
    rospy.loginfo("OFFBOARD 模式指令已发送")

    # 3. 解锁 (ARM)
    rospy.loginfo("解锁无人机 (ARM)")
    arm_srv(value=True)
    rospy.loginfo("已发送解锁指令")

    # 4. 等待 5 秒，让无人机爬升到 2m
    rospy.loginfo("保持目标点 (0,0,2) 约 5 秒...")
    for _ in range(50):
        takeoff_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(takeoff_pose)
        rate.sleep()

    # 清空数据记录
    clear_data()

    # 5. MPC 8 字轨迹控制阶段（示例飞行 20 秒，让 8 字更完整）
    MPC_TIME = 15  # 可根据需要加长时间
    rospy.loginfo(f"开始 MPC 8 字轨迹 (飞行 {MPC_TIME} 秒)")
    start_time = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start_time
        if elapsed > MPC_TIME:
            rospy.loginfo("MPC 8 字轨迹结束")
            break

        # 生成期望轨迹 (x, y, z=2.0)
        desired_pos, desired_vel = generate_figure8_3d(elapsed, z_const=2.0)
        desired_state = np.array(desired_pos)
        desired_vel_np = np.array(desired_vel)

        # 当前状态
        cp = current_pose.pose.position
        current_state_np = np.array([cp.x, cp.y, cp.z])

        # 记录数据
        record_data(current_state_np, desired_state)

        # 调用 MPC 求解器
        u_opt = solve_mpc_3d(current_state_np, desired_state, desired_vel_np)

        # 发布速度指令
        vel_cmd = Twist()
        vel_cmd.linear.x = u_opt[0]
        vel_cmd.linear.y = u_opt[1]
        vel_cmd.linear.z = u_opt[2]
        vel_pub.publish(vel_cmd)
        rate.sleep()

    # 6. 停止移动（速度清零）
    rospy.loginfo("停止 MPC，速度清零...")
    stop_cmd = Twist()
    vel_pub.publish(stop_cmd)
    rospy.sleep(1)

    # ============== 不自动降落，而是悬停在最后位置 =============

    # 获取当前位置
    hover_pose = PoseStamped()
    hover_pose.pose.position.x = current_pose.pose.position.x
    hover_pose.pose.position.y = current_pose.pose.position.y
    hover_pose.pose.position.z = current_pose.pose.position.z
    # 保持当前姿态
    hover_pose.pose.orientation = current_pose.pose.orientation

    rospy.loginfo("悬停在当前点，不自动降落 (持续发送当前位置)")
    # 这里示例：悬停 10 秒后退出脚本
    start_hover = time.time()
    while not rospy.is_shutdown():
        if (time.time() - start_hover) > 10.0:
            rospy.loginfo("悬停结束，脚本退出")
            break
        hover_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(hover_pose)
        rate.sleep()

    # 7. 结束前绘图
    rospy.loginfo("飞行结束，即将绘图")
    ax, ay, dx, dy = get_recorded_data()
    if len(ax) > 0:
        plot_figure8(ax, ay, dx, dy, title="MPC Figure-8 (No AutoLanding)")

if __name__ == '__main__':
    try:
        mpc_figure8_controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("脚本中断，退出")
