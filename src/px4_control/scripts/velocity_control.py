#!/usr/bin/env python3
# coding=utf-8
"""
MAVROS OFFBOARD 模式 - 速度控制 & 结束位置悬停
-------------------------------------------------
1. 起飞到 2m 高度
2. 沿 X 轴方向飞行 5 秒（速度 1 m/s）
3. 在当前停止的位置悬停（不回原点，不上升）
"""

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# 存储无人机当前位置和姿态
current_pose = PoseStamped()

def pose_callback(msg):
    """
    回调函数：订阅 /mavros/local_position/odom 获取无人机的当前位置与姿态
    """
    global current_pose
    current_pose.pose = msg.pose.pose  # 位置
    # orientation 也包含在 msg.pose.pose.orientation 内，这里一起赋值到 current_pose.pose.orientation

def velocity_control():
    global current_pose

    rospy.init_node('velocity_control', anonymous=True)

    # 1. 连接 MAVROS 的解锁与模式切换服务
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # 2. 订阅无人机位姿信息
    rospy.Subscriber('/mavros/local_position/odom', Odometry, pose_callback)

    # 3. 创建发布者：用于发送位置 setpoint（PoseStamped）
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # 4. 创建发布者：用于发送速度 setpoint（Twist）
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # 5. 目标位置：起飞到 (x=0, y=0, z=2)
    target_pose = PoseStamped()
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 2.0
    # 此处若要固定姿态，也可给 target_pose.pose.orientation 赋值一个单位四元数

    # 6. 切换 OFFBOARD 前，先连续发送一段时间的目标位置，保证 PX4 接收到足够多的 setpoint
    rospy.loginfo("发送起飞目标点，准备进入 OFFBOARD...")
    for _ in range(100):
        pose_pub.publish(target_pose)
        rate.sleep()

    # 7. 切换到 OFFBOARD 模式
    rospy.loginfo("切换到 OFFBOARD 模式")
    set_mode_service(custom_mode="OFFBOARD")

    # 8. 解锁（ARM）
    rospy.loginfo("解锁无人机 (ARM)")
    arm_service(value=True)

    # 9. 保持目标点一段时间，让无人机爬升到 2m
    rospy.loginfo("保持目标点 (0,0,2) 约 5 秒...")
    for _ in range(50):  # 10Hz * 5s
        pose_pub.publish(target_pose)
        rate.sleep()

    # 10. 定义速度指令，沿 X 轴 1 m/s
    vel_cmd = Twist()
    vel_cmd.linear.x = 1.0  # 向前 1 m/s
    vel_cmd.linear.y = 0.0
    vel_cmd.linear.z = 0.0

    rospy.loginfo("以 1 m/s 前飞 5 秒...")
    for _ in range(50):  # 10Hz * 5s
        vel_pub.publish(vel_cmd)
        rate.sleep()

    # 11. 停止移动
    rospy.loginfo("停止移动，速度清零")
    vel_cmd.linear.x = 0.0
    vel_cmd.linear.y = 0.0
    vel_cmd.linear.z = 0.0
    vel_pub.publish(vel_cmd)
    rospy.sleep(1)  # 等 1s 保证 PX4 处理最后一次速度为 0

    # 12. 切换回基于位置的控制 (PoseStamped)，并在当前位置悬停
    rospy.loginfo("切换回位置控制，悬停在当前点")

    hover_pose = PoseStamped()
    hover_pose.pose.position.x = current_pose.pose.position.x
    hover_pose.pose.position.y = current_pose.pose.position.y
    hover_pose.pose.position.z = current_pose.pose.position.z
    # 关键：保持当前姿态，否则默认 quaternion(0,0,0,0) 可能无效
    hover_pose.pose.orientation = current_pose.pose.orientation

    # 先发送若干次，让 PX4 更新 setpoint
    for _ in range(10):
        pose_pub.publish(hover_pose)
        rate.sleep()

    # 13. 进入循环，持续在 hover_pose 悬停
    rospy.loginfo("开始悬停 (持续发送当前位置)")
    while not rospy.is_shutdown():
        pose_pub.publish(hover_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("脚本中断，退出")
