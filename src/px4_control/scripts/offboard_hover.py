#!/usr/bin/env python3
# coding=utf-8
"""
MAVROS OFFBOARD 模式 - 起飞 & 悬停
----------------------------------
本代码用于在 OFFBOARD 模式下，让无人机起飞并保持悬停在 2m 高度。

工作流程：
1. 连接 MAVROS
2. 发送 `PoseStamped` 目标点至 `/mavros/setpoint_position/local`
3. 切换到 `OFFBOARD` 模式
4. ARM（解锁）无人机
5. 持续发送目标点，保持悬停

"""

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

def offboard_hover():
    rospy.init_node('offboard_hover', anonymous=True)

    # 1. 等待 MAVROS 关键服务启动
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')

    # 2. 创建 ROS 服务客户端（用于解锁 & 切换模式）
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # 3. 创建 ROS 话题发布者（用于发送目标点）
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz 发送目标点

    # 4. 定义目标位置 (x=0, y=0, z=2) - 悬停点
    target_pose = PoseStamped()
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 2.0  # 目标高度 2m

    # 5. 先发送 100 次目标点，确保 PX4 进入 OFFBOARD 模式
    rospy.loginfo("正在发送目标点，以便 PX4 进入 OFFBOARD 模式...")
    for _ in range(100):
        pose_pub.publish(target_pose)
        rate.sleep()

    # 6. 切换至 OFFBOARD 模式
    rospy.loginfo("切换至 OFFBOARD 模式...")
    set_mode_service(custom_mode="OFFBOARD")

    # 7. ARM 无人机
    rospy.loginfo("解锁无人机...")
    arm_service(value=True)

    # 8. 持续保持目标位置，确保悬停
    rospy.loginfo("无人机起飞并悬停在 2m 高度")
    while not rospy.is_shutdown():
        pose_pub.publish(target_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        offboard_hover()
    except rospy.ROSInterruptException:
        rospy.loginfo("脚本被中断，退出程序")
