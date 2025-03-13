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
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
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
current_velocity = None

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def mpc_figure8_controller():
    rospy.init_node('mpc_figure8_offboard', anonymous=True)
    rate = rospy.Rate(20)  # 将在后续步骤中调整为 100Hz
    
    # 订阅状态与位姿
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, pose_cb)
    
    # 订阅速度信息
    def velocity_cb(msg):
        global current_velocity
        current_velocity = msg
    
    vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_cb)
    
    # 发布者：位置 & 速度
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
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
    
    # 1. 连续发送位置 setpoint，保证 PX4 获得足够数据流以进入 OFFBOARD
    rospy.loginfo("发送起飞目标(0,0,2)，准备进入 OFFBOARD...")
    last_request = rospy.Time.now()
    
    # 创建单独线程持续发送位置 setpoint
    def setpoint_thread():
        setpoint_rate = rospy.Rate(50)  # 50Hz, 高于 OFFBOARD 模式要求的 2Hz
        while not rospy.is_shutdown():
            takeoff_pose.header.stamp = rospy.Time.now()
            pos_pub.publish(takeoff_pose)
            setpoint_rate.sleep()
    
    import threading
    setpoint_publisher = threading.Thread(target=setpoint_thread)
    setpoint_publisher.daemon = True
    setpoint_publisher.start()
    
    # 等待 2 秒确保 setpoint 流已建立
    rospy.sleep(2.0)
    
    # 2. 切换到 OFFBOARD 模式并解锁
    offboard_sent = False
    armed = False
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        # 先尝试切换到 OFFBOARD 模式
        if not offboard_sent and (now - last_request) > rospy.Duration(5.0):
            rospy.loginfo("尝试切换到 OFFBOARD 模式")
            response = mode_srv(custom_mode="OFFBOARD")
            if response.mode_sent:
                rospy.loginfo("OFFBOARD 模式指令已发送成功")
                offboard_sent = True
            else:
                rospy.logwarn("OFFBOARD 模式切换失败，将重试")
            last_request = now
        
        # 确认已进入 OFFBOARD 模式后尝试解锁
        if offboard_sent and current_state.mode == "OFFBOARD" and not armed and (now - last_request) > rospy.Duration(5.0):
            rospy.loginfo("尝试解锁无人机 (ARM)")
            response = arm_srv(value=True)
            if response.success:
                rospy.loginfo("解锁成功")
                armed = True
            else:
                rospy.logwarn("解锁失败，将重试")
            last_request = now
        
        # 如果已解锁并且在 OFFBOARD 模式，开始执行任务
        if armed and current_state.armed and current_state.mode == "OFFBOARD":
            rospy.loginfo("无人机已解锁并进入 OFFBOARD 模式，开始执行任务")
            break
        
        # 如果已经发送了 OFFBOARD 但模式不正确，重试
        if offboard_sent and current_state.mode != "OFFBOARD" and (now - last_request) > rospy.Duration(5.0):
            rospy.logwarn(f"当前模式: {current_state.mode}，重新尝试切换到 OFFBOARD")
            response = mode_srv(custom_mode="OFFBOARD")
            last_request = now
        
        # 如果已经解锁但状态不正确，重试
        if armed and not current_state.armed and (now - last_request) > rospy.Duration(5.0):
            rospy.logwarn("无人机未保持解锁状态，重新尝试解锁")
            response = arm_srv(value=True)
            last_request = now
        
        rate.sleep()
    
    # 4. 等待 8 秒，让无人机稳定爬升到 2m (增加等待时间)
    rospy.loginfo("保持目标点 (0,0,2) 约 8 秒...")
    rospy.sleep(8.0)
    
    # 清空数据记录
    clear_data()
    
    # 5. MPC 8 字轨迹控制阶段
    MPC_TIME = 45  # 增加飞行时间到 30 秒
    rospy.loginfo(f"开始 MPC 8 字轨迹 (飞行 {MPC_TIME} 秒)")
    
    # 使用 20Hz 控制频率，匹配 0.05s 的 MPC 时间步长
    control_rate = rospy.Rate(20)  # 20Hz
    
    # 初始化上一次的控制输入，用于平滑控制
    last_u_opt = np.zeros(3)
    
    # 错误计数器，用于监控 MPC 求解器的健康状态
    error_count = 0
    max_errors = 10  # 最大允许连续错误次数
    
    start_time = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start_time
        if elapsed > MPC_TIME:
            rospy.loginfo("MPC 8 字轨迹结束")
            break
        
        # 生成期望轨迹 (x, y, z=2.0) - 使用与 trajectory.py 一致的参数
        desired_pos, desired_vel, desired_acc = generate_figure8_3d(elapsed,z_const=2.0)
        desired_state = np.array(desired_pos)
        desired_vel_np = np.array(desired_vel)
        
        # 当前状态
        cp = current_pose.pose.position
        current_state_np = np.array([cp.x, cp.y, cp.z])
        
        # 记录数据
        # 从单独的速度话题获取当前速度
        current_vel_np = None
        if current_velocity is not None:
            cv = current_velocity.twist.linear
            current_vel_np = np.array([cv.x, cv.y, cv.z])
            
        record_data(current_state_np, desired_state, current_vel_np, desired_vel_np)
        
        # 调用 MPC 求解器
       
        u_opt = solve_mpc_3d(current_state_np,desired_state, desired_vel_np)
                
            
            
            
         # 发布速度指令
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.header.frame_id = "base_link"  # 使用飞行器坐标系
        vel_cmd.twist.linear.x = u_opt[0]
        vel_cmd.twist.linear.y = u_opt[1]
        # 增加负向偏置以抵消高度漂移
        vel_cmd.twist.linear.z = u_opt[2] - 0.1  # 增加下降偏置
        vel_pub.publish(vel_cmd)
            
          
    
    # 6. 停止移动（速度清零）
    rospy.loginfo("停止 MPC，速度清零...")
    stop_cmd = TwistStamped()
    stop_cmd.header.stamp = rospy.Time.now()
    stop_cmd.header.frame_id = "base_link"  # 使用飞行器坐标系
    stop_cmd.twist.linear.z = -0.1  # 增加下降偏置以抵消高度漂移
    vel_pub.publish(stop_cmd)
    rospy.sleep(1)
    
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
    position_data, velocity_data, timestamps = get_recorded_data()
    if len(position_data[0]) > 0:
        ax, ay, az, dx, dy, dz = position_data
        avx, avy, avz, dvx, dvy, dvz = velocity_data
        plot_figure8(ax, ay, dx, dy, 
                    actual_z=az, desired_z=dz,
                    actual_vx=avx, actual_vy=avy, 
                    desired_vx=dvx, desired_vy=dvy,
                    timestamps=timestamps,
                    title="MPC Figure-8 (No AutoLanding)")

if __name__ == '__main__':
    try:
        mpc_figure8_controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("脚本中断，退出")
