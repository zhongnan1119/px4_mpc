#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define POSITION_CONTROL 0b101111111000 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
#define VELOCITY3D_CONTROL 0b011111000111

unsigned short position_mask = POSITION_CONTROL;
unsigned short velocity_mask_3D = VELOCITY3D_CONTROL;
mavros_msgs::PositionTarget current_goal, takeoff_goal, hover_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::Twist target_vel;
mavros_msgs::State current_state;
float position_x, position_y, position_z, now_x, now_y, now_z, now_yaw, current_yaw, targetvel_x, targetvel_y, targetvel_z, targetvel_yaw;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
bool received = false, get_now_pos = false, vel_pubed_un = true;//触发判断
float pi = 3.14159265;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	if (!get_now_pos) 
	{
		now_x = position_msg.pose.pose.position.x;
		now_y = position_msg.pose.pose.position.y;
		now_z = position_msg.pose.pose.position.z;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat);
		now_yaw = yaw;
		get_now_pos = true;
	}
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	current_yaw = yaw;

}

void fcu_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
	received = true;
	targetvel_x = msg->velocity.x;
	targetvel_y = msg->velocity.y;
	targetvel_z = msg->velocity.z;
	targetvel_yaw = msg->yaw_dot;	
}

void target_cb(const geometry_msgs::Twist::ConstPtr& msg)//cmd_vel回调
{
	received = true;
	target_vel = *msg;
	targetvel_x = target_vel.linear.x;
	targetvel_y = target_vel.linear.y;
	targetvel_z = target_vel.linear.z;
	targetvel_yaw = target_vel.angular.z;

	if (targetvel_yaw > 0.5)
	{
		targetvel_yaw = 0.5;
	}else if (targetvel_yaw < -0.5)
	{
		targetvel_yaw = -0.5;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL,"");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	("/iris_0/mavros/state", 10, state_cb);//读取飞控状态的话题
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	("/iris_0/mavros/setpoint_raw/local", 1); //这个话题很重要，可以控制无人机的位置速度加速度和yaw以及yaw-rate，里面有个掩码选择，需要注意
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	("/iris_0/mavros/cmd/arming");//控制无人机解锁的服务端，不需要用
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	("/iris_0/mavros/set_mode");//设置飞机飞行模式的服务端，也不需要用

	//订阅PositionCommand控制指令
    ros::Subscriber ego_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd", 10, fcu_cmd_cb);

	//订阅cmd_vel控制指令
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>
	("/attitude_control", 10, target_cb);

	ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry>
    ("/iris_0/mavros/local_position/odom",10,position_cb);
    ros::Rate rate(50.0); //控制频率尽可能高点，大于30hz
	
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	//send a few setpoints before starting
	for(int i = 50; ros::ok() && i > 0; --i)
	{
		takeoff_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		local_pos_pub.publish(takeoff_goal);
		ros::spinOnce();
		rate.sleep();
	}

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	while(ros::ok())
	{
		if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		//take off 1m
		takeoff_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		takeoff_goal.header.stamp = ros::Time::now();
		takeoff_goal.type_mask = position_mask;
		takeoff_goal.position.x = now_x;
		takeoff_goal.position.y = now_y;
		takeoff_goal.position.z =  1;
		takeoff_goal.yaw = 0;

		if(received)//触发后进行轨迹跟踪
		{
			
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;//选择body系
			current_goal.header.stamp = ros::Time::now();
			current_goal.type_mask = velocity_mask_3D;//掩码设置
			current_goal.velocity.x = targetvel_x;
			current_goal.velocity.y = targetvel_y;
			current_goal.velocity.z = targetvel_z;
			current_goal.yaw_rate = targetvel_yaw;
			// ROS_INFO("velicity control");
			local_pos_pub.publish(current_goal);
			received = false;

			ROS_INFO("速度: velocity x = %.2f, velocity y = %.2f", current_goal.velocity.x, current_goal.velocity.y);

			hover_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			hover_goal.header.stamp = ros::Time::now();
			hover_goal.type_mask = position_mask;
			hover_goal.position.x = position_x;
			hover_goal.position.y = position_y;
			hover_goal.position.z = position_z;
			hover_goal.yaw = current_yaw;

			vel_pubed_un = false;
		}
		else
		{
			if (vel_pubed_un)
			{
				local_pos_pub.publish(takeoff_goal);
			}else
			{
				local_pos_pub.publish(hover_goal);
			}
			
		}
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}