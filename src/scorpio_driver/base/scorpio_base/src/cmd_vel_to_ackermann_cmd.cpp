/*********************************************************************
*
* Software License Agreement
*
*  Copyright (c) 2015, NXROBO.
*  All rights reserved.
*
* Author: litian.zhuang on 11/22/2015
*********************************************************************/
#define NODE_VERSION 0.01
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>			// cmd_vel
#include <ackermann_msgs/AckermannDriveStamped.h>

std::string frame_id;
double wheelbase;
ros::Publisher pub_acker;
ros::Subscriber sub_vel;
float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wb)
{
	float radius;
	if((omega == 0)||(v == 0))
		return 0;

	radius = v / omega;
	return atan(wb / radius);
}
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
	float steering;
	ackermann_msgs::AckermannDriveStamped msg;
	steering = convert_trans_rot_vel_to_steering_angle(cmd_vel->linear.x, cmd_vel->angular.z, wheelbase);

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id;
	msg.drive.steering_angle = steering;
	msg.drive.speed = cmd_vel->linear.x;
	
	pub_acker.publish(msg);
  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_to_ackermann_cmd");
    ros::NodeHandle _n;
	ros::NodeHandle n_private("~");
    ROS_INFO("cmd_vel_to_ackermann_cmd_node for ROS %.2f", NODE_VERSION);

	n_private.param<std::string>("frame_id", frame_id, "odom");
	n_private.param("wheelbase", wheelbase, 0.315);
	sub_vel = _n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cmdVelReceived);
	pub_acker = _n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    ros::spin();
}
