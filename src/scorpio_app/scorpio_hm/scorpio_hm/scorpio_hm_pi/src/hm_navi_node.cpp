/*

	需求：实现订阅到自动消息后发布机器人自动导航到目标点话题
		
    实现流程：
		1.包含头文件
		2.初始化ros节点
		3.实例化ros句柄
		4.实例化订阅者对象
		5.处理订阅的消息（回调函数）
        6.设置循环调用回调函数

*/

//1.包含头文件
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> //MoveBaseAction消息类型
#include <actionlib/client/simple_action_client.h> //action客户端
#include <std_msgs/String.h>
#include <string.h>

 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;


//移动到目标点
void hm_move_goal_callback(const std_msgs::String::ConstPtr& msg)
{
	//订阅move_base服务器的消息 
	MoveBaseClient movebase("move_base", true);
	//等待服务器连接
	ROS_INFO("Waiting for the move_base action server");
	movebase.waitForServer(ros::Duration(60));
	ROS_INFO("Connected to move base server");
	move_base_msgs::MoveBaseGoal goal;
	// 发送目标点消息
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	//移动到设置的目标点（需要手动设着目标点）
	if(!strcmp(msg->data.c_str(),"1"))
	{
		goal.target_pose.pose.position.x = -3.31;
		goal.target_pose.pose.position.y = 4.0;
		goal.target_pose.pose.orientation.z = -0.28;
		goal.target_pose.pose.orientation.w = 0.96;
	}
	//移动回原点
	else if(!strcmp(msg->data.c_str(),"0"))
	{
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.41;
		goal.target_pose.pose.orientation.w = 0.90;
	}
	else
	{
		return;
	}
	ROS_INFO("Sending goal");
	//发送目标点
	movebase.sendGoal(goal);
	//等待到达目标点
	movebase.waitForResult();
	if (movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have reached the goal!");
	else
		ROS_INFO("The base failed for some reason");
}

int main(int argc, char** argv) {
	//2.初始化ros节点
	ros::init(argc, argv, "hm_navi_node");
	//3.实例化ros句柄
	ros::NodeHandle nh;
	//4.实例化订阅者对象
	ros::Subscriber sub = nh.subscribe<std_msgs::String>("/hm_move_goal", 1, hm_move_goal_callback);
	//5.处理订阅的消息（回调函数）
	//6.设置循环调用回调函数
	ros::spin();
	return 0;
}

