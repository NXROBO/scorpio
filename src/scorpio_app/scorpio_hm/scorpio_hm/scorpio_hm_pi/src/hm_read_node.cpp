/*
	需求：实现串口读取数据对串口读取的数据执行相应的动作
		1.接收运动指令发布控制底盘运动话题
		2.接收抓取放置指令发布控制机械臂进行话题
		3.接受自动导航指令发布控制小车进行自动导航话题
    实现流程：
		1.包含头文件
		2.初始化hm_read_node节点
		3.实例化ros句柄
		4.实例化发布者对象
		5.开启串口实现
		6.串口接收到数据实现相应动作实现

*/

//1.包含头文件
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <string>
#include <std_msgs/Empty.h>
#include <iostream>
#include <common/ShiftNav.h>
#include <fstream>
#include <ros/package.h>
#include <vector>
#include <sstream>
#include <algorithm>

#include "../../common/include/common/json/json.h"

#define WIFIFILENAME "/home/scorpio/wifi_info.txt"

using namespace std;


bool can_grasp = true;
bool can_release = false;


int readNumToArray(string buff, float *data) 
{
	int i = 0;//行数i
	vector<float> nums;
	// string->char *
	char *s_input = (char *)buff.c_str();
	// printf("\n %s jj %s \n", s_input, buff.c_str());
	const char * split = ",";
	// 以‘,’为分隔符拆分字符串
	char *p = strtok(s_input, split);
	float a;
	// printf("eee:");
	while (p != NULL) 
	{
		a = atof(p);
		nums.push_back(a);
		p = strtok(NULL, split);
	}
	for (int b = 0; b < nums.size(); b++) 
	{
		data[b] = nums[b];
		// printf("%f ",data[b]);
	}
	// printf("end size :%d\n",nums.size());
	return nums.size();
}

// 解析x:数字,z:数字格式的数据，提取x和z的值
// 返回值：true表示解析成功，false表示数据格式错误或包含重复数据
bool parseXZValues(const string& data, float& x_value, float& z_value)
{
	// 检查数据是否包含重复（通过检查是否包含多个"x:"）
	size_t first_x = data.find("x:");
	if (first_x == string::npos)
	{
		return false; // 没有找到x:标记
	}
	
	size_t second_x = data.find("x:", first_x + 1);
	if (second_x != string::npos)
	{
		// 发现重复的x:标记，数据过长，过滤掉
		// ROS_WARN_STREAM("检测到重复数据，已过滤: " << data);
		return false;
	}
	
	// 检查数据长度是否合理（正常格式应该是类似"x:1,z:-1"，长度不会太长）
	// 如果数据长度超过合理范围（比如超过20个字符），可能是异常数据
	if (data.length() > 20)
	{
		// ROS_WARN_STREAM("数据长度异常，已过滤: " << data);
		return false;
	}
	
	// 提取x的值
	size_t x_pos = data.find("x:");
	size_t comma_pos = data.find(",z:");
	if (x_pos == string::npos || comma_pos == string::npos || comma_pos <= x_pos)
	{
		return false; // 格式不正确
	}
	
	// 提取x的值（从"x:"后面到",z:"之前）
	string x_str = data.substr(x_pos + 2, comma_pos - x_pos - 2);
	
	// 提取z的值（从",z:"后面到字符串末尾）
	string z_str = data.substr(comma_pos + 3);
	
	// 转换为浮点数
	try {
		x_value = stof(x_str);
		z_value = stof(z_str);
		// ROS_INFO_STREAM("解析成功 - x: " << x_value << ", z: " << z_value);
		return true;
	}
	catch (const std::exception& e)
	{
		// ROS_WARN_STREAM("数值转换失败: " << e.what());
		return false;
	}
}

int main(int argc, char** argv)
{	
	//2.初始化hm_read_node节点
	ros::init(argc, argv, "hm_read_node");

	//3.实例化ros句柄
	ros::NodeHandle nh;

	//4.实例化发布者对象
	ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);	//底盘控制话题
	ros::Publisher pub_nv = nh.advertise<common::ShiftNav>("/shift_nav_cmd",1); //语音控制移动话题
	ros::Publisher pub_lp = nh.advertise<std_msgs::String>("/mark_nav",1);//学习途经点的话题

	serial::Serial _serial;
	common::ShiftNav nav_cmd;
	geometry_msgs::Twist cmd;
	std_msgs::String result;
	std_msgs::String learn_point_cmd;

    // 移动速度
    float walk_vel = 0;
    // 旋转速度
    float yaw_rate = 0;
    // 参数初始化
    float speed = 0;
	float turn = 0;

	ros::Rate loop_rate(20);
	//5.开启串口实现
	try
	{
		_serial.setPort("/dev/SPARK-HM-PI");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ROS_INFO_STREAM("Port is open");
	}
	
	while (ros::ok())							// publish positionesian coordinates
	{
		//6.串口接收到数据实现相应动作实现
		if (_serial.available())
		{
			result.data.clear();
			result.data = _serial.read(_serial.available());
			ROS_INFO_STREAM("Read:" << result.data);

			// 移动命令
			if (result.data.find("x:") == 0)
			{
				float x_val = 0.0f;
				float z_val = 0.0f;
				
				if (parseXZValues(result.data, x_val, z_val))
				{
					// ROS_INFO_STREAM("" << x_val << "," << z_val);
					// 根据x和z的值设置前进和旋转速度
					if (x_val > 0.5) {
						walk_vel = 0.5;  // 限制最大前进速度
					}
					else if (x_val < -0.5) {
						walk_vel = -0.5;  // 限制最大后退速度
					}
					else {
						walk_vel = x_val;  // 使用原值
					}
					yaw_rate = z_val;
				}
				else
				{
					// 解析失败或数据被过滤，忽略这条消息
					// ROS_WARN_STREAM("数据解析失败或被过滤，忽略: " << result.data);
				}
				speed = 0;
				turn = 0;
			}

			else if (result.data.rfind("plan", 0) == 0) 
			{
				// ROS_INFO_STREAM("learn:" << result.data);
				learn_point_cmd.data=result.data.c_str();
				pub_lp.publish(learn_point_cmd);
			}

			else if (result.data.rfind("voice_cmd:", 0) == 0) 
			{
				if(result.data.find("start_navigation")!=std::string::npos)
				{
					printf("收到比赛开始命令\n");
					learn_point_cmd.data="go";
					pub_lp.publish(learn_point_cmd);
				}
			}
			// 导航命令
			else if (result.data.rfind("nav_cmd:", 0) == 0) 
			{
				// printf("get_message is: %s\n",result.data.c_str());
				float array_num[10];
				string str0 = result.data;
				string str1 = str0.substr(strlen("nav_cmd:"));
				// printf("fff:%s bbb:%s\n",str0.c_str(), str1.c_str());
				readNumToArray(str1, array_num);
				// printf("%f,%f,%f xx\n",array_num[0],array_num[1],array_num[2]);
				nav_cmd.type = 0;
				nav_cmd.pose.x = array_num[0];
				nav_cmd.pose.y = array_num[1];
				nav_cmd.pose.theta = array_num[2];		
				pub_nv.publish(nav_cmd);			
				walk_vel = 0;
				yaw_rate = 0;					
			}		
		
			//wifi设置
			else if(!strcmp(result.data.c_str(),"wifi_msg"))
			{
				std::fstream m_fs;
				m_fs.open(WIFIFILENAME, ios::in);
				if (!m_fs.is_open())
				{
					cout << "读取文件失败" << endl;
				}
				string buf;
				string str_buf;
				str_buf = "set_wifi:";
				while (getline(m_fs,buf))
				{
					cout << buf << endl;
					Json::Value value_json;
    				Json::Reader reader_js(Json::Features::strictMode());
					if(reader_js.parse (buf, value_json))
					{
						str_buf = str_buf+buf;
						// printf("msg: %s\n",str_buf.c_str());
						_serial.write(str_buf);	
						printf("send:%s\n",str_buf.c_str());
					}
					else
					{
						ROS_ERROR("%s is not a valid json file", WIFIFILENAME);
						ROS_ERROR("please check json format validity in wifi_info.txt");
						ROS_ERROR("or use script/reset_wifi_info_file.sh to reset the file and re-config it");
					}
				}
				walk_vel = 0;
				yaw_rate = 0;	
			}	
			else
			{
				walk_vel = 0;
				yaw_rate = 0;	
			}
			// cmd.linear.x = speed * walk_vel;
    		// cmd.angular.z = turn * yaw_rate;

			cmd.linear.x = walk_vel;
    		cmd.angular.z = yaw_rate;
			pub1.publish(cmd);//发布/cmd_vel消息
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
