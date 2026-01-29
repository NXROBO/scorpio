/*

	需求：用于实现将目标消息通过串口传输出去
    实现流程：
		1.包含头文件
		2.初始化ros节点
		3.实例化ros句柄
		4.实例化订阅者对象
		5.开启串口实现
        6.设置循环调用回调函数

*/


//1.包含头文件
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial _serial;				// serial object	


int main(int argc, char** argv)
{	
	//2.初始化hm_write_node节点
	ros::init(argc, argv, "hm_write_node");

	//3.实例化ros句柄
	ros::NodeHandle nh;

	//4.处理订阅的消息（回调函数）

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
		ROS_INFO_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ROS_INFO_STREAM("Port is open ");
	}

    //6.设置循环调用回调函数
	ros::spin();
    return 0;

}


