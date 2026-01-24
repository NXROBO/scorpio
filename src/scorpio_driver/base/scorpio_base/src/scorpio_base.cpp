/*
 * Copyright (c) 2019, SHENZHEN NXROBO Co.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Litain Zhuang
 *      Email: litian.zhuang@nxrobo.com
 * 		2025-12-10:加入了新电机的支持。
 */
// 小车16T的减速比为36.0234375, 轮子直径wheel=0.105m，  (PI*Wheel*((139/60)/36.0234375))/1(电机发送的值) = vel/(x)

#define NODE_VERSION 3.01
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <time.h>
#include <istream>
#include "cereal_port/CerealPort.h"
#include <std_msgs/String.h>
#include "scorpio_base/CarPwm.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "kfilter.hpp"
#include <sys/time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include "scorpio_base/DeviceParamsConfig.h"
#define MOTOR_TYPE_HY 0
#define MOTOR_TYPE_JZD 1

#define NONE "\e[0m"
#define BLACK "\e[0;30m"
#define RED "\e[0;31m"
#define GREEN "\e[0;32m"
#define YELLOW "\e[1;33m"
#define BLUE "\e[1;34m"
#define WHITE "\e[1;37m"
#define GRAY "\e[0;37m"
#define CLEAR "\033[2J"
#define CYAN "\e[0;36m"
#define MOVETO(x, y) printf("\033[%d;%dH", (x), (y))
using namespace std;
#define ANGLE_MIDDLE_POINT 1100 // right--1080++left  1118
#define ROOMBATIMEOUT (3000 * 1e6)
#define PI 3.141592654
#define WD 0.105
#define COUNT_TIMES 20
#define MAX_SPEED 1.00
#define MAX_ENCODER_COUNTS 0xFFFF
int Flag_Motor_Error = 0;
int Current_Speed;
/*****************************************************************************
	@func :Calculate CRC16-MODBUS@poly :8005(x16+x15+x2+1)
	@init :0xFFFF
	@xorout :0x0000
	@refin :yes
	@refout :yes
	*****************************************************************************/
unsigned char CRCH[256] =
	{
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

unsigned char CRCL[256] =
	{
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
		0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
		0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
		0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
		0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
		0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
		0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
		0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
		0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};
class STM32ComSwitchNode;

typedef int (STM32ComSwitchNode::*pfunc)(unsigned char *buf, int len);
union Char2Float
{
	float value;
	unsigned char buffer[4];
};

class STM32ComSwitchNode
{
private:
	ros::Time current_time;
	ros::Subscriber sub_pwm;
	ros::Subscriber sub_acker_vel;
	ros::Timer stimer;
	ros::Timer motor_send_timer;
	tf::TransformBroadcaster tf_broadcaster;
	nxsparkbase::KFilter odom_x_kfilter, odom_y_kfilter;
	std::string base_frame_id;
	std::string odom_frame_id;
	bool hall_encoder;
	double dt;
	double limited_speed;

public:
	ros::NodeHandle n;
	ros::Publisher pub_imu;
	ros::Publisher pub_odom;
	ros::Publisher pub_fback_cmd_vel;
	std::map<int, pfunc> func_map;
	std::map<int, pfunc> func_map1;
	unsigned int countSerial, lastCountSerial;
	boost::mutex t_mutex;
	boost::mutex s3_mutex;
	unsigned short Current_PWM;
	int Flag_Get_Motor = 0;
	double all_dist = 0;
	float current_speed;
	int cur_pwm;
	int new_vel_bit;
	int overCurrent;
	double odometry_x_;
	double odometry_y_;
	double odometry_yaw_;
	int Angular_Offset;
	int motor_type = 0;
	int Flag_Motor_Switch = 0;

	// Cereal port object
	cereal::CerealPort *serial_port_0_stm32;
	cereal::CerealPort *serial_port_3_motor;
	dynamic_reconfigure::Server<scorpio_base::DeviceParamsConfig> server;

	// *****************************************************************************
	// Constructor
	STM32ComSwitchNode(ros::NodeHandle _n, const char *new_serial_port)
	{
		n = _n;
		motor_type = checkMotorType();

		overCurrent = 0;
		new_vel_bit = 0;
		current_speed = 0;
		odometry_x_ = 0;
		odometry_y_ = 0;
		odometry_yaw_ = 0;
		Angular_Offset = 0;
		dynamic_reconfigure::Server<scorpio_base::DeviceParamsConfig>::CallbackType f;
		f = boost::bind(&STM32ComSwitchNode::DeviceParamsDyRecfgCallBack, this, _1, _2);
		server.setCallback(f);
		sleep(1);
		loadMotorConfig();

		// readAngularOffsetFromFile(&Angular_Offset);
		serial_port_0_stm32 = new cereal::CerealPort();
		serial_port_3_motor = new cereal::CerealPort();

		getPtrFunction();
		//	ros::param::get("~hall_encoder",hall_encoder);
		if (n.getParam("hall_encoder", hall_encoder))
		{
			if (hall_encoder)
			{
				ROS_INFO("hall_encoder is true");
				n.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
				n.param<std::string>("odom_frame_id", odom_frame_id, "odom");
				pub_fback_cmd_vel = n.advertise<geometry_msgs::Twist>("/scorpio_base/command/velocity", 1);
				// the velocity of robot's feedback
				pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1);
			}
		}
		else
		{
			ROS_ERROR_STREAM("Failed to load " << "hall_encoder");
		}
		if (n.getParam("limited_speed", limited_speed))
		{
			if (limited_speed > MAX_SPEED)
				limited_speed = MAX_SPEED;
			ROS_INFO("limited_speed is %f", limited_speed);
		}
		else
		{
			limited_speed = MAX_SPEED;
			ROS_INFO("not set ! limited_speed is %f", limited_speed);
		}
		pub_imu = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
		stimer = n.createTimer(ros::Duration(1), &STM32ComSwitchNode::checkSerialGoon, this);
		motor_send_timer = n.createTimer(ros::Duration(0.1), &STM32ComSwitchNode::motorSendData, this);
		resetOdometry();
	}

	// *****************************************************************************
	// Destructor
	~STM32ComSwitchNode()
	{
		startCloseCmd(0x00, 0);
		closeSerialPort(&serial_port_0_stm32);
		closeSerialPort(&serial_port_3_motor);
		delete serial_port_0_stm32;
		delete serial_port_3_motor;
	}

	/**
	 * 	消毁线程
	 */
	bool destroyThread(boost::thread **th)
	{
		if ((*th) != NULL)
		{
			(*th)->interrupt();
			(*th)->join();
			delete (*th);
			(*th) = NULL;
			return true;
		}
		return true;
	}
	// 检测电机型号
	int checkMotorType(void)
	{
		std::string filePath = "/opt/nxrobo/ScorpioMotorType.txt";
		std::ifstream file(filePath);

		// 检查文件是否存在
		if (!file.is_open())
		{
			std::cout << "文件不存在: " << filePath << std::endl;
			return MOTOR_TYPE_HY;
		}

		// 读取文件内容
		std::string content;
		std::getline(file, content);
		file.close();

		// 去除可能的空白字符
		size_t start = content.find_first_not_of(" \t\n\r");
		size_t end = content.find_last_not_of(" \t\n\r");

		if (start != std::string::npos && end != std::string::npos)
		{
			content = content.substr(start, end - start + 1);
		}

		// 检查内容是否为"JZD"
		if (content == "JZD")
		{
			std::cout << "电机型号为: JZD" << std::endl;
			return MOTOR_TYPE_JZD;
		}
		else if (content == "HW")
		{
			std::cout << "电机型号为: " << content << std::endl;
			return MOTOR_TYPE_HY;
		}
		else
		{
			std::cout << "电机型号为: " << content << std::endl;
			return MOTOR_TYPE_HY;
		}
	}
	void DeviceParamsDyRecfgCallBack(scorpio_base::DeviceParamsConfig &config, uint32_t level)
	{
		server.updateConfig(config);
		Angular_Offset = config.angle_offset;
		ROS_INFO("dynamic_reconfigure message, motor angle offset is %d", config.angle_offset);
	}

	void loadMotorConfig()
	{
		ros::NodeHandle nh;
		int tmp_angle_offset;
		std::string cmd;
		std::string home_path = "/opt/nxrobo/device_config.yaml";
		std::string ros_ws_path = ros::package::getPath("scorpio_base");
		ros_ws_path.append("/cfg/");
		cmd = "cp ";
		cmd = cmd + home_path + " " + ros_ws_path;
		system(cmd.c_str());
		if (n.getParam("/motor/angle_offset", tmp_angle_offset))
		{
			// n.param("/motor/angle_offset", tmp_angle_offset, 0);
			ROS_INFO("/motor/angle_offset %d", tmp_angle_offset);
		}
		else
		{
			n.param("/motor/angle_offset", tmp_angle_offset, 0);
			ROS_INFO("xx /motor/angle_offset %d", tmp_angle_offset);
		}

		scorpio_base::DeviceParamsConfig config;
		server.getConfigDefault(config);
		config.angle_offset = tmp_angle_offset;
		Angular_Offset = tmp_angle_offset;
		server.updateConfig(config);
	}

	void saveMotorConfig()
	{
		std::string cmd;
		std::string home_path = "/opt/nxrobo/device_config.yaml";
		std::string ros_ws_path = ros::package::getPath("scorpio_base");
		ros_ws_path.append("/cfg/device_config.yaml");
		ROS_INFO("YAML path \"%s\"", ros_ws_path.c_str());
		YAML::Node config = YAML::LoadFile(ros_ws_path);
		config["motor"]["angle_offset"] = Angular_Offset;

		std::ofstream fout(ros_ws_path);
		fout << config;
		cmd = "sudo cp ";
		cmd = cmd + ros_ws_path + " " + home_path;
		system(cmd.c_str());
	}
	// 从文件读取角度偏移量并转换为INT
	int readAngularOffsetFromFile(int *result)
	{
		FILE *file = NULL;
		char buffer[256] = {0};
		int number = 0;
		char filename[256];
		const char *home = getenv("HOME"); // 获取用户目录
		sprintf(filename, "%s/Documents/angular_offset.txt", home);
		// 打开文件－－只读方式
		file = fopen(filename, "r");
		if (file == NULL)
		{
			ROS_ERROR("错误，无法打开角度偏移量文件 %s", filename);
			ROS_ERROR("建议先校准好再使用！");

			return -1; //
		}

		// 读取内容
		if (fgets(buffer, sizeof(buffer), file) == NULL)
		{
			ROS_ERROR("错误，无法读取内容\n");
			fclose(file);
			return -2; //
		}

		// 关闭文件
		fclose(file);

		// 将字符串转成整数
		number = atoi(buffer);

		// 将字符串转成整数
		*result = number;

		ROS_INFO("读取成功，数字值为%d\n", number);
		return 0; // 成功
	}

	void ackerMannCmdVelReceived(const ackermann_msgs::AckermannDriveStamped::ConstPtr &ack_vel)
	{
		float vel = ack_vel->drive.speed;
		t_mutex.lock();
		current_speed = vel;
		new_vel_bit = 1;
		t_mutex.unlock();
		rcvPwmFun(ack_vel->drive.speed, ack_vel->drive.steering_angle);
		countSerial++;
	}

	void rcvPwmFun(float x, float z)
	{
		int angular_middle_point = 1080 + 20 - Angular_Offset;
		int pwml = 1080, pwma = angular_middle_point;
		unsigned char buf[20];
		float dz;
		if (z > 1)
			z = 1;
		dz = -180 * z / M_PI * 6;
		if (dz > 0) // right转
		{
			pwma = angular_middle_point - dz;
			if (pwma < 800) // 770
				pwma = 800; // 770
		}
		else if (dz < 0) // left转
		{
			pwma = angular_middle_point - dz;
			if (pwma > 1460) // 1230
				pwma = 1460; // 1230
		}
		buf[0] = pwml >> 8;
		buf[1] = pwml;
		buf[2] = pwma >> 8;
		buf[3] = pwma;
		writeData(0x01, buf, 4);
	}
	// send car vel to stm32
	void sendVel2Stm(float vel)
	{
		unsigned char buf[10];
		int velx100 = vel * 100;
		buf[0] = velx100 >> 24;
		buf[1] = velx100 >> 16;
		buf[2] = velx100 >> 8;
		buf[3] = velx100;
		writeData(0x07, buf, 4);
	}
	// type:00 is bottom switch,01 is motor power
	void startCloseCmd(char type, char onoff)
	{
		unsigned char buf[10];
		buf[0] = type;
		buf[1] = onoff;
		writeData(0x06, buf, 2);
	}
	void checkSerialGoon(const ros::TimerEvent &event)
	{
		static int last_pwm;
		static int first_time = 1;
		static int swap_bit;
		static int ovcnt = 0;
		if (first_time)
		{
			sleep(1);
			if (motor_type == 1) // JZD电机
			{
				while((Flag_Get_Motor == 0)||(first_time == 1))
				{
					printf("======init the motor!======\n");
					for (int i = 0; i < 10; i++)
					{
						startCloseCmd(0x01, 0x01); // open motor power
						usleep(100000);
						write_Can_Start_Data();
						usleep(100000);
						rcvPwmFun(0, 0);
						usleep(100000);

					}

					write_Can_Start_Data();
					usleep(100000);

					write_Can_Clear_Stall();
					usleep(100000);
					rcvPwmFun(0, 0);

					write_Can_Free_Wheel();
					usleep(100000);

					write_Can_Set_Odom_Feedback();
					usleep(100000);

					write_Can_Auto_Send_Odom_Time(10);
					usleep(100000);

					write_Can_Odom_Switch(0x01);
					usleep(100000);
					first_time = 0;


				}

			}
			else
			{
				write_config_Data(0x0006, 0x0001, 0x00);
				first_time = 0;
				sleep(1);
			}
			sub_acker_vel = n.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, &STM32ComSwitchNode::ackerMannCmdVelReceived, this);
			return;
		}
		else
		{
			if (motor_type == MOTOR_TYPE_JZD)
			{
				float speed = (double)Current_Speed / 360 * PI * WD / 36.0234375;
				//printf("current speed is %d, %f\n", Current_Speed, speed);
				sendVel2Stm(speed);
			}
		}
		if (countSerial == lastCountSerial)
		{
			rcvPwmFun(0, 0);
			if (swap_bit)
			{
				if (motor_type == 1) // JZD电机
				{
					if (Flag_Motor_Error)
					{
						write_Can_Clear_Stall(); // 清除堵转标志
						Flag_Motor_Error = 0;
					}
					else
						write_Can_Get_Motor_Status();
				}
				else
				{
					write_config_Data(0x0006, 0x0001, 0x00);
				}
				swap_bit = 0;
			}
			else
			{
				write_vel2motor(0.0);
				swap_bit = 1;
			}
		}
		else
		{
			lastCountSerial = countSerial;
		}
		if (overCurrent)
		{
			ovcnt++;
			if (ovcnt > 4)
			{
				startCloseCmd(0x01, 0x01); // open motor power
			}
			else if (ovcnt > 2)
			{
				startCloseCmd(0x01, 0x00); // close motor power
			}
		}
		else
			ovcnt = 0;
#if 0
		double cur_dist = PI*WD*(cur_pwm-last_pwm)*5/574; //287
		last_pwm = cur_pwm;
		ROS_WARN("current speed is %f", cur_dist);
#endif
	}

	int read_write_Data(unsigned short read_addr, unsigned short read_len, unsigned short write_addr, unsigned short write_len, short vel)
	{
		unsigned int i;
		unsigned char sum = 0;
		unsigned char buffer[40];
		unsigned short crc_word;
		vel = -vel;
		Char2Float uvel;
		uvel.value = vel;
		buffer[0] = 0x01;
		buffer[1] = 0x17;
		buffer[2] = read_addr >> 8;
		buffer[3] = read_addr;
		buffer[4] = read_len >> 8;
		buffer[5] = read_len;
		buffer[6] = write_addr >> 8;
		buffer[7] = write_addr;
		buffer[8] = write_len >> 8;
		buffer[9] = write_len;
		buffer[10] = 0x02;
		buffer[11] = vel >> 8;
		buffer[12] = vel;
		crc_word = CalculateCRC16(buffer, 13);
		buffer[13] = crc_word;
		buffer[14] = crc_word >> 8;
		//	    for(int i=0; i<15; i++)
		//			printf("%02x ",buffer[i]);
		//	    printf("\n");

		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 15);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}
	int write_config_Data(unsigned short write_addr, unsigned short write_len, short dat)
	{
		unsigned int i;
		unsigned char sum = 0;
		unsigned char buffer[40];
		unsigned short crc_word;

		buffer[0] = 0x01;
		buffer[1] = 0x10;
		buffer[2] = write_addr >> 8;
		buffer[3] = write_addr;
		buffer[4] = write_len >> 8;
		buffer[5] = write_len;
		buffer[6] = write_len * 2;
		buffer[7] = dat >> 8;
		buffer[8] = dat;
		crc_word = CalculateCRC16(buffer, 9);
		buffer[9] = crc_word;
		buffer[10] = crc_word >> 8;
		//	   for(int i=0; i<11; i++)
		//			printf("%02x ",buffer[i]);
		//	   printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 11);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}
	//====================JZD start============================
	int write_Can_Start_Data()
	{
		unsigned int i;
		unsigned char buffer[10];
		if (Flag_Motor_Switch == 1)
		{
			return 1;
		}
		buffer[0] = 0x2B;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x01;
		buffer[4] = 0x0F;
		buffer[5] = 0x00;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		Flag_Motor_Switch = 1;
		return (0);
	}

	int write_Can_Free_Wheel()
	{
		unsigned int i;
		unsigned char buffer[10];
		// return 1;
		if (Flag_Motor_Switch == 0)
		{
			return 1;
		}
		buffer[0] = 0x2B;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x01;
		buffer[4] = 0x06;
		buffer[5] = 0x00;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		Flag_Motor_Switch = 0;
		return (0);
	}
	// time:单位10ms
	int write_Can_Auto_Send_Odom_Time(unsigned char time)
	{
		unsigned int i;
		unsigned char buffer[10];
		buffer[0] = 0x2F;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x00;
		buffer[4] = 0x34;
		buffer[5] = time;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}
	int write_Can_Get_Motor_Status()
	{
		unsigned int i;
		unsigned char buffer[10];

		buffer[0] = 0x40;
		buffer[1] = 0x01;
		buffer[2] = 0x26;
		buffer[3] = 0x00;
		buffer[4] = 0x00;
		buffer[5] = 0x00;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}

	int write_Can_Odom_Switch(unsigned char onoff)
	{
		unsigned int i;
		unsigned char buffer[10];

		buffer[0] = 0x2F;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x00;
		buffer[4] = 0x10;
		buffer[5] = onoff; // 0x01表示打开，0x00表示关闭
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}
	int write_Can_Clear_Stall()
	{
		unsigned int i;
		unsigned char buffer[10];
		buffer[0] = 0x2F;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x00;
		buffer[4] = 0x39;
		buffer[5] = 0x01;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}
	int write_Can_Set_Odom_Feedback()
	{
		unsigned int i;
		unsigned char buffer[10];
		buffer[0] = 0x2F;
		buffer[1] = 0x40;
		buffer[2] = 0x60;
		buffer[3] = 0x00;
		buffer[4] = 0x0F;
		buffer[5] = 0x01;
		buffer[6] = 0x00;
		buffer[7] = 0x00;
		// for (int i = 0; i < 8; i++)
		// 	printf("%02x ", buffer[i]);
		// printf("\n");
		try
		{
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}

	void write_vel2motor(float speed)
	{
		if (motor_type == MOTOR_TYPE_JZD)
		{
			unsigned int i;
			unsigned char buffer[10];
			unsigned char result = std::abs(static_cast<int>(speed / 0.0212137559910961));
			if (result != 0)
			{
				write_Can_Start_Data();
			}
			// printf("speed is %f, result is %d\n", speed, result);
			buffer[0] = 0x2B;
			buffer[1] = 0xF0;
			buffer[2] = 0x2F;
			buffer[3] = 0x09;
			if (speed >= 0)
			{
				buffer[4] = 0x01;
				buffer[5] = result;
			}
			else
			{
				buffer[4] = 0x00;
				buffer[5] = result;
			}
			buffer[6] = 0x00;
			buffer[7] = 0x00;
			// for (int i = 0; i < 8; i++)
			// 	printf("%02x ", buffer[i]);
			// printf("\n");
			s3_mutex.lock();
			serial_port_3_motor->write((char *)buffer, 8);
			s3_mutex.unlock();

			if (result == 0)
			{
				write_Can_Free_Wheel();
			}
		}
		else
		{
			short mv;
			if (speed > limited_speed)
				speed = limited_speed;
			else if (speed < -limited_speed)
				speed = -limited_speed;
			mv = speed * 11800;
			read_write_Data(0x002a, 0x0001, 0x002B, 0x0001, mv);
		}
	}
	//====================JZD end============================
	unsigned short CalculateCRC16(unsigned char *msgPtr, unsigned int msgLen)
	{
		unsigned char crcHigh = 0xFF;
		unsigned char crcLow = 0xFF;
		unsigned char index;
		while (msgLen--)
		{
			index = crcLow ^ (*(msgPtr++));
			crcLow = crcHigh ^ CRCH[index];
			crcHigh = CRCL[index];
		}
		return (unsigned short)((unsigned short)(crcHigh << 8) | crcLow);
	}

	void motorSendData(const ros::TimerEvent &event)
	{
		float vel;
		int newbit = 0;
		t_mutex.lock();
		if (new_vel_bit)
		{
			vel = current_speed;
			newbit = 1;
			new_vel_bit = 0;
		}
		t_mutex.unlock();
		if (newbit)
			write_vel2motor(vel);
	}

	void getPtrFunction()
	{
		func_map[0x0000] = &STM32ComSwitchNode::nullFun;
		func_map[0x01] = &STM32ComSwitchNode::baseFun;
	}

	void callFunction(int index, unsigned char *recvbuf, int len)
	{
		if (func_map.count(index))
			(this->*(func_map[index]))(recvbuf, len);
		/*else
			ROS_ERROR("unknown function:%02x", index);*/
	}

	int nullFun(unsigned char *buf, int len)
	{
		ROS_INFO("this is a null function!");
	}
	void resetOdometry()
	{
		setOdometry(0.0, 0.0, 0.0);
	}

	void setOdometry(double new_x, double new_y, double new_yaw)
	{
		odometry_x_ = new_x;
		odometry_y_ = new_y;
		odometry_yaw_ = new_yaw;
	}

	int baseFun(unsigned char *buf, int len)
	{
		static unsigned int timesec, lastsec;
		static int lastpwm, curpwm;
		static double robot_yaw;
		static int idx;
		static double vel_x, vel_y, vel_yaw;
		static double wheel_dist = 0;
		static int first_time = 1;
		double cur_dist;
		static double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
			odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
		float speed;
		float acvx, acvy, acvz, anvx, anvy, anvz, roll, pitch, yaw;
		float qx, qy, qz, qw;
		sensor_msgs::Imu car_imu;
		tf::Quaternion q;
		int curr_idx = (idx + COUNT_TIMES - 1) % COUNT_TIMES;
		struct timeval tv;
		gettimeofday(&tv, NULL);
		long long ts = (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
		current_time = ros::Time::now(); // ros time
		if (motor_type == MOTOR_TYPE_JZD)
		{
			if (Flag_Get_Motor == 0)
				return 1;
			fb_time[curr_idx] = ts; // set spark base time which is different from ros time
			curpwm = Current_PWM;
			if (first_time)
			{
				lastpwm = curpwm;
				first_time = 0;
			}
			cur_pwm = curpwm;
			short encoder_counts_ = curpwm - lastpwm;
			// printf("first=%d ", encoder_counts_);
			if (encoder_counts_ > MAX_ENCODER_COUNTS / 2)
				encoder_counts_ = -(MAX_ENCODER_COUNTS - curpwm + lastpwm);
			else if (encoder_counts_ < -MAX_ENCODER_COUNTS / 2)
				encoder_counts_ = (MAX_ENCODER_COUNTS - lastpwm + curpwm);
			cur_dist = -(double)encoder_counts_ / 360 / 36.0234375 * PI * WD;
			all_dist = all_dist + cur_dist;
		}
		else
		{
			fb_time[curr_idx] = ts; // set spark base time which is different from ros time
			timesec = (buf[30] << 24) | (buf[31] << 16) | (buf[32] << 8) | buf[33];
			curpwm = (buf[26] << 24) | (buf[27] << 16) | (buf[28] << 8) | buf[29];
			if (first_time)
			{
				lastpwm = curpwm;
				first_time = 0;
			}
			cur_pwm = curpwm;
			cur_dist = PI * WD * (curpwm - lastpwm) * 5 / 574;
		}

		lastpwm = curpwm;
		// ROS_INFO("the speed is %fm/s", speed);
		acvx = (float(short((buf[1] << 8) | buf[0])) / 32768 * 16 * 9.8); // m/s^2
		acvy = (float(short((buf[3] << 8) | buf[2])) / 32768 * 16 * 9.8);
		acvz = (float(short((buf[5] << 8) | buf[4])) / 32768 * 16 * 9.8);

		anvx = (float(short((buf[7] << 8) | buf[6])) / 32768 * 2000);
		anvy = (float(short((buf[9] << 8) | buf[8])) / 32768 * 2000);
		anvz = (float(short((buf[11] << 8) | buf[10])) / 32768 * 2000);

		roll = (float(short((buf[13] << 8) | buf[12]))) / 32768 * M_PI;
		pitch = (float(short((buf[15] << 8) | buf[14]))) / 32768 * M_PI;
		yaw = (float(short((buf[17] << 8) | buf[16]))) / 32768 * M_PI;
		//	printf("yaw is %f\n", yaw);
		qx = (float(short((buf[19] << 8) | buf[18])) / 32768);
		qy = (float(short((buf[21] << 8) | buf[20])) / 32768);
		qz = (float(short((buf[23] << 8) | buf[22])) / 32768);
		qw = (float(short((buf[25] << 8) | buf[24])) / 32768);

		/*	q = tf::createQuaternionFromYaw(yaw);
			car_imu.orientation.x = q.x();
			car_imu.orientation.y = q.y();
			car_imu.orientation.z = q.z();
			car_imu.orientation.w = q.w();
			car_imu.orientation_covariance[8] = pow(0.0017, 2);*/

		//	car_imu.orientation.x = qx;
		//	car_imu.orientation.y = qy;
		//	car_imu.orientation.z = qz;
		//	car_imu.orientation.w = qw;
		q = tf::createQuaternionFromRPY(roll, pitch, yaw);
		car_imu.orientation.x = q.x();
		car_imu.orientation.y = q.y();
		car_imu.orientation.z = q.z();
		car_imu.orientation.w = q.w();
		car_imu.orientation_covariance[0] = pow(0.0017, 2); //
		car_imu.orientation_covariance[4] = pow(0.0017, 2);
		car_imu.orientation_covariance[8] = pow(0.0017, 2);

		car_imu.angular_velocity.x = anvx * M_PI / 180.0; // rad/s
		car_imu.angular_velocity.y = anvy * M_PI / 180.0;
		car_imu.angular_velocity.z = anvz * M_PI / 180.0;
		car_imu.angular_velocity_covariance[0] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[4] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[8] = pow(0.1, 2);

		car_imu.linear_acceleration.x = acvx; // m/s^2
		car_imu.linear_acceleration.y = acvy;
		car_imu.linear_acceleration.z = acvz;
		car_imu.linear_acceleration_covariance[0] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[4] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[8] = pow(0.1, 2);

		car_imu.header.stamp = ros::Time::now();
		car_imu.header.frame_id = "IMU_link";
		pub_imu.publish(car_imu);

		if (hall_encoder)
		{
			// Update odometry
			odometry_x_ = odometry_x_ + cur_dist * cos(odometry_yaw_); // m
			odometry_y_ = odometry_y_ + cur_dist * sin(odometry_yaw_); // m
			odometry_yaw_ = yaw;
			wheel_dist = wheel_dist + cur_dist;
			// first, we'll publish the transforms over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = odom_frame_id;
			odom_trans.child_frame_id = base_frame_id;
			odom_trans.transform.translation.x = odometry_x_;
			odom_trans.transform.translation.y = odometry_y_;
			//    ROS_DEBUG("x=%f,y=%f",sparkbase->odometry_x_,sparkbase->odometry_y_);
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_yaw_);
			tf_broadcaster.sendTransform(odom_trans);

			// next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = odom_frame_id;

			// printf("%f,%f\n",sparkbase->odometry_x_,sparkbase->odometry_y_);
			// set the position
			odom.pose.pose.position.x = odometry_x_;
			odom.pose.pose.position.y = odometry_y_;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_yaw_);
			double est_x = odom_x_kfilter.predict(wheel_dist);

			odom_x[curr_idx] = est_x;
			odom_yaw[curr_idx] = odometry_yaw_;

			dt = (fb_time[curr_idx] - fb_time[idx]) * 0.001;
			vel_x_list[curr_idx] = (odom_x[curr_idx] - odom_x[idx]) / dt;
			vel_x = 0;
			for (int i = 0; i < COUNT_TIMES; i++)
			{
				vel_x += vel_x_list[i];
			}
			vel_x = vel_x / COUNT_TIMES;

			vel_y = 0; //(odom_y[curr_idx] - odom_y[idx])/dt;

			double delodom = (odom_yaw[curr_idx] - odom_yaw[idx]);
			if (delodom > 3.14159265359)
			{
				delodom = delodom - 2 * 3.14159265359;
			}
			if (delodom < -3.14159265359)
			{
				delodom = delodom + 2 * 3.14159265359;
			}
			vel_yaw = delodom / dt;

			double tmp_dist = 0;
			fb_dist[curr_idx] = wheel_dist;
			for (int i = 0; i < COUNT_TIMES; i++)
			{
				tmp_dist += fb_dist[i];
			}

			double fb_x = tmp_dist / dt;

			idx = (idx + 1) % COUNT_TIMES;

			odom.child_frame_id = base_frame_id;
			odom.twist.twist.linear.x = vel_x;
			odom.twist.twist.linear.y = vel_y;
			odom.twist.twist.angular.z = vel_yaw;
			// publish the odom's message

			// add covariance
			odom.pose.covariance[0] = pow(0.01, 2);
			odom.pose.covariance[1] = pow(0.05, 2);
			odom.pose.covariance[5] = pow(0.1, 2);
			pub_odom.publish(odom);

			// printf("odom x: %f, y: %f, z: %f,  w: %f\n",odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odometry_yaw_);
			// publish the feedback's twist message from the car base
			pub_fback_cmd_vel.publish(odom.twist.twist);
		}
	}

	int writeData(unsigned char cmd, unsigned char *buf, unsigned int len)
	{
		// Compose comand
		unsigned int i;
		unsigned char sum = 0;
		unsigned char buffer[200];
		buffer[0] = 'N';
		buffer[1] = 'X';
		buffer[2] = (len + 6) >> 8;
		buffer[3] = len + 6;
		buffer[4] = cmd;
		for (i = 0; i < len; i++)
		{
			buffer[5 + i] = buf[i];
		}
		for (i = 0; i < len + 5; i++)
		{
			sum = sum + buffer[i];
		}
		buffer[5 + len] = sum;
		// for(int i=0; i<length; i++)
		//    printf("%02x ",buffer[i]);
		//    printf("\n");
		try
		{
			serial_port_0_stm32->write((char *)buffer, len + 6);
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}

	void startSerial(boost::function<void(char *, int)> f, cereal::CerealPort **serial_port_, std::string port_name_, int port)
	{
		if (openSerialPort(f, serial_port_, port_name_, port) == 0)
		{
			ROS_INFO("Connected to Scorpio base successfully.%s", port_name_.c_str());
			startCloseCmd(0x00, 1);
		}
		else
		{
			ROS_FATAL("Could not connect to Scorpio base.%s", port_name_.c_str());
			ROS_BREAK();
		}
	}

	// *****************************************************************************
	// Open the serial port
	int openSerialPort(boost::function<void(char *, int)> f, cereal::CerealPort **serial_port_, std::string port_name_, int port)
	{
		try
		{
			(*serial_port_)->open(port_name_.c_str(), port);
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		if ((*serial_port_)->startReadStream(f) != true)
		{
			closeSerialPort(serial_port_);
			return (-1);
		}
		return (0);
	}
	// *****************************************************************************
	// Close the serial port
	int closeSerialPort(cereal::CerealPort **serial_port_)
	{
		(*serial_port_)->stopStream();
		try
		{
			(*serial_port_)->close();
		}
		catch (cereal::Exception &e)
		{
			return (-1);
		}
		return (0);
	}

	// *****************************************************************************
	// check sum
	unsigned char checkSum(unsigned char *buf)
	{
		unsigned char sum = 0;
		int i;
		int len = (buf[2] << 8) + buf[3];
		for (i = 0; i < len - 1; i++)
		{
			sum += buf[i];
		}
		return sum;
	}
	void getComCanData(char *buf_r, int len)
	{
		int i;
		unsigned short crc_word;
		unsigned char *buf;
		buf = (unsigned char *)buf_r;
		/*for(i=0; i<len; i++)
			printf("%02x ", (buf[i]));
		printf("\n");*/
		if ((buf[0] == 0x08) && (len > 10))
		{
			if ((buf[1] == 0x01) && (buf[2] == 0x83))
			{
				if ((buf[3]) == 0x43)
				{
					Current_PWM = (buf[8] << 8) | buf[7];
					int cs = (buf[5] << 8) | buf[4];
					if (buf[6] == 0)
						Current_Speed = -cs;
					else
						Current_Speed = cs;
					// printf("current speed is %d, %f\n", Current_Speed,speed);
					// printf("%02X%02X\n", buf[8], buf[7]);
					Flag_Get_Motor = 1;
				}
			}
			else if ((buf[1] == 0x05) && (buf[2] == 0x83))
			{
				//ROS_ERROR("the motor status is %02X, %02X, %d", buf[9], buf[10], len);

				if ((buf[3] == 0x43) && (buf[9] == 0x04C) && (buf[10] == 0x4D))
				{
					if ((buf[7] != 0x00) || (buf[8] != 0x00))
					{
						Flag_Motor_Error = 1;
						ROS_ERROR("the motor status is %02X, %02X", buf[7], buf[8]);

					}
				}
			}
		}
	}
	void getCom3Data(char *buf_r, int len)
	{
		int i;
		unsigned short crc_word;
		unsigned char *buf;
		buf = (unsigned char *)buf_r;
		/*for(i=0; i<len; i++)
			printf("%02x ", (buf[i]));
		printf("\n");*/
		if ((buf[0] == 1) && (len > 3))
		{
			if (buf[1] == 0x17)
			{
				if ((buf[2] + 5) == len)
				{
					crc_word = CalculateCRC16((unsigned char *)buf, 5);
					//	printf("%04x\n", crc_word);
					if ((unsigned short)(buf[len - 2] + (buf[len - 1] << 8)) == crc_word)
					{
						if (buf[4] & 0x01)
						{
							ROS_ERROR("the motor is overcurrent!");
							overCurrent = 1;
						}
						else
							overCurrent = 0;
					}
					else
					{
						//	ROS_ERROR("check crc error");
					}
				}
			}
		}
	}

	void getStm32ComData(char *buf, int len)
	{
		int i, j;
		static unsigned int count = 0;
		unsigned int checkcount;
		long long timediff;
		unsigned char tmpbuf[2550];
		static unsigned char recvbuf[2550];
		ros::Time currenttime;
		static ros::Time headertime;
		static int firsttime = 1;
		if (firsttime)
		{
			headertime = ros::Time::now();
			firsttime = 0;
		}
		currenttime = ros::Time::now();

		if (count == 0)
		{
			headertime = currenttime;
		}
		timediff = (currenttime - headertime).toNSec();

		if (timediff > ROOMBATIMEOUT)
		{
			count = 0;
			ROS_ERROR("nx-base time out-%lld\n", timediff);
			headertime = currenttime;
		}
		if ((len + count) > 255)
		{
			count = 0;
			ROS_ERROR("nx-base receive data too long! Drop it!");
			return;
		}
		memcpy(recvbuf + count, buf, len);
		count += len;
	BACKCHECK:
		if (count > 2)
		{
			checkcount = count - 1;
			for (i = 0; i < checkcount; i++)
			{
				if ((recvbuf[i] == 'N') && (recvbuf[i + 1] == 'X'))
				{
					if (i > 0)
					{
						count = count - i;
						memcpy(tmpbuf, recvbuf + i, count);
						memcpy(recvbuf, tmpbuf, count);
					}
					break;
				}
			}
#if 0
			if (i != 0)
			{
				for (j = 0; j < count; j++)
					printf(L_GREEN "%02X " NONE, (unsigned char)recvbuf[j]);  //
				printf("\n");
			}
#endif
			if (i == checkcount)
			{
				if (recvbuf[checkcount] == 'N')
				{
					count = 1;
					recvbuf[0] = 'N';
				}
				else
				{
					count = 0;
				}
			}
			if (count > 4)
			{
				unsigned int framelen = (recvbuf[2] << 8) + recvbuf[3];
				if (framelen < 6)
				{
					count = 0;
				}
				else
				{
					if (count >= framelen)
					{
#if 1
						if (0)
						{
							for (j = 0; j < framelen; j++)
								printf("%02X ", (unsigned char)recvbuf[j]);
							printf("\n");
						}
#endif
						if ((recvbuf[0] == 'N') && (recvbuf[1] == 'X')) // check the header
						{
							if (checkSum(recvbuf) == recvbuf[framelen - 1])
							{
								callFunction(recvbuf[4], recvbuf + 5, len - 6);
							}
							else
							{
								ROS_ERROR("roombase-check sum error");
								for (j = 0; j < framelen; j++)
									printf(RED "%02X " NONE, (unsigned char)recvbuf[j]);
								printf("\n");
							}
						}
						else
						{
							ROS_ERROR("carbase-header error");
							for (j = 0; j < framelen; j++)
								printf(RED "%02X " NONE, (unsigned char)recvbuf[j]);
							printf("\n");
						}
						if (count > framelen)
						{
							memcpy(tmpbuf, recvbuf + framelen, count - framelen);
							memcpy(recvbuf, tmpbuf, count - framelen);
							count = count - framelen;
							headertime = currenttime;
							goto BACKCHECK;
						}
						count = 0;
					}
				}
			}
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "car_base");
	ros::NodeHandle _n("~");
	ROS_INFO("car_base_node for ROS %.2f", NODE_VERSION);
	sleep(2);
	STM32ComSwitchNode stmcsn(_n, argv[1]);
	stmcsn.startSerial(boost::bind(&STM32ComSwitchNode::getStm32ComData, &stmcsn, _1, _2), &stmcsn.serial_port_0_stm32, "/dev/ttyS0", 115200);
	if (stmcsn.motor_type == 1) // jzd电机
	{
		stmcsn.startSerial(boost::bind(&STM32ComSwitchNode::getComCanData, &stmcsn, _1, _2), &stmcsn.serial_port_3_motor, "/dev/CanBase", 460800);
	}
	else // hy电机
	{
		stmcsn.startSerial(boost::bind(&STM32ComSwitchNode::getCom3Data, &stmcsn, _1, _2), &stmcsn.serial_port_3_motor, "/dev/ttyS3", 57600);
	}
	ros::spin();
}
