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
 */
#define NODE_VERSION 0.01
#include <ros/ros.h>
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
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))
using namespace std;
#define ANGLE_MIDDLE_POINT 1118			//right--1080++left
#define ROOMBATIMEOUT (3000*1e6)
#define PI 3.141592654
#define WD 0.105
#define COUNT_TIMES 20
#define MAX_SPEED 1.00

/*****************************************************************************
	@func :Calculate CRC16-MODBUS@poly :8005(x16+x15+x2+1)
	@init :0xFFFF
	@xorout :0x0000
	@refin :yes
	@refout :yes
	*****************************************************************************/
unsigned char CRCH[256] =
{
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40
};
	
unsigned char CRCL[256] =
{
	0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
	0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
	0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
	0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
	0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
	0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
	0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
	0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
	0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
	0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
	0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
	0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
	0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
	0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
	0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};
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
	float current_speed;
	int cur_pwm;
	int new_vel_bit;
	int overCurrent;
	double odometry_x_ ;
	double odometry_y_ ;
	double odometry_yaw_ ;
	//Cereal port object
	cereal::CerealPort * serial_port_0_stm32;
	cereal::CerealPort * serial_port_3_motor;
	// *****************************************************************************
	// Constructor
	STM32ComSwitchNode(ros::NodeHandle _n, const char * new_serial_port)
	{
		n = _n;
		overCurrent = 0;
		new_vel_bit = 0;
		current_speed = 0;
		odometry_x_ = 0;
		odometry_y_ = 0;
		odometry_yaw_ = 0;
		serial_port_0_stm32 = new cereal::CerealPort();
		serial_port_3_motor = new cereal::CerealPort();
		getPtrFunction();
		//	ros::param::get("~hall_encoder",hall_encoder);
		if (n.getParam("hall_encoder", hall_encoder)) 
		{
			if(hall_encoder)
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
		if(n.getParam("limited_speed", limited_speed))
		{
			if(limited_speed > MAX_SPEED)
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
		if((*th) != NULL)
		{
			(*th)->interrupt();
			(*th)->join();
			delete (*th);
			(*th) = NULL;
			return true;
		}
		return true;
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
		int pwml=1080, pwma=ANGLE_MIDDLE_POINT;
		unsigned char buf[20];
		float dz;
		if(z>1)
		  z=1;
		dz=-180*z/M_PI*6;
		if(dz>0)			//right转
		{
			pwma = ANGLE_MIDDLE_POINT-dz;
			if(pwma<800)	//770
			pwma = 800;	//770
		}
		else if(dz<0)	//left转
		{
			pwma = ANGLE_MIDDLE_POINT-dz;
			if(pwma>1460)	//1230
			pwma = 1460;	//1230
		}
		buf[0] = pwml >> 8;
		buf[1] = pwml;
		buf[2] = pwma >> 8;
		buf[3] = pwma;
		writeData(0x01, buf, 4);
	}

	//type:00 is bottom switch,01 is motor power
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
		if(first_time)
		{
			write_config_Data(0x0006, 0x0001,0x00);
			first_time = 0;
			sleep(1);
			sub_acker_vel = n.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, &STM32ComSwitchNode::ackerMannCmdVelReceived, this);	
			return ;
		}
		if (countSerial == lastCountSerial)
		{
			rcvPwmFun(0, 0);			
			if(swap_bit) 
			{
				write_config_Data(0x0006, 0x0001,0x00);
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
		if(overCurrent)
		{
				ovcnt++;
				if(ovcnt > 4)
				{
					startCloseCmd(0x01, 0x01); //close motor power
				}
				else if(ovcnt > 2)
				{
					startCloseCmd(0x01, 0x00); //close motor power
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
	void write_vel2motor(float vel)
	{
		short mv ;
		if(vel>limited_speed)
			vel = limited_speed;
		else if (vel<-limited_speed)
			 vel = -limited_speed;
		mv = vel *11800;
		read_write_Data(0x002a, 0x0001, 0x002B, 0x0001, mv);
	}

	int read_write_Data(unsigned short read_addr, unsigned short read_len, unsigned short write_addr, unsigned short write_len,  short  vel)
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
		buffer[2] = read_addr>>8;          
		buffer[3] = read_addr;             
		buffer[4] = read_len>>8; 
		buffer[5] = read_len; 
		buffer[6] = write_addr>>8;          
		buffer[7] = write_addr;             
		buffer[8] = write_len>>8; 
		buffer[9] = write_len; 		
		buffer[10] = 0x02; 		
		buffer[11] = vel>>8; 
		buffer[12] = vel; 
		crc_word = CalculateCRC16(buffer, 13);
		buffer[13] = crc_word; 
		buffer[14] = crc_word>>8; 				
	//	    for(int i=0; i<15; i++)
	//			printf("%02x ",buffer[i]);
	//	    printf("\n"); 
		 
		try{ s3_mutex.lock();serial_port_3_motor->write((char *)buffer, 15); s3_mutex.unlock();}
		catch(cereal::Exception& e){ return(-1); }	   
		return(0);
	}
	int write_config_Data(unsigned short write_addr, unsigned short write_len,  short  dat)
	{
		unsigned int i;
		unsigned char sum = 0;
		unsigned char buffer[40];
		unsigned short crc_word;

		buffer[0] = 0x01;               
		buffer[1] = 0x10;               
		buffer[2] = write_addr>>8;          
		buffer[3] = write_addr;             
		buffer[4] = write_len>>8; 
		buffer[5] = write_len; 		
		buffer[6] = write_len*2; 		
		buffer[7] = dat>>8; 
		buffer[8] = dat; 
		crc_word = CalculateCRC16(buffer, 9);
		buffer[9] = crc_word; 
		buffer[10] = crc_word>>8; 				
	//	   for(int i=0; i<11; i++)
	//			printf("%02x ",buffer[i]);
	//	   printf("\n"); 
		try{s3_mutex.lock(); serial_port_3_motor->write((char *)buffer, 11); s3_mutex.unlock();}
		catch(cereal::Exception& e){ return(-1); }   
		return(0);
	}
	
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
		return (unsigned short)((unsigned short)(crcHigh<<8) | crcLow);
	}

	void motorSendData(const ros::TimerEvent &event)
	{
		float vel;
		int newbit = 0;
		t_mutex.lock();
		if(new_vel_bit)
		{
			vel = current_speed;
			newbit = 1;
			new_vel_bit = 0;
		}
		t_mutex.unlock();
		if(newbit)
			write_vel2motor(vel);
	}	
	
	void getPtrFunction()
	{
		func_map[0x0000]=&STM32ComSwitchNode::nullFun;
		func_map[0x01]=&STM32ComSwitchNode::baseFun;
	}
	
	void callFunction(int index, unsigned char *recvbuf, int len)
	{
		if(func_map.count (index))
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
		static   double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
		  odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
		float speed;
		float acvx,acvy,acvz,anvx,anvy,anvz,roll,pitch,yaw;
		float qx,qy,qz,qw;
		sensor_msgs::Imu car_imu;
		tf::Quaternion q;
		int curr_idx = (idx + COUNT_TIMES - 1) % COUNT_TIMES;
		struct timeval tv;
		gettimeofday(&tv, NULL);
		long long ts = (long long)tv.tv_sec*1000 + tv.tv_usec/1000;
		current_time = ros::Time::now();  // ros time

		fb_time[curr_idx] = ts;  // set spark base time which is different from ros time



		timesec = (buf[30]<<24)|(buf[31]<<16)|(buf[32]<<8)|buf[33];
		curpwm = (buf[26]<<24)|(buf[27]<<16)|(buf[28]<<8)|buf[29];
		if(first_time)
		{
			lastpwm = curpwm;
			first_time = 0;
		}
		cur_pwm = curpwm;
		double cur_dist = PI*WD*(curpwm-lastpwm)*5/574;
		lastpwm = curpwm;
			//ROS_INFO("the speed is %fm/s", speed);
		acvx = (float(short((buf[1]<<8)|buf[0]))/32768*16*9.8);// m/s^2
		acvy = (float(short((buf[3]<<8)|buf[2]))/32768*16*9.8);
		acvz = (float(short((buf[5]<<8)|buf[4]))/32768*16*9.8);

		anvx = (float(short((buf[7]<<8)|buf[6]))/32768*2000);
		anvy = (float(short((buf[9]<<8)|buf[8]))/32768*2000);
		anvz = (float(short((buf[11]<<8)|buf[10]))/32768*2000);

		roll  = (float(short((buf[13]<<8)|buf[12])))/32768*M_PI;
		pitch = (float(short((buf[15]<<8)|buf[14])))/32768*M_PI;
		yaw   = (float(short((buf[17]<<8)|buf[16])))/32768*M_PI;
	//	printf("yaw is %f\n", yaw);
		qx = (float(short((buf[19]<<8)|buf[18]))/32768);	
		qy = (float(short((buf[21]<<8)|buf[20]))/32768);
		qz = (float(short((buf[23]<<8)|buf[22]))/32768);
		qw = (float(short((buf[25]<<8)|buf[24]))/32768);

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
		car_imu.orientation_covariance[0] = pow(0.0017, 2);//
		car_imu.orientation_covariance[4] = pow(0.0017, 2);
		car_imu.orientation_covariance[8] = pow(0.0017, 2);


		car_imu.angular_velocity.x = anvx*M_PI/180.0;// rad/s
		car_imu.angular_velocity.y = anvy*M_PI/180.0;
		car_imu.angular_velocity.z = anvz*M_PI/180.0;
		car_imu.angular_velocity_covariance[0] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[4] = pow(0.1, 2);
		car_imu.angular_velocity_covariance[8] = pow(0.1, 2);

		car_imu.linear_acceleration.x = acvx;// m/s^2
		car_imu.linear_acceleration.y = acvy;
		car_imu.linear_acceleration.z = acvz;
		car_imu.linear_acceleration_covariance[0] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[4] = pow(0.1, 2);
		car_imu.linear_acceleration_covariance[8] = pow(0.1, 2);

		car_imu.header.stamp = ros::Time::now();
		car_imu.header.frame_id = "IMU_link";
		pub_imu.publish(car_imu);



		if(hall_encoder)
		{
			// Update odometry
			odometry_x_ = odometry_x_ + cur_dist * cos(odometry_yaw_);  // m
			odometry_y_ = odometry_y_ + cur_dist * sin(odometry_yaw_);  // m
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

			vel_y = 0;  //(odom_y[curr_idx] - odom_y[idx])/dt;

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

		   //printf("odom x: %f, y: %f, z: %f,  w: %f\n",odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odometry_yaw_);
		  // publish the feedback's twist message from the car base
			pub_fback_cmd_vel.publish(odom.twist.twist);
		}
	}

	int writeData(unsigned char cmd, unsigned char *buf, unsigned int len)
	{
		// Compose comand
		unsigned int i;
		unsigned char sum = 0;
		unsigned char buffer[5024];
		buffer[0] = 'N';               
		buffer[1] = 'X';               
		buffer[2] = (len+6)>>8;          
		buffer[3] = len+6;             
		buffer[4] = cmd; 
		for(i=0; i<len; i++)
		{
			buffer[5+i] = buf[i];
		}	
		for(i=0; i<len+5; i++)
		{
			sum = sum+buffer[i];
		}   
		buffer[5+len] = sum;                     
		// for(int i=0; i<length; i++)
		//    printf("%02x ",buffer[i]);
		//    printf("\n"); 
		try{ serial_port_0_stm32->write((char *)buffer, len+6); }
		catch(cereal::Exception& e){ return(-1); }
		return(0);
	}


	void startSerial(boost::function<void(char*, int)> f, cereal::CerealPort ** serial_port_, std::string port_name_, int port)
	{
		if( openSerialPort(f, serial_port_, port_name_, port) == 0)
		{
			ROS_INFO("Connected to Scorpio base successfully.");
			startCloseCmd(0x00, 1);
		}
		else
		{
			ROS_FATAL("Could not connect to Scorpio base.");
			ROS_BREAK();
		}
	}



	// *****************************************************************************
	// Open the serial port
	int openSerialPort(boost::function<void(char*, int)> f, cereal::CerealPort ** serial_port_, std::string port_name_, int port)
	{
		try{ (*serial_port_)->open(port_name_.c_str(), port); }
		catch(cereal::Exception& e){ return(-1); }
		if((*serial_port_)->startReadStream(f) != true)
		{
			closeSerialPort(serial_port_);
			return(-1);
		}
		return(0);
	}
	// *****************************************************************************
	// Close the serial port
	int closeSerialPort(cereal::CerealPort ** serial_port_)
	{
		(*serial_port_)->stopStream();
		try{ (*serial_port_)->close(); }
		catch(cereal::Exception& e){ return(-1); }
		return(0);
	}

	// *****************************************************************************
	// check sum
	unsigned char checkSum(unsigned char *buf)
	{
		unsigned char sum=0;
		int i;
		int len = (buf[2]<<8)+buf[3];
		for(i=0; i<len-1; i++)
		{
			sum += buf[i];
		}
		return sum;
	}

	void getCom3Data(char *buf_r, int len)
	{
		int i;
		unsigned short crc_word;
		unsigned char* buf;
		buf = (unsigned char *)buf_r;
		/*for(i=0; i<len; i++)
			printf("%02x ", (buf[i]));
		printf("\n");*/
		if((buf[0] == 1)&&(len > 3))
		{
			if(buf[1] == 0x17)
			{
				if((buf[2]+5) == len)
				{
					crc_word = CalculateCRC16((unsigned char *)buf, 5);
					//	printf("%04x\n", crc_word);
					if((unsigned short)(buf[len-2]+(buf[len-1]<<8)) == crc_word)
					{
						if(buf[4]&0x01)
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
				unsigned int framelen = (recvbuf[2]<<8) + recvbuf[3];
				if (framelen < 6)
				{
					count = 0;
				}
				else
				{
					if (count >= framelen)
					{
						#if 1
						if(0)
						{
							for(j=0;j<framelen; j++)
								printf("%02X ",(unsigned char)recvbuf[j]);
							printf("\n");
						}
						#endif
						if ((recvbuf[0] == 'N') && (recvbuf[1] == 'X'))  // check the header 
						{
							if(checkSum(recvbuf) == recvbuf[framelen-1])
							{
								callFunction(recvbuf[4], recvbuf+5, len-6);
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "car_base");
	ros::NodeHandle _n("~");
	ROS_INFO("car_base_node for ROS %.2f", NODE_VERSION);
	sleep(2);
	STM32ComSwitchNode stmcsn(_n, argv[1]);
	stmcsn.startSerial(boost::bind(&STM32ComSwitchNode::getStm32ComData, &stmcsn, _1, _2), &stmcsn.serial_port_0_stm32, "/dev/ttyS0", 115200);
	stmcsn.startSerial(boost::bind(&STM32ComSwitchNode::getCom3Data, &stmcsn, _1, _2), &stmcsn.serial_port_3_motor, "/dev/ttyS3", 57600);
	ros::spin();
}
