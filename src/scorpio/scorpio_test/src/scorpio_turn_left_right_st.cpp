#include "ros/ros.h"
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MotionTest
{
public:
  ros::NodeHandle nhandle;
  boost::thread* test_motion_thread_;
  ros::Publisher cmd_pub;
  MotionTest(ros::NodeHandle in_nh)
  {
    nhandle = in_nh;

    cmd_pub = nhandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    test_motion_thread_ = new boost::thread(boost::bind(&MotionTest::testMotion, this));
  }

  ~MotionTest()
  {
    if (test_motion_thread_ != NULL)
    {
      delete test_motion_thread_;
    }
  }

  bool testMotion()
  {
    ros::Rate rate(10);
    sleep(5);
    double start_sec = ros::Time().now().toSec();
    int keep_sec = 300;
    ROS_INFO("time is start! start turning.");      
    while(ros::ok)
    {
        double prev_sec = ros::Time().now().toSec();
        int sec = 10;
        ROS_INFO("ni shi zhen");
        while (ros::ok)
        {
          //逆时针旋转
          if (ros::Time().now().toSec() - prev_sec > sec)
            break;

          testTurnBody(0.5, 2, 10);
          rate.sleep();
        }
        ROS_INFO("shen shi zhen");
        prev_sec = ros::Time().now().toSec();
        while (ros::ok)
        {

          //顺时针旋转
          if (ros::Time().now().toSec() - prev_sec > sec)
            break;

          testTurnBody(0.5, -2, 10);
          rate.sleep();
        }
	if (ros::Time().now().toSec() - start_sec > keep_sec)
        {
           ROS_WARN("time is over! stop turning.");      
           break;
        }
    }

  }

  void testTurnBody(float linearx, float angularz, float sec)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = linearx;
    vel.angular.z = angularz;
    cmd_pub.publish(vel);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scorpio_test_node_st");
  ROS_INFO("scorpio test bring up");

  ros::NodeHandle n;
  MotionTest test(n);

  // start to test the scorpio
  // test.testTopicStatus();
  //ROS_INFO("scorpio test stop");
  ros::spin();
  return 0;
}
