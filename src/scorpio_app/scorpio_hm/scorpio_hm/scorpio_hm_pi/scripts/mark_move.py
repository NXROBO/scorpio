#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import tf
import tf.transformations as tf_transformations
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np

class MarkNav():
    def __init__(self):
        # 初始化节点
        rospy.init_node('MarkNav')
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))
        # 订阅标记事件
        rospy.Subscriber('mark_nav', String, self.mark_nav)
        # 定义标记地点字典
        global dict_mark
        dict_mark = {}
        # 监听TF坐标
        self.listener = tf.TransformListener()
        rospy.sleep(1) # need to delay
    
    def mark_nav(self,mark_name):
        '''
        设定标定地点，与后面导航相关，采用字典的方式存放
        '''
        # 转化成字符型变量
        mark_name = str(mark_name)
        print("当前内容：\n",mark_name)

        # 前往地点    
        if str.find(mark_name,str("go"))!= -1:
            for key in dict_mark.keys():
                print("target", key)
                self.navigation(key)

        # 学习地点
        if str.find(mark_name,str("plan"))!= -1:
            # 提取mark名称
            mark_name = (mark_name.split())[2].strip('"')
            # 获取当前位置
            self.currrent_position = self.get_currect_pose()
            # 标记当前位置
            dict_mark[mark_name] = self.currrent_position
            print("当前字典内容：\n",dict_mark)

    # 获取当前位置
    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        print("target_trans:",target_trans)
        print("target_rot:",target_rot)
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))
    
    def navigation(self, mark_name):
        '''
        根据地点进行导航
        '''
        print("start navigation")
        # movebase初始化
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # 设定目标地点
        self.drop_position = dict_mark[mark_name]
        tran = tf_transformations.translation_from_matrix(self.drop_position)
        quat = tf_transformations.quaternion_from_matrix(self.drop_position)
        pose = Pose(Point(tran[0], tran[1], tran[2]),
                    Quaternion(quat[0], quat[1], quat[2], quat[3]))
        goal.target_pose.pose = pose
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)

        # ===== 3分钟阻塞等待导航结果 =====
        reached = self.move_base.wait_for_result(rospy.Duration(180.0))
        if not reached:
            # 超时：取消目标
            rospy.logwarn("导航超时(>180s)，取消当前目标: %s" % mark_name)
            self.move_base.cancel_goal()
            return False   # 导航失败

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("已到达目标点: %s" % mark_name)
            return True    # 导航成功
        else:
            rospy.logwarn("导航失败，状态码: %d, 目标: %s" % (state, mark_name))
            return False

if __name__ == '__main__':
    try:
        MarkNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mark_move finished.")
