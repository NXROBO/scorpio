#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tf_transformations
import math
import numpy as np
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction, MoveStraightDistanceGoal, TurnBodyDegreeGoal, ShiftNav


class MarkNav():
    def __init__(self):
        # 初始化节点
        rospy.init_node('spark_nav_shift_node')
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 订阅标记事件
        rospy.Subscriber('shift_nav_cmd', ShiftNav, self.excute_CB)
        # 监听TF坐标
        self.listener = tf.TransformListener()
        rospy.sleep(1)  # need to delay

    # 获取当前位置
    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform(
            "map", "base_footprint", rospy.Time(0))
        # print("target_trans:",target_trans)
        # print("target_rot:",target_rot)
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))

    def _move_straight(self, distance, vel=0.1, timeout=30):
        # Creates a goal to send to the action server.
        goal = MoveStraightDistanceGoal(
            type=MoveStraightDistanceGoal.TYPE_ODOM,
            move_distance=distance,
            const_rot_vel=vel
        )
        self.move_action_cli.send_goal_and_wait(
            goal, rospy.Duration.from_sec(timeout))

        rospy.sleep(0.5)
        return self.move_action_cli.get_result()  # A FibonacciResult

    def _turn_body(self, degree, vel=0.25, timeout=30, is_const=True):
        if(degree > 180):
            degree -= 360
        elif(degree < -180):
            degree += 360

        # Creates a goal to send to the action server.
        goal = TurnBodyDegreeGoal(
            is_const_vel=is_const,
            goal_degree=degree,
            const_rot_vel=vel
        )

        self.turn_action_cli.send_goal_and_wait(
            goal, rospy.Duration.from_sec(timeout))

        rospy.sleep(0.5)
        return self.turn_action_cli.get_result()  # A FibonacciResult

    def move_demo(self, shift_pose: Pose2D):
        ''' 以当前位置为原点, 移动 Spark 到目标点去
        '''
        px = shift_pose.x
        py = shift_pose.y
        angle = math.degrees(shift_pose.theta)

        if px > 0:
            tmp_angle = math.atan2(py, px) * 360 / math.pi / 2
        else:
            tmp_angle = math.atan2(-py, -px) * 360 / math.pi / 2
        mileage = math.sqrt(px**2 + py**2)

        rospy.loginfo("Move to the target...")
        rospy.loginfo(f"where the current position is displaced by x={shift_pose.x} y={shift_pose.y} theta={shift_pose.theta}")
        self._turn_body(degree=tmp_angle)
        if px > 0:
            self._move_straight(distance=mileage, vel=0.1,
                           timeout=mileage/0.1*1.25)
        else:
            self._move_straight(distance=mileage, vel=-0.1,
                            timeout=mileage/0.1*1.25)
        self._turn_body(degree=angle-tmp_angle)

    def navigation(self, shift_pose: Pose2D):
        '''
        根据地点进行导航
        '''
        # print("start navigation")
        # movebase初始化
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # 设定目标地点
        current_pose = self.get_currect_pose()
        target_pose = np.dot(current_pose, tf_transformations.compose_matrix(
            translate=[shift_pose.x, shift_pose.y, 0], angles=[0, 0, shift_pose.theta]))
        tran = tf_transformations.translation_from_matrix(target_pose)
        quat = tf_transformations.quaternion_from_matrix(target_pose)
        pose = Pose(Point(tran[0], tran[1], tran[2]),
                    Quaternion(quat[0], quat[1], quat[2], quat[3]))
        goal.target_pose.pose = pose

        # 把目标位置发送给MoveBaseAction的服务器
        rospy.loginfo("Navigate to the target...")
        rospy.loginfo(f"where the current position is displaced by x={shift_pose.x} y={shift_pose.y} theta={shift_pose.theta}")
        self.move_base.send_goal_and_wait(goal, rospy.Duration.from_sec(30))

    def turn_body(self, shift_pose: Pose2D):
        angle = math.degrees(shift_pose.theta)
        rospy.loginfo("turn body...")
        rospy.loginfo(f"where the current position is displaced by theta={angle}")
        self._turn_body(degree=angle)

    def excute_CB(self, sn: ShiftNav):
        if sn.type == 0:
            rospy.loginfo("run nav")
            self.navigation(sn.pose)
        elif sn.type == 1:
            self.move_demo(sn.pose)
        elif sn.type == 2:
            self.turn_body(sn.pose)


if __name__ == '__main__':
    try:
        MarkNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Spark_Nav_Shift finished.")
