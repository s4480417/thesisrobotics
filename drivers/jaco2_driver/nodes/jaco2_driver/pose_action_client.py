#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from util.util import *


import math
import argparse

from abc import abstractmethod

def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_

class RobotPose:
    @abstractmethod
    def set_cartesian(self, pose):
        pass

    @abstractmethod
    def get_cartesian(self):
        pass


class Jaco2Pose(RobotPose):
    
    def __init__(self, arm, prefix="j2n6s300_"):
        """ Initialize Jaco2 6-DOF 3-Finger Robot """
        self.arm = arm
        self.prefix = prefix
        self.currentCartesianCommand = [0] * 6
        self.__get_currentCartesianCommand()
    
    def set_cartesian(self, pose, relative=False):
        """ Set angles of robot """
        position = pose[:3]
        orientation = pose[3:]

        if relative:
            position_absolute = [position[i] + self.currentCartesianCommand[i] for i in range(3)]

            orientation_deg_list = list(map(math.degrees, self.currentCartesianCommand[3:]))
            orientation_deg = [orientation[i] + orientation_deg_list[i] for i in range(3)]
            orientation_rad = list(map(math.radians, orientation_deg))
            orientation_q = EulerXYZ2Quaternion(orientation_rad)
            orientation_absolute = orientation_q

        else:
            position_absolute = position

            orientation_deg = orientation
            orientation_rad = list(map(math.radians, orientation_deg))
            orientation_q = EulerXYZ2Quaternion(orientation_rad)
            orientation_absolute = orientation_q

        result = self.__cartesian_pose_client(position_absolute, orientation_absolute)
        return result

    def get_cartesian(self):
        return self.__get_currentCartesianCommand()

    def __get_currentCartesianCommand(self):
        topic_address = '/' + self.arm + '_driver/out/cartesian_command'
        #rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.__set_currentCartesianCommand)
        #rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
        print 'position listener obtained message for Cartesian pose. '

    def __set_currentCartesianCommand(feedback):
        currentCartesianCommand_str_list = str(feedback).split("\n")
        for index in range(0, len(currentCartesianCommand_str_list)):
            temp_str = currentCartesianCommand_str_list[index].split(": ")
            self.currentCartesianCommand[index] = float(temp_str[1])

    def __cartesian_pose_client(self, position, orientation):
        """Send a cartesian goal to the action server."""
        action_address = '/' + self.arm + '_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(self.arm + '_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None

