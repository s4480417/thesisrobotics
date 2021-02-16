#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse

from abc import abstractmethod

# Left Home := [90, 180, 270, 180, 270, 0, 0]
# Right Home := [270, 180, 90, 180, 90, 0, 0]

class RobotJoints:

    @abstractmethod
    def set_angles(self, joint_degree, relative=False):
        pass
    
    @abstractmethod
    def get_angles(self):
        pass

class Jaco2Joints(RobotJoints):
<<<<<<< HEAD

=======
>>>>>>> b45512c5000c07f6d799a427b45014732cc9fe7e
    def __init__(self, arm, prefix="j2n6s300_",):
        """ Initialize Jaco2 6-DOF 3-Finger Robot """
        self.arm = arm
        self.prefix = prefix
        self.currentJointCommand = [0] * 7
        self.__get_currentJointCommand()

    def set_angles(self, joint_degree, relative=False):
        """ Set angles of robot """
        if relative:
            joint_degree_absolute = [joint_degree[i] + self.currentJointCommand[i] for i in range(0, len(joint_degree))]
        else:
            joint_degree_absolute = joint_degree
        result = self.__joint_angle_client(joint_degree_absolute)
        return result
    
    def get_angles(self):
        self.__get_currentJointCommand()
        return self.currentJointCommand


    def __get_currentJointCommand(self):
        """ Get the current joint command (internal) """
        topic_address = '/' + self.arm + '_driver/out/joint_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, self.__set_currentJointCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
        print 'position listener obtained message for joint position. '


    def __set_currentJointCommand(self, feedback):
        """ Set the current joint command (internal) """
        currentJointCommand_str_list = str(feedback).split("\n")
        for index in range(0, len(currentJointCommand_str_list)):
            temp_str = currentJointCommand_str_list[index].split(": ")
            self.currentJointCommand[index] = float(temp_str[1])


    def __joint_angle_client(self, angle_set):
        """Send a joint angle goal to the action server."""
        action_address = '/' + self.arm + '_driver/joints_action/joint_angles'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            return None
