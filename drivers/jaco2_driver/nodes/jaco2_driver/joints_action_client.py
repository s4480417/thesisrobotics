#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse

# Left Home := [90, 180, 270, 180, 270, 0, 0]
# Right Home := [270, 180, 90, 180, 90, 0, 0]

class RobotArm:
    def __init__(self, arm, prefix="j2n6s300_",):
        self.arm = arm
        self.prefix = prefix
        self.currentJointCommand = [0] * 7
        self.getcurrentJointCommand()

    def set_angles(self, joint_degree, relative=False):
        if relative:
            joint_degree_absolute = [joint_degree[i] + self.currentJointCommand[i] for i in range(0, len(joint_degree))]
        else:
            joint_degree_absolute = joint_degree
        result = self.joint_angle_client(joint_degree_absolute, self.arm)
        return result

    def getcurrentJointCommand(self):
        # wait to get current position
        topic_address = '/' + self.arm + '_driver/out/joint_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, self.setcurrentJointCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
        print 'position listener obtained message for joint position. '


    def setcurrentJointCommand(self, feedback):

        currentJointCommand_str_list = str(feedback).split("\n")
        for index in range(0, len(currentJointCommand_str_list)):
            temp_str=currentJointCommand_str_list[index].split(": ")
            self.currentJointCommand[index] = float(temp_str[1])


    def joint_angle_client(self, angle_set, arm):
        """Send a joint angle goal to the action server."""
        action_address = '/' + arm + '_driver/joints_action/joint_angles'
        client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.ArmJointAnglesAction)
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