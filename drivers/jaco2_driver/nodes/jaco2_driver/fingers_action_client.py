#! /usr/bin/env python
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys

import actionlib
import kinova_msgs.msg

import math

import argparse

from abc import abstractmethod


finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger


class RobotFingers:

    @abstractmethod
    def set_percent(self, percent, relative=False):
        pass
    
    @abstractmethod
    def get_percent(self, percent):
        pass

class Jaco2Fingers(RobotFingers):

    def __init__(self, hand, prefix="j2n6s300_", verbose=False):
        """ Initialize fingers """
        self.verbose = verbose                  # Display Logging Information
        self.hand = hand                        # Left or right hand
        self.prefix = prefix                    # Kinova Robot Prefix (default Jaco2 6-DOF 3-Finger)
        self.currentFingerPosition = [0] * 3    # Default finger position (float: 0.0 - 1.0)
        self.__get_currentFingerPosition()      # Retrieve actual finger position from server

    def set_percent(self, percent):
        result = self.__gripper_client(percent)
        return result

    def get_percent(self):
        self.__get_currentFingerPosition()
        return self.currentFingerPosition
    
    def __get_currentFingerPosition(self):
        topic_address = '/' + self.hand + '_driver/out/finger_position'
        #rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, self.__set_currentFingerPosition)
        #rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
        print 'obtained current finger position '

    def __set_currentFingerPosition(self, feedback):
        self.currentFingerPosition[0] = feedback.finger1
        self.currentFingerPosition[1] = feedback.finger2
        self.currentFingerPosition[2] = feedback.finger3
        
    def __gripper_client(self, percent):
        """Send a gripper goal to the action server."""
        action_address = '/' + self.hand + '_driver/fingers_action/finger_positions'

        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(percent[0])
        goal.fingers.finger2 = float(percent[1])
        goal.fingers.finger3 = float(percent[2])
        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(5.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')
            return None


