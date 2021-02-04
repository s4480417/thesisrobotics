#! /usr/bin/env python
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys

import actionlib
import kinova_msgs.msg

import math

import argparse


""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]
hand = 'left'

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + hand + '_driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None


def getCurrentFingerPosition(prefix_):
    # wait to get current position
    topic_address = '/' + hand + '_driver/out/finger_position'
    rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
    print 'obtained current finger position '


def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive fingers to command position')
    parser.add_argument('hand', metavar='hand', type=str, default='left', choices={'left', 'right'}, help='Choose which hand to move (left or right)')
    parser.add_argument('finger_value', type=float)
    args_ = parser.parse_args(argument_)
    return args_


def kinova_robotTypeParser():
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = 'j'
    robot_category_version = 2
    wrist_type = 'n'
    arm_joint_number = 6
    robot_mode = 's'
    finger_number = 3
    prefix = "j2n6s300" + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger

def handParser(hand_):
    global hand
    hand = hand_

def unitParser(finger_value_):
    """ Argument unit """
    global currentFingerPosition
    finger_turn_ = [finger_value_/100.0 * finger_maxTurn] * 3
    return finger_turn_

if __name__ == '__main__':
    
    args = argumentParser(None)

    kinova_robotTypeParser()
    handParser(args.hand)
    rospy.init_node(hand + '_gripper_workout')

    getCurrentFingerPosition(prefix)

    finger_turn = unitParser(args.finger_value)

    try:
        positions_temp1 = [max(0.0, n) for n in finger_turn]
        positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
        positions = [float(n) for n in positions_temp2]
        print('Sending finger position to ' + hand + ' hand')
        result = gripper_client(positions)
        print('Finger position sent!')
        print(result)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')