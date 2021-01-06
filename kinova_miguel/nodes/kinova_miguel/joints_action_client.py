#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_miguel')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse

# Left Home := [90 180 270 180 270 0]
# Right Home := [270 180 90 180 90 0]

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [] # number of joints is defined in __main__
arm = 'left'

def joint_angle_client(angle_set):
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
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


def getcurrentJointCommand(prefix_):
    # wait to get current position
    topic_address = '/' + arm + '_driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'position listener obtained message for joint position. '


def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])

    # print 'currentJointCommand is: '
    # print currentJointCommand


def argumentParser(argument):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    parser.add_argument('arm', metavar='arm', type=str, default='left', choices={'left', 'right'}, help='Choose which arm to move (left or right)')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('joint_value', nargs=6, type=float)

    args_ = parser.parse_args(argument)
    return args_


def kinova_robotTypeParser():
    """ Argument kinova_robotType """
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


def unitParser(joint_value, relative_):
    """ Argument unit """
    global currentJointCommand

    joint_degree_command = joint_value
    # get absolute value
    if relative_:
        joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
    else:
        joint_degree_absolute_ = joint_degree_command
    joint_degree = joint_degree_absolute_
    return joint_degree

def armParser(arm_):
    global arm
    arm = arm_

if __name__ == '__main__':

    args = argumentParser(None)

    kinova_robotTypeParser()
    armParser(args.arm)
    rospy.init_node(arm + '_arm_workout')

    # currentJointCommand = [0]*arm_joint_number
    # KinovaType defines AngularInfo has 7DOF, so for published topics on joints.
    currentJointCommand = [0] * 7

    # get Current finger position if relative position
    getcurrentJointCommand(prefix)
    joint_degree = unitParser(args.joint_value, args.relative)

    positions = [0]*7

    try:
        positions = joint_degree + [0]
        print('Sending arm position to ' + arm + ' arm...')
        result = joint_angle_client(positions)
        print('Arm position sent!')
        print(result)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')