#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco2_ral')
import rospy

import rospkg

from util.util import *
from jaco2_driver.joints_action_client import *

import time


LEFT_HOME = [90, 180, 270, 180, 270, 100, 0]
RIGHT_HOME = [270, 180, 90, 180, 90, 100, 0]

if __name__ == '__main__':

    rospy.init_node('jaco2_joint_control')

    left_arm = Jaco2Arm('left')
    right_arm = Jaco2Arm('right')
    try:
        left_arm.set_angles(LEFT_HOME)
        right_arm.set_angles(RIGHT_HOME)
        print("LEFT:")
        print(left_arm.set_angles([0, 0, 0, 0, 0, 0, 0], relative=True))
        print("RIGHT:")
        print(right_arm.set_angles([0, 0, 0, 0, 0, 0, 0], relative=True))

    except rospy.ROSInterruptException:
        print('program interrupted before completion')