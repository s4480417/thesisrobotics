#! /usr/bin/env python


import roslib; roslib.load_manifest('jaco2_ral')
import rospy

import rospkg

from util.util import *
from jaco2_driver.joints_action_client import *

import time
import math
import copy


LEFT_HOME = [90, 180, 270, 180, 270, 100, 0]
RIGHT_HOME = [270, 180, 90, 180, 90, 100, 0]

if __name__ == '__main__':

    rospy.init_node('jaco2_joint_control')

    left_arm = Jaco2Joints('left')
    right_arm = Jaco2Joints('right')
    try:
        left_arm.set_angles(LEFT_HOME)
        right_arm.set_angles(RIGHT_HOME)
        print("LEFT:")
        N = 25
        for i in range(N):
            t = i / float(N)
            off = 25.0 * math.sin(2.0 * math.pi * t)
            angles = copy.copy(LEFT_HOME)
            angles[1] += off
            angles[2] -= off
            angles[3] += off
            print(left_arm.set_angles(angles, relative=False))
        for i in range(5):
            print(left_arm.set_angles([10, 0, 0, 0, 0, 0, 0], relative=True))
        left_arm.set_angles(LEFT_HOME)
        print("RIGHT:")
        for i in range(N):
            t = i / float(N)
            off = 25.0 * math.sin(2.0 * math.pi * t)
            angles = copy.copy(RIGHT_HOME)
            angles[1] -= off
            angles[2] += off
            angles[3] -= off
            print(right_arm.set_angles(angles, relative=False))
        for i in range(5):
            print(right_arm.set_angles([-10, 0, 0, 0, 0, 0, 0], relative=True))
        right_arm.set_angles(RIGHT_HOME)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')