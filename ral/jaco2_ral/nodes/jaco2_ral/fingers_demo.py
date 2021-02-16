#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco2_ral')
import rospy

import rospkg

from util.util import *
from jaco2_driver.fingers_action_client import *

import time

if __name__ == '__main__':

    rospy.init_node('jaco2_fingers_control')
    left_hand = Jaco2Fingers('left')
    left_hand.set_percent(100)