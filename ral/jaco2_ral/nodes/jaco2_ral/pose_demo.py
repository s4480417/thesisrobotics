#! /usr/bin/env python


import roslib; roslib.load_manifest('jaco2_ral')
import rospy

import rospkg

from util.util import *
from jaco2_driver.pose_action_client import *

import time

if __name__ == '__main__':

    rospy.init_node('jaco2_pose_control')