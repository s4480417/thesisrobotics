#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse

from abc import abstractmethod

import numpy as np


delta_Jx = 0.15675
delta_Jy = 0.13335
delta_Kx = 0.125
delta_Kz = 0.3

T_0K = np.array([
    [0, 0, 1, delta_Kx],
    [1, 0, 0, 0],
    [0, -1, 0, delta_Kz],
    [0, 0, 0, 1]
])

T_0L = np.array([
    [0, 0, 1, -delta_Jx],
    [-1, 0, 0, delta_Jy],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])


T_0R = np.array([
    [0, 0, 1, -delta_Jx],
    [-1, 0, 0, -delta_Jy],
    [0, -1, 0, 0],
	[0, 0, 0, 1]
])

def origin_to_kinect(v0):
    vK = np.matmul(np.linalg.inv(T_0K), v0)
    return vK

def origin_to_left(v0):
    vL = np.matmul(np.linalg.inv(T_0L), v0)
    return vL

def origin_to_right(v0):
    vR = np.matmul(np.linalg.inv(T_0R), v0)
    return vR

def kinect_to_origin(vK):
    v0 = np.matmul(T_0K, vK)
    return v0

def left_to_origin(vL):
    v0 = np.matmul(T_0L, vL)
    return v0

def right_to_origin(vR):
    v0 = np.matmul(T_0R, vR)
    return v0
