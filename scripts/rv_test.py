
#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import message_filters

stamps = []
poses = []
def callback(hand_pose, cam_pose):
    global stamps
    stamp = hand_pose.header.stamp
    stamps.append(stamp)
    poses.append((cam_pose, hand_pose))

rospy.init_node('rv_test')
hand_sub = message_filters.Subscriber('/robotviewer/pose/HRP2JRL/RARM_JOINT6',
                                      PoseStamped)
cam_sub = message_filters.Subscriber('/robotviewer/CAMERA_LU/pose0',
                                     PoseStamped)

ts = message_filters.TimeSynchronizer((hand_sub, cam_sub), 50)
ts.registerCallback(callback)

raw_input()
callback = None
print len(poses)

from regressor import regress
regress(poses)
