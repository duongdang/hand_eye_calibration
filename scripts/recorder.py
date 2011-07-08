#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from demo0611.msg import ControllerState

import re
import rosbag

class Recorder(object):
    """
    """

    def __init__(self, ):
        """
        """

        rospy.init_node("recorder")

        self.camera_info = None
        self.image_raw = None
        self.hand_pose = None
        self.camera_pose = None
        self.ns = rospy.get_namespace()
        bagfn = "/tmp/" + re.sub(r"\W", "_", self.ns.strip("/"), ) + ".bag"
        self.bag = rosbag.Bag(bagfn, 'w')

        rospy.Subscriber("camera_info", CameraInfo, self.camera_info_cb)
        rospy.Subscriber("image_raw", Image, self.image_raw_cb)
        rospy.Subscriber("hand_pose", TransformStamped, self.hand_pose_cb)
        rospy.Subscriber("camera_pose", TransformStamped, self.camera_pose_cb)
        rospy.Subscriber("/controller", ControllerState, self.controller_state_cb)
        rospy.spin()

    def camera_info_cb(self, data):
        self.camera_info = data

    def image_raw_cb(self, data):
        self.image_raw = data

    def hand_pose_cb(self, data):
        self.hand_pose = data

    def camera_pose_cb(self, data):
        self.camera_pose = data

    def controller_state_cb(self, state):
        self.bag.write(self.ns + "camera_info", self.camera_info)
        self.bag.write(self.ns + "image_raw", self.image_raw)
        self.bag.write(self.ns + "hand_pos", self.hand_pose)
        # self.bag.write(self.ns + "camera_pos", self.camera_pose)
        self.bag.write("/controller", state)

        if state.done:
            self.bag.close()
            rospy.signal_shutdown("Done recording")

def main():
    """
    """
    rec = Recorder()

if __name__ == '__main__':
    main()

