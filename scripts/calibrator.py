#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
from demo0611.msg import ControllerState
import pickle
import re
import rosbag

class Calibrator(object):
    """
    """

    def __init__(self, ):
        """
        """

        rospy.init_node("calibrator")
        self.hand_pos = None

        self.ns = rospy.get_namespace()
        self.hand_pos_cam_pos = []
        self.stamps = []

        # rospy.Subscriber("camera_info", CameraInfo, self.echo_cb)
        rospy.Subscriber("hand_pos", TransformStamped, self.hand_pos_cb)
        rospy.Subscriber("pose0", PoseStamped, self.chessboard_pos_cb)
        rospy.spin()

    def echo_cb(self, data):
        print "received ", data._connection_header['type']

    def chessboard_pos_cb(self, data):
        stamp = data.header.stamp
        if stamp not in self.stamps:
            self.stamps.append(stamp)
            print len(self.stamps)
            self.hand_pos_cam_pos.append([data.pose,self.hand_pos])
            p = open('pos.pickle','w')
            pickle.dump(self.hand_pos_cam_pos, p)
            p.close()
        else:
            print self.hand_pos_cam_pos

    def hand_pos_cb(self, data):
        self.hand_pos = data.transform

def main():
    """
    """
    rec = Calibrator()

if __name__ == '__main__':
    main()

