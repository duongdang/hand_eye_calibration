#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from demo0611.msg import ControllerState
from cv_bridge import CvBridge, CvBridgeError
import cv, numpy

class Blender(object):
    def __init__(self, in1, in2, out):
        self.in1 = in1
        self.in2 = in2
        self.out = out
        self.im1 = None
        self.im2 = None

        self.im1_sub = rospy.Subscriber(in1, Image, self.in1_cb)
        self.im2_sub = rospy.Subscriber(in2, Image, self.in2_cb)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(out, Image)

        self.blended_img = cv.CreateMat(480,640,cv.CV_8UC3)

    def in1_cb(self, data):
        try:
            self.im1 = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        if self.im1 and self.im2:
            try:
                cv.AddWeighted(self.im1, 0.8, self.im2, 0.8, 0., self.blended_img)
            except:
                print self.im1, self.im2, self.blended_img
                raise

            self.pub.publish(self.bridge.cv_to_imgmsg(self.blended_img, "bgr8"))

    def in2_cb(self, data):
        try:
            self.im2 = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e


def main():
    """
    """
    rospy.init_node('blender')
    in1 = rospy.get_param("~image1")
    in2 = rospy.get_param("~image2")
    out = rospy.get_param("~output")

    blender = Blender(in1, in2, out)
    rospy.spin()

if __name__ == '__main__':
    main()

