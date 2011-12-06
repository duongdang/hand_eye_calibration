
#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import message_filters
import numpy, numpy.linalg

class RvCollector(object):
    stamps = []
    poses = []
    collect = False
    Hs = []
    Hl = None
    Hc = None
    Tl_bef = None
    Tl_aft = None
    Tc_bef = None
    Tc_aft = None
    T_hand = numpy.eye(4)
    T_head = numpy.eye(4)
    T_chessboard = numpy.eye(4)
    T_hand_head = numpy.eye(4)

    def pose_to_matrix(self, P):
        T = tf.transformations.quaternion_matrix([P.pose.orientation.x,
                                                  P.pose.orientation.y,
                                                  P.pose.orientation.z,
                                                  P.pose.orientation.w,
                                                  ])
        T[:3,3] = [P.pose.position.x,
                   P.pose.position.y,
                   P.pose.position.z]
        return T

    def callback(self, hand_pose, head_pose, chessboard_pose):
        if not self.collect:
            rospy.logdebug("synced, ignored")
            return

        self.T_hand = self.pose_to_matrix(hand_pose)
        self.T_head = self.pose_to_matrix(head_pose)

        # T_chessboard, T_hand_head in chessboard frame
        self.T_chessboard  = self.pose_to_matrix(chessboard_pose)
        self.T_hand_head = numpy.dot(numpy.linalg.inv(self.T_hand),
                                     self.T_head)

        if self.Tc_bef == None:
            self.Tc_bef = self.T_chessboard
            self.Tl_bef = self.T_hand_head
            print "Record bef params"
        else:
            self.Tc_aft = self.T_chessboard
            self.Hc = numpy.dot(self.Tc_bef, numpy.linalg.inv(self.Tc_aft))
            self.Tl_aft = self.T_hand_head
            self.Hl = numpy.dot(numpy.linalg.inv(self.Tl_bef), self.Tl_aft)

        self.collect = False

        if self.Hl != None and self.Hc != None:
            self.Hs.append((self.Hl, self.Hc))
            self.Hl = None
            self.Hc = None
            self.Tl_bef = None
            self.Tl_aft = None
            self.Tc_bef = None
            self.Tc_aft = None
            print len(self.Hs), "th motion recorded"

    def __init__(self):
        rospy.init_node('rv_test')
        hand_sub = message_filters.Subscriber('/robotviewer/pose/HRP2JRL/RARM_JOINT5',
                                              PoseStamped)
        head_sub = message_filters.Subscriber('/robotviewer/pose/HRP2JRL/HEAD_JOINT1',
                                              PoseStamped)
        chessboard_sub = message_filters.Subscriber('/wide/left/pose0',
                                             PoseStamped)

        ts = message_filters.TimeSynchronizer((hand_sub, head_sub, chessboard_sub), 50)
        ts.registerCallback(self.callback)

        while raw_input() != "q":
            self.collect = True

        from regress_chou import regress_pose, regress_Hs
        Ax, E, lambdas, res, no_samples = regress_Hs(self.Hs,)
        quat = tf.transformations.quaternion_from_matrix(Ax)
        print "quat:",quat
        print "xyz:", Ax[:3,3]
        print "lambdas: (smaller the better)", lambdas

RvCollector()
