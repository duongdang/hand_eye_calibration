
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

class Calibrator(object):
    stamps = []
    poses = []
    collect = False
    Hls = []
    Hcs = []
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

    T_hands = []
    T_heads = []
    T_chessboards = []
    T_hand_heads = []
    done = False

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
        stamp = hand_pose.header.stamp
        if stamp in self.stamps:
            self.done = True
        #if len(self.T_hands) > 2:
        #    self.done = True
        self.stamps.append(stamp)


        self.T_hand = self.pose_to_matrix(hand_pose)
        self.T_head = self.pose_to_matrix(head_pose)

        # T_chessboard, T_hand_head in chessboard frame
        self.T_chessboard  = numpy.linalg.inv(self.pose_to_matrix(chessboard_pose))
        self.T_hand_head = numpy.dot(numpy.linalg.inv(self.T_hand),
                                     self.T_head)
        self.T_hands.append(self.T_hand)
        self.T_heads.append(self.T_head)
        self.T_chessboards.append(self.T_chessboard)
        self.T_hand_heads.append(self.T_hand_head)
        print "Recorded {0} poses".format(len(self.T_hand_heads))


    def __init__(self):
        rospy.init_node('calibrator')
        hand_pose_topic = rospy.get_param("~hand_pose",
                                          '/robotviewer/pose/HRP2JRL/RARM_JOINT5')
        head_pose_topic = rospy.get_param("~head_pose",
                                          '/robotviewer/pose/HRP2JRL/HEAD_JOINT1')
        chessboard_pose_topic = rospy.get_param("~chessboard_pose",
                                                '/wide/left/pose0')

        hand_sub = message_filters.Subscriber(hand_pose_topic,
                                              PoseStamped)
        head_sub = message_filters.Subscriber(head_pose_topic,
                                              PoseStamped)
        chessboard_sub = message_filters.Subscriber(chessboard_pose_topic,
                                             PoseStamped)

        ts = message_filters.TimeSynchronizer((hand_sub, head_sub, chessboard_sub), 50)
        ts.registerCallback(self.callback)

        #print "Enter 'done' when finish"
        while not self.done:
            pass

        from regress_chou import regress_pose, regress_Hs, regress_pose2
        Ax, E, lambdas, res = regress_pose2(self.T_hand_heads, self.T_chessboards)



        quat = tf.transformations.quaternion_from_matrix(Ax)
        print "quat:",quat
        print "xyz:", Ax[:3,3]
        print "lambdas: (smaller the better)", lambdas
        f = open("/tmp/calib_poses.pickle",'w')
        import pickle
        pickle.dump(self,f)
        f.close()

Calibrator()
