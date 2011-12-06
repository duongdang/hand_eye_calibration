
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
import datetime
import pickle

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
            print "Done"
            return
        #if len(self.T_hands) > 2:
        #    self.done = True
        self.stamps.append(stamp)


        self.T_hand = self.pose_to_matrix(hand_pose)
        self.T_head = self.pose_to_matrix(head_pose)

        # T_chessboard, T_hand_head in chessboard frame
        self.T_chessboard  = self.pose_to_matrix(chessboard_pose)
        self.T_hand_head = numpy.dot(numpy.linalg.inv(self.T_hand),
                                     self.T_head)
        self.T_hands.append(self.T_hand)
        self.T_heads.append(self.T_head)
        self.T_chessboards.append(self.T_chessboard)
        self.T_hand_heads.append(self.T_hand_head)
        print "Recorded {0} poses".format(len(self.T_hand_heads))


    def __init__(self):
        pass

    def spin(self):
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
        while not (self.done or rospy.is_shutdown()):
            pass
        self.callback = None

        suff = datetime.datetime.now().strftime("%Y%m%d%H%M")
        pname = None
        pname = "/tmp/calib_poses_{0}.pickle".format(suff)
        i = 0
        while not pname or os.path.isfile(pname):
            i += 1
            pname = "/tmp/calib_poses_{0}_{1}.png".format(suff, i)

        f = open(pname,'w')
        pickle.dump([self.T_hands, self.T_heads, self.T_hand_heads, self.T_chessboards],f)
        f.close()


if __name__ == '__main__':
    cal = Calibrator()

    if sys.argv[1:]:
        p_fn = sys.argv[1]
        f = open(p_fn)
        data =  pickle.load(f)
        cal.T_hands, cal.T_heads, cal.T_hand_heads, cal.T_chessboards = data
    else:
        cal.spin()
    print len(cal.T_hand_heads), len(cal.T_chessboards)
    from regress_chou import regress_pose, regress_Hs, regress_pose2
    for Ax, E, lambdas, res, samples in regress_pose2(cal.T_hand_heads, cal.T_chessboards):
        quat = tf.transformations.quaternion_from_matrix(Ax)
        print "Optimized on {0} (pair) poses".format(samples)
        print "quat:",quat
        print "xyz:", Ax[:3,3]
        print "lambdas: (smaller the better)", lambdas
        opt_to_ros = numpy.eye(4)
        opt_to_ros[:3,0] = [0, 0,1]
        opt_to_ros[:3,1] = [-1,0,0]
        opt_to_ros[:3,2] = [0, -1,0]
        A_ros = numpy.dot(Ax, opt_to_ros)
        rpy_ros = tf.transformations.euler_from_matrix(A_ros)
        print "ros_params", rpy_ros, A_ros[:3,3]
        print "---"
