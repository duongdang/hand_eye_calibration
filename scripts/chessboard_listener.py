#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
import tf
import tf.transformations as transformations
import re
import yaml
import atexit
import subprocess, numpy.linalg


def pose_matrix(t ):
    ro = t.orientation
    tr = t.position
    T = tf.transformations.quaternion_matrix(
        [ro.x, ro.y, ro.z, ro.w]
        )
    T[:3,3] = [tr.x, tr.y, tr.z]
    return T


def main():
    """
    """
    rospy.init_node("chessboard_listener")
    chessboard_topic = rospy.get_param("~chessboard_topic")
    world_frame = rospy.get_param("~world_frame")
    link_frame  = rospy.get_param("~link_frame")
    bag  = rospy.get_param("~bag")
    camera = rospy.get_param("~camera")
    camera = camera.replace("/","_")

    bag = bag.replace(".bag","")
    bag = os.path.basename(bag)
    path = os.path.abspath(os.path.dirname(__file__))
    out_fn = os.path.join(path,"/tmp/{0}_{1}_poses.yaml".format(bag, camera))
    tf_listener = tf.TransformListener()

    yaml_data = []

    def cb(data):
        #rospy.loginfo("{0}".format(data))
        stamp = data.header.stamp
        tf_listener.waitForTransform(world_frame, link_frame, stamp, rospy.Duration(4.0))

        source_frame = world_frame
        target_frame = link_frame
        tf_link = tf_listener.lookupTransform(source_frame, target_frame, stamp)
        T_link = transformations.quaternion_matrix(tf_link[1])
        T_link[:3,3] = tf_link[0]


        pose_chessboard = data.pose
        T_chessboard = pose_matrix(pose_chessboard)
        #T_chessboard =  numpy.linalg.inv(T_chessboard)

        yaml_data.append({
                'T_chessboard': sum(T_chessboard.tolist(),[]),
                'T_link': sum(T_link.tolist(),[]),
                }
                         )

        yf = open(out_fn, 'w')
        yaml.dump(yaml_data, yf)

        print len(yaml_data)
        print stamp
        print T_chessboard
        print T_link
        print "---"
        yf.close()

    def calibrate():
        program = os.path.join(path, "..",  "bin", "calibrator_tsai")
        outrs = out_fn.replace("poses.yaml", "result.yaml")
        subprocess.call([program, out_fn, outrs])

    atexit.register(calibrate)

    count = 0
    rospy.Subscriber(chessboard_topic, PoseStamped, cb)
    rospy.spin()

if __name__ == '__main__':
    main()
