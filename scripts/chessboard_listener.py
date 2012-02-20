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
def pos_to_mat(data):
    if 'pose' in dir(data):
        data = data.pose
    if 'position' in dir(data):
        translation = [data.position.x, data.position.y, data.position.z]
        rotation = [data.orientation.w, data.orientation.x, data.orientation.y,
                       data.orientation.z, ]
    else:
        translation = [data.translation.x, data.translation.y,
                       data.translation.z]
        rotation = [ data.rotation.w, data.rotation.x, data.rotation.y,
                       data.rotation.z,]
    res = transformations.quaternion_matrix(rotation)
    res[:3,3] = translation
    return res

def main():
    """
    """
    rospy.init_node("chessboard_listener")
    cb_topic = rospy.get_param("~cb_topic")
    world_frame = rospy.get_param("~world_frame")
    link_frame  = rospy.get_param("~link_frame")
    bag  = rospy.get_param("~bag")
    camera = rospy.get_param("~camera")
    camera = camera.replace("/","_")

    bag = bag.replace(".bag","")
    bag = os.path.basename(bag)
    path = os.path.abspath(os.path.dirname(__file__))
    out_fn = os.path.join(path,"/tmp/poses_{0}_{1}.yaml".format(bag, camera))
    tf_listener = tf.TransformListener()

    yaml_data = []

    def cb(data):
        #rospy.loginfo("{0}".format(data))
        stamp = data.header.stamp
        link_tf = tf_listener.waitForTransform(world_frame, link_frame, stamp, rospy.Duration(4.0))
        link_tf = tf_listener.lookupTransform(world_frame, link_frame, stamp)
        T_link = transformations.quaternion_matrix(link_tf[1])
        T_link[:3,3] = link_tf[0]

        T_chessboard = data.pose
        T_chessboard = pos_to_mat(T_chessboard)
        #T_chessboard =  numpy.linalg.inv(T_chessboard)
        yaml_data.append({
                'T_chessboard': sum(T_chessboard.tolist(),[]),
                'T_link': sum(T_link.tolist(),[]),
                }
                         )

        yf = open(out_fn, 'w')
        yaml.dump(yaml_data, yf)
        yf.close()

    def calibrate():
        program = os.path.join(path, "..",  "bin", "calibrator_tsai")
        outrs = out_fn.replace("yaml", "result")
        subprocess.call([program, out_fn, outrs])

    atexit.register(calibrate)


    rospy.Subscriber(cb_topic, PoseStamped, cb)
    rospy.spin()

if __name__ == '__main__':
    main()
