
#! /usr/bin/env python
__author__ = "Duong Dang"
__version__ = "0.1"

import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
import tf
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

import yaml
import pprint
path = "/home/nddang/src/ros/ros-hrp2/hrp2_14/launch/calibration"

calib_files = {
    "/wide/left":"wide_left.yaml",
    "/wide/right":"wide_right.yaml",
    "/narrow/left":"narrow_left.yaml",
    "/narrow/right":"narrow_right.yaml",
    }

for cam_name, fn in calib_files.items():
    service_name = cam_name + "/set_camera_info"
    print "waiting for service", service_name
    rospy.wait_for_service(service_name,5)
    service = rospy.ServiceProxy(service_name, SetCameraInfo )
    info = CameraInfo()
    full_fn = os.path.join(path, fn)
    params = yaml.load(open(full_fn).read())
    info.width = params['image_width']
    info.height = params['image_height']
    info.distortion_model = params["distortion_model"]
    info.D = params['distortion_coefficients']['data']
    info.K = params['camera_matrix']['data']

    info.P = params['projection_matrix']['data']
    info.R = params['rectification_matrix']['data']
    response = service(info)
    print response
