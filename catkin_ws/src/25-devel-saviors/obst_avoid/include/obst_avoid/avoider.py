#!/bin/bash/env python

import argparse
import numpy as np
from numpy.linalg import inv
from numpy import linalg as LA
from os.path import basename, expanduser, isfile, join, splitext
import socket
from matplotlib import pyplot as plt
import time
from skimage import measure

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify

class Avoider():
	'''class to avoid detected obstacles'''
	def __init__(self, robot_name=''):
	# Robot name
	self.robot_name = robot_name

