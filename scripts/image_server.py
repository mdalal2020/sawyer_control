#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
from sensor_msgs.msg import Image as Image_msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import copy
import socket
import thread
import numpy as np
import imutils
import pdb
from sawyer.srv import *
from PIL import Image
import cPickle
import imageio
import argparse
import moviepy.editor as mpy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


class Latest_observation(object):
    def __init__(self):
        # color image:
        self.img_cv2 = None
        self.img_cropped = None
        self.tstamp_img = None  # timestamp of image
        self.img_msg = None

        # depth image:
        self.d_img_raw_npy = None  # 16 bit raw data
        self.d_img_cropped_npy = None
        self.d_img_cropped_8bit = None
        self.tstamp_d_img = None  # timestamp of image
        self.d_img_msg = None


class KinectRecorder(object):
    def __init__(self, save_dir, seq_len=None, use_aux=True, save_video=False,
                 save_actions=True, save_images=True):

        self.save_actions = save_actions
        self.save_images = save_images

        """
        Records joint data to a file at a specified rate.
        rate: recording frequency in Hertz
        :param save_dir  where to save the recordings
        :param rate
        :param start_loop whether to start recording in a loop
        :param whether the recorder instance is an auxiliary recorder
        """

        side = "right"
        self.state_sequence_length = seq_len
        self.overwrite = True
        self.use_aux = False
        self.endeffector_pos = None
        self.angles = None
        self.itr = 0
        self.highres_imglist = []



        rospy.Subscriber("/kinect2/hd/image_color", Image_msg, self.store_latest_im)
        rospy.Subscriber("/kinect2/sd/image_depth_rect", Image_msg, self.store_latest_d_im)

        self.ltob = Latest_observation()
        self.ltob_aux1 = Latest_observation()

        self.bridge = CvBridge()
        self.ngroup = 1000
        self.igrp = 0

        # for timing analysis:
        self.t_finish_save = []

        self.instance_type = 'main'

        self.get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)
        self.save_kinectdata_func = rospy.ServiceProxy('save_kinectdata', save_kinectdata)

        def spin_thread():
            rospy.spin()

            thread.start_new(spin_thread, ())


    def save_kinect_handler(self, req):
        self.t_savereq = rospy.get_time()
        self.t_get_request.append(self.t_savereq)
        self._save_img_local(req.itr)
        return save_kinectdataResponse()

    def get_kinect_handler(self, req):
        print "handle get_kinect_request"

        img = np.asarray(self.ltob.img_cropped)
        img = self.bridge.cv2_to_imgmsg(img)
        return get_kinectdataResponse(img)


    def store_latest_d_im(self, data):

        self.ltob.tstamp_d_img = rospy.get_time()

        self.ltob.d_img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        img = cv2.resize(cv_image, (0, 0), fx=1 / 5.5, fy=1 / 5.5, interpolation=cv2.INTER_AREA)

        img = np.clip(img, 0, 1400)

        startcol = 7
        startrow = 0
        endcol = startcol + 64
        endrow = startrow + 64
        # crop image:
        img = img[startrow:endrow, startcol:endcol]

        self.ltob.d_img_cropped_npy = img
        img = img.astype(np.float32) / np.max(img) * 256
        img = img.astype(np.uint8)
        img = np.squeeze(img)
        self.ltob.d_img_cropped_8bit = img

    def store_latest_im(self, data):
        self.ltob.img_msg = data
        self.ltob.tstamp_img = rospy.get_time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")# (1920, 1080)
        self.ltob.img_cv2 = self.crop_highres(cv_image)
        self.ltob.img_cropped = self.crop_lowres(cv_image)

    def crop_highres(self, cv_image):
        startcol = 180
        startrow = 0
        endcol = startcol + 1500
        endrow = startrow + 1500
        cv_image = copy.deepcopy(cv_image[startrow:endrow, startcol:endcol])
        cv_image = cv2.resize(cv_image, (0, 0), fx=0.056, fy=0.07777777777, interpolation=cv2.INTER_AREA)
        if self.instance_type == 'main':
            cv_image = imutils.rotate_bound(cv_image, 180)
        return cv_image

    def crop_lowres(self, cv_image):
        self.ltob.d_img_raw_npy = np.asarray(cv_image)

        img = cv2.resize(cv_image, (0, 0), fx=1 / 16., fy=1 / 16., interpolation=cv2.INTER_AREA)
        startrow = 3
        startcol = 27

        img = imutils.rotate_bound(img, 180)
        # crop image:
        img = img[startrow:endrow, startcol:endcol]
        assert img.shape == (64, 64, 3)
        return img


  

def get_observation(unused):
    img = kr.ltob.img_cv2
    img = np.array(img)
    image = img.flatten().tolist()
    return imageResponse(image)

def image_server():
    s = rospy.Service('images', image, get_observation)
    rospy.spin()

if __name__ == "__main__":
    kr = KinectRecorder('~/Documents')
    image_server()

