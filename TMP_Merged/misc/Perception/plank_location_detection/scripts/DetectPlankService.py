#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import numpy as np

from plank_location_detection.srv import DetectPlank



class image_converter:

	def __init__(self):
		self.bridge = CvBridge()
		self.backSub = cv2.createBackgroundSubtractorMOG2(history=1)

	def image_function(self):
		msg = rospy.wait_for_message("kinect2/hd/image_color",Image)
		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		# # For expts with two stations kept beside each other. Need to adjust values based on adjust_kinect script output
		# y = 600
		# x = 550
		# h = 200
		# w = 1000
		# cv_image_cropped = cv_image[y:y+h, x:x+w]

		# For expts with two stations seperated. Need to adjust values based on adjust_kinect script output
		y1 = 720
		x1 = 900
		y2 = 723
		x2 = 1350
		h = 25
		w = 300
		cv_image_cropped = cv2.hconcat([cv_image[y1:y1+h, x1:x1+w/2], cv_image[y2:y2+h, x2:x2+w/2]])

		print("Image found")
		# cv2.imshow("Image window", cv_image)
		# cv2.waitKey(1000)
		return cv_image_cropped

	def capture_background(self):
		self.background = self.image_function()
		self.bc = True
		fgMask = self.backSub.apply(self.background)
		cv2.imwrite('/home/local/ASUAD/dkalavas/background_image.png',self.background)
		return 2, self.bridge.cv2_to_imgmsg(self.background, "bgr8")

	def capture_foreground(self):
		if not self.bc:
			return -1, None
		self.foreground = self.image_function()
		self.bc = False
		self.fc = True

		return 2, self.bridge.cv2_to_imgmsg(self.foreground, "bgr8")

	def detect_plank_location(self):
		if not self.fc:
			return -1
		fgMask = self.backSub.apply(self.foreground )
		left_wrt_camera = np.mean(fgMask[:,:500])
		right_wrt_camera = np.mean(fgMask[:,500:])
		if abs(left_wrt_camera - right_wrt_camera) < 10:
			return -2, self.bridge.cv2_to_imgmsg(fgMask,'mono8')
		location = int(left_wrt_camera > right_wrt_camera)
		return location, self.bridge.cv2_to_imgmsg(fgMask, 'mono8')

ic = image_converter()	

def DetectPlankFn(req):
	if req.x == 'capture_background':
		ret, img = ic.capture_background()
	elif req.x == 'capture_foreground':
		ret, img = ic.capture_foreground()
	elif req.x == 'get_plank_location':
		ret, img = ic.detect_plank_location()
	return ret, img

def gps():
    rospy.init_node('placed_string')
    s = rospy.Service('detect_plank_location', DetectPlank, DetectPlankFn)
    rospy.spin()

if __name__ == "__main__":
    gps()



