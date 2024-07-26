from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import numpy as np

from plank_location_detection.srv import DetectPlank
# For expts with two stations kept beside each other. Need to adjust values based on camera angle and position
# def image_function(bridge):
#         msg = rospy.wait_for_message("kinect2/hd/image_color",Image)
#         cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#         y = 600
#         x = 550
#         h = 200
#         w = 1000
#         cv_image = cv_image[y:y+h, x:x+w]
#         cv2.imshow("Image window", cv_image)
#         cv2.waitKey(3)

# For expts with two stations seperated. Need to adjust values based on camera angle and position
def image_function(bridge):
        msg = rospy.wait_for_message("kinect2/hd/image_color",Image)
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        y1 = 720
        x1 = 900
        y2 = 723
        x2 = 1350
        h = 25
        w = 300
        # cv_imagefin = cv2.hconcat([cv_image[y1:y1+h, x1:x1+w/2], cv_image[y2:y2+h, x2:x2+w/2])
        cv2.rectangle(cv_image, (x1, y1), (x1+w/2, y1+h/2), (255,0,0), 2)
        cv2.rectangle(cv_image, (x2, y2), (x2+w/2, y2+h/2), (255,0,0), 2)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


rospy.init_node('calib_kinect')
bridge = CvBridge()
while True:
        image_function(bridge)