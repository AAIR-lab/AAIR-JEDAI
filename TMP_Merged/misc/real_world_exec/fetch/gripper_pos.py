#!/usr/bin/env python
import tf2_ros
import sys
import rospy
import time
rospy.init_node('gframe')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
while not rospy.is_shutdown():
	try:
		tstmp = rospy.Time()
		trans1 = tfBuffer.lookup_transform(sys.argv[1],sys.argv[2], tstmp)
		#trans1 = tfBuffer.lookup_transform('base_link','wrist_roll_link', tstmp)
		break
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print("Fatal error!")
		time.sleep(0.1)
print("trans1", trans1)
