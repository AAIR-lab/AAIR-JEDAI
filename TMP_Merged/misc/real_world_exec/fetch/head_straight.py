from fetch_control import Head
import rospy
rospy.init_node('head_straight')
head = Head()
head.execute(0.0,0.60)
