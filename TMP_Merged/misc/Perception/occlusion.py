import sys, os
import math

sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-2]))
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import time
import pickle
import numpy as np


def exec_launch(string):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [string])
    launch.start()
    return launch

def get_ar_message(launch):
    count = 0
    while (1):
        try:
            print('fat2')
            msg = rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
            break
        except:
            count += 1
            print("Fatal error!", count)
            time.sleep(1)
            if count == 5:
                print("Max attempts reached")
                launch.shutdown()
                # sys.exit("Killing code!")
                exit()
            continue
    return msg
class obscure_check:
    def __init__(self):
        self.occlusion_list = self.obscure_checks()
        
    def obscure_checks(self):
        launch = exec_launch("/opt/ros/kinetic/share/ar_track_alvar/launch/kinect_indiv.launch")  ## Replace with your own launch file uri
        block_present = {}
        marker_ids = [6,11]
        for ar_id in marker_ids:
            block_present.update({ar_id:True})
        count = 0
        for ar_id in marker_ids:
            while(1):
                print('fat')
                msg = get_ar_message(launch)
                for i in msg.markers:
                    if(i.id == ar_id):
                        block_present[i.id] = False
                        print(i.id)

                if len(msg.markers) < 2 and count <= 5:
                    print(count)
                    count += 1
                    print(count)
                    time.sleep(1)
                    continue
                else:
                	break
                
        launch.shutdown()
        occlusion_list = []
        for j in block_present:
        	occlusion_list.append(block_present[j])
        return occlusion_list

if __name__ == '__main__':
    rospy.init_node('occlusion_node',anonymous=True)
    oc = obscure_check()
