#!/usr/bin/env python

import sys, os
from fetch_control import *
sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-2]))
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import time
import pickle
import Config
import numpy as np

use_kinect =False
if use_kinect:
    kinect_str = ''
else:
    kinect_str = "_no_kinect"
find_table = True
find_cylinders = True
MIN_MATCHES = 10


def exec_launch(string):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [string])
    launch.start()
    return launch


def robot_joint_state_extractor():
    def get_pose_message():
        while (1):
            try:
                msg = rospy.wait_for_message("joint_states", JointState)
                break
            except:
                print("Fatal error in joint pose estimation!")
                time.sleep(1)
                continue
        return msg

    joint_dump = open(Config.PROJ_DIR + "real_world_exec/init_joint_state.pickle", "wb")
    while (1):
        msg = get_pose_message()
        if len(msg.name) == 13:
            print(msg.name)
            print(msg.position)
            break
    pickle.dump([msg.position], joint_dump)
    joint_dump.close()
    print("...Joint Joint State Extracted...")
    print(msg.position)


def get_ar_message(launch):
    count = 0
    while (1):
        try:
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


def table_pose_extractor():
    # Get Table Info
    launch = exec_launch(Config.PROJ_DIR + "real_world_exec/fetch_multi_markers_black_table"+kinect_str+".launch")
    print(Config.PROJ_DIR + "real_world_exec/fetch_multi_markers_black_table"+kinect_str+".launch")
    count = 0
    flag = False
    table_pose = []
    while True:
        try:
            msg = get_ar_message(launch)
        except SystemExit:
            sys.exit()
	print("lol",msg)
        for i in msg.markers:
	    print(i)
            if i.id == 20:
                table_id = i.id
                pose = i.pose.pose
                curr = np.asarray(
                    [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x,
                     pose.orientation.y, pose.orientation.z])
                if flag and np.allclose(curr, prev, rtol=0.001, atol=0.001):
                    count += 1
                    print(count)
                else:
                    count = 0
                if count == MIN_MATCHES:
                    table_pose = pose
                    print 'table pose extracted'
                    break
        if not flag:
            flag = True
	prev = curr
        if count == MIN_MATCHES:
            break
    print("...Table Pose Extracted...")
    print(pose)
    launch.shutdown()
    return table_pose


def cylinder_pose_extractor(obj_to_markers_ids):
    launch = exec_launch(Config.PROJ_DIR + "real_world_exec/fetch_indiv"+kinect_str+".launch")
    cylinder_poses = []
    count = 0
    marker_ids = obj_to_markers_ids.keys()
    flag = np.zeros(len(marker_ids))
    curr = {}
    prev = {}
    count = np.zeros(len(marker_ids))
    print(len(marker_ids), count)
    cylinder_id = -1
    total_cyl = len(marker_ids)
    while (True):
        try:
            msg = get_ar_message(launch)
        except SystemExit:
            sys.exit()
        except:
            print("Caught Exception")
            continue
        for i in msg.markers:
            if i.id in marker_ids:
                cylinder_id = obj_to_markers_ids[i.id]
                print(cylinder_id)
                if count[cylinder_id] == MIN_MATCHES:
                    continue
                print("Inside", cylinder_id, i.id)
                pose = i.pose.pose
                curr[cylinder_id] = np.asarray([pose.position.x, pose.position.y,
                                                pose.position.z])  # ,pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z])
                if flag[cylinder_id] == 1 and np.allclose(curr[cylinder_id], prev[cylinder_id], rtol=0.001,
                                                          atol=0.001):
                    count[cylinder_id] += 1
                    print(cylinder_id, count[cylinder_id])
                    if count[cylinder_id] == MIN_MATCHES:
                        cylinder_poses.append([cylinder_id, pose])
                        print("object", cylinder_id, " pose extracted")
                        marker_ids.remove(i.id)
                        print("marker ids left to find:", marker_ids)
                        continue
                else:
                    count[cylinder_id] = 0
                if flag[cylinder_id] == 0:
                    flag[cylinder_id] = 1
                prev[cylinder_id] = curr[cylinder_id]
        print("marker ids left to find:", marker_ids)
        if len(cylinder_poses) == total_cyl:
            break
    launch.shutdown()
    return cylinder_poses


if __name__ == '__main__':
    try:
        pose_dump = open(Config.PROJ_DIR + "real_world_exec/pose_dump.pickle", "rb")
        table_pose, cylinder_poses = pickle.load(pose_dump)
        pose_dump.close()
    except:
        print("Failed to read file")
    rospy.init_node("generate_env", anonymous=True)
    a = Head()
    #a.execute(0,0.60)
    # Extract Table Info
    if find_table:
        table_pose = []
        table_pose = table_pose_extractor()
    # Get Cylinder Info
    # markers_ids = [4,5,6,7,8,9,10,11,12,13]
    if find_cylinders:
        cylinder_poses = []
        obj_to_markers_ids = {4: 0, 5: 1, 6: 2, 7:3, 8:4, 9:5, 10:6, 11:7, 12:8 , 13:9}
        cylinder_poses = cylinder_pose_extractor(obj_to_markers_ids)
    pose_dump = open(Config.PROJ_DIR + "real_world_exec/pose_dump.pickle", "wb")
    pickle.dump([table_pose, cylinder_poses], pose_dump)
    pose_dump.close()
    print(table_pose)
    # Extract Robot Joint States
    robot_joint_state_extractor()
