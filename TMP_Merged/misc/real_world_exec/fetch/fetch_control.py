#! /usr/bin/env python
import actionlib
import control_msgs.msg
import rospy
import sys, time
import trajectory_msgs.msg
import math

class Gripper(object):
    def __init__(self):
        # Gripper controls the robot's gripper.
        self.MIN_EFFORT = 75   # Min grasp force, in Newtons
        self.MAX_EFFORT = 100  # Max grasp force, in Newtons
        self.CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
        self.OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
        self.ACTION_SERVER = 'gripper_controller/gripper_action'
        self._client = actionlib.SimpleActionClient(self.ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(4))
    
    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = self.OPENED_POS
        print("sending open goal")
        self._client.send_goal(goal)
        print("waiting")
        self._client.wait_for_result(rospy.Duration(1))
        print("done")
        # self._client.send_goal_and_wait(goal, rospy.Duration(4))

    def close(self, max_effort=75):
        """Closes the gripper.
        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = self.CLOSED_POS
        goal.command.max_effort = max_effort
        print("sending close goal")
        self._client.send_goal(goal)
        print("waiting")
        self._client.wait_for_result(rospy.Duration(1))
        print("done")
        # self._client.send_goal_and_wait(goal, rospy.Duration(4))

class Head(object):
    """Head controls the Fetch's head.
    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """

    def __init__(self):
        self.PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'
        self.LOOK_AT_ACTION_NAME = 'head_controller/point_head'
        self.PAN_JOINT = 'head_pan_joint'
        self.TILT_JOINT = 'head_tilt_joint'
        self.PAN_TILT_TIME = 2.5
        self.MIN_PAN = -math.pi / 2
        self.MAX_PAN = math.pi / 2
        self.MIN_TILT = -math.pi / 2
        self.MAX_TILT = math.pi / 4
        self.traj_client = actionlib.SimpleActionClient(self.PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.point_client = actionlib.SimpleActionClient(self.LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        while not self.traj_client.wait_for_server(timeout=rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.logwarn('Waiting for head trajectory server...')
        while not self.point_client.wait_for_server(timeout=rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.logwarn('Waiting for head pointing server...')

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        Requires a frame ID, such as `base_link`
        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z

        goal.min_duration = rospy.Duration(self.PAN_TILT_TIME)
        self.point_client.send_goal(goal)
        self.point_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.
        This is more similar to the torso code which uses a simple action server
        and then a trajectory. Note that here we have two joints to consider.
        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        pan = min(max(pan, self.MIN_PAN), self.MAX_PAN)
        tilt = min(max(tilt, self.MIN_TILT), self.MAX_TILT)

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(self.PAN_TILT_TIME)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = [self.PAN_JOINT, self.TILT_JOINT]
        goal.trajectory.points.append(point)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()


    def wait_for_time(self):
        """ Wait for simulated time to begin. """
        while rospy.Time().now().to_sec() == 0:
            pass


    def execute(self,pan,tilt):
        self.wait_for_time()
        rospy.sleep(2)

        # # Look at (i.e., point head)
        # frame_id = 'base_link'
        # x, y, z = 10, -10, 0
        # head.look_at(frame_id, x, y, z)

        # Pan tilt (values are in radians)
        self.pan_tilt(pan, tilt)

