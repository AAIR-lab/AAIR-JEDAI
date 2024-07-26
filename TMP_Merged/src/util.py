#!/usr/bin/env python

import random
import numpy as np
import openravepy as orpy
import numpy
import os
import Config


# Local modules
def sortArrays(array,index_to_sort):
  array = np.asarray(array)
  return array[array[:,index_to_sort].argsort()]


def getTranslationFromTransform(transform):
  return orpy.poseFromMatrix(transform)[4:]

def getRotationFromTransform(transform):
  return orpy.poseFromMatrix(transform)[0:4]  

def getRaveTransformFromROSPose(ros_pose):
  """
  OpenRAVE Convention: Quaternions are defined with the scalar value as the first component. For example [w x y z]
  """
  rave_pose = numpy.asarray([\
  ros_pose.orientation.w, \
  ros_pose.orientation.x, \
  ros_pose.orientation.y, \
  ros_pose.orientation.z, \
  ros_pose.position.x, \
  ros_pose.position.y, \
  ros_pose.position.z])

  return orpy.matrixFromPose(rave_pose)

def getRandomOnTopSurface(table_link):

	transform = table_link.GetTransform()
	
	tableTop = table_link.GetGeometries()[0]
	extentX,extentY,extentZ = tableTop.GetBoxExtents()

	tableX = transform[0][3]
	tableY = transform[1][3]

	randX = random.uniform(tableX-extentX,tableX+extentX)
	randY = random.uniform(tableY-extentY,tableY+extentY)

	transform[0][3]=randX
	transform[1][3]=randY
	transform[2][3]=1 # hover above the surface

	return transform

def ros_base_trajectory_from_openrave(robot, sim, traj):
    from openravepy import quatFromAxisAngle
    from openravepy import DOFAffine
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    old_dof_indices = robot.GetActiveDOFIndices()
    robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
    numWayPoints = traj.GetNumWaypoints()
    ros_waypoints = []
    for i in range(numWayPoints):
        wayPoint = traj.GetWaypoint(i)
        wayPointDOFs = sim.get_joint_values_from_waypoint(wayPoint, robot)
        orientation = quatFromAxisAngle(np.asarray([0.,0.,wayPointDOFs[2]]))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = wayPointDOFs[0]
        goal.target_pose.pose.position.y = wayPointDOFs[1]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = orientation[1]
        goal.target_pose.pose.orientation.y = orientation[2]
        goal.target_pose.pose.orientation.z = orientation[3]
        goal.target_pose.pose.orientation.w = orientation[0]
        ros_waypoints.append(goal)
    robot.SetActiveDOFs(old_dof_indices)
    return ros_waypoints

def ros_manip_trajectory_from_openrave(robot, traj, velocity_scale = 0.05, dof_indices = None, time_tolerance=0.01):
  """ Convert an OpenRAVE trajectory to a ROS trajectory.
  @param robot: OpenRAVE robot
  @type  robot: openravepy.Robot
  @param traj: input trajectory
  @type  traj: openravepy.Trajectory
  @param dof_indices: indices to convert
  @type dof_indices: [int]
  @param time_tolerance: minimum time between two waypoints
  @type  time_tolerance: float
  """
  from rospy import Duration
  from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

  assert time_tolerance >= 0.
  time_tolerance /= velocity_scale
  if traj.GetEnv() != robot.GetEnv():
      raise ValueError(
          'Robot and trajectory are not in the same environment.')

  cspec = traj.GetConfigurationSpecification()

  if dof_indices == None:
      dof_indices, _ = cspec.ExtractUsedIndices(robot)
  else:
      used_indices, _ = cspec.ExtractUsedIndices(robot)
      dof_indices = list(set(used_indices) & set(dof_indices))

  traj_msg = JointTrajectory(
      joint_names=[ robot.GetJointFromDOFIndex(dof_index).GetName()
                    for dof_index in dof_indices ]
  )

  time_from_start = 0.
  prev_time_from_start = 0.
  
  waypoint_nums = range(traj.GetNumWaypoints())
  for iwaypoint in waypoint_nums:
      waypoint = traj.GetWaypoint(iwaypoint)

      dt = cspec.ExtractDeltaTime(waypoint)
      q = cspec.ExtractJointValues(waypoint, robot, dof_indices, 0)
      qd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 1)
      qdd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 2)

      if dt is None:
          raise ValueError('Trajectory is not timed.')
      elif q is None:
          raise ValueError('Trajectory does not contain joint values')
      elif qdd is not None and qd is None:
          raise ValueError('Trajectory contains accelerations,'
                           ' but not velocities.')

      # Duplicate waypoints break trajectory execution, so we explicitly
      # filter them out. Note that we check the difference in time between
      # the current and the previous waypoint, not the raw "dt" value. This
      # is necessary to support very densely sampled trajectories.
      time_from_start += dt/velocity_scale
      deltatime = time_from_start - prev_time_from_start

      if iwaypoint > 0 and deltatime < time_tolerance:
          # logger.warning('Skipped waypoint %d because deltatime is %.3f < %.3f.',
              # deltatime, time_tolerance)
          continue

      prev_time_from_start = time_from_start

      # Create the waypoint.
      vel =  list(qd) if qd is not None else []
      vel = map(lambda x: x * velocity_scale, vel)
      traj_msg.points.append(
          JointTrajectoryPoint(
              positions=list(q),
              velocities=vel,
              accelerations=list(qdd) if qdd is not None else [],
              time_from_start=Duration.from_sec(time_from_start)
          )
      )

  assert abs(time_from_start - traj.GetDuration()/velocity_scale) < time_tolerance
  return traj_msg


def getRobotTraj(joint_trajectory):
    import moveit_msgs.msg
    rt=moveit_msgs.msg.RobotTrajectory()
    rt.joint_trajectory = joint_trajectory

def drawTransform(env,T,length=0.1):
  """draws a set of arrows around a coordinate system"""
  return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=0.01,color=[1.0,0.0,0.0]),env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=0.01,color=[0.0,1.0,0.0]),env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=0.01,color=[0.0,0.0,1.0])]

def createWaypoint(robot):
  arm_dofs = robot.GetActiveManipulator().GetArmDOFValues().tolist()
  arm_velocities = numpy.zeros(len(arm_dofs)).tolist()
  deltatime = [1]
  iswaypoint = [1]
  waypoint = arm_dofs+arm_velocities+deltatime+iswaypoint
  return numpy.asarray(waypoint)



class Stack(object):
  def __init__(self):
    self.list = []

  def push(self,item):
    self.list.append(item)

  def pop(self):
      if (len(self.list) == 0):
          return None
      else:
        return self.list.pop()

  def top(self):
    if(len(self.list) == 0):
      return None
    else:
      return self.list[len(self.list)-1]

  def isEmpty(self):
    return len(self.list) == 0


import itertools
def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)

PDDL_FILE_COUNT = 2
def incorporate_errors_to_pddl(pddl_problem_file, state, failed_preconditions ):
    state_changed = False
    if not failed_preconditions:
        failed_preconditions = []

    if failed_preconditions is None:
        raise StandardError
    for prep_str in failed_preconditions.get_proposition_strings():
        state.addProposition(prep_str)
        print "Adding Proposition: "+str(prep_str)
        state_changed = True
        # break; #TODO: Breaking here this for testing

    if not state_changed:
        print "No failed preconditions found, PDDL problem file unchanged."
        raise StandardError
        return pddl_problem_file

    from Parser.InitFileMgr import InitFileMgr
    fileMgr = InitFileMgr(pddl_problem_file)
    fileMgr.replaceInitState(state)
    global PDDL_FILE_COUNT
    new_problem_file_name = pddl_problem_file+"_"+str(PDDL_FILE_COUNT) #Change name
    PDDL_FILE_COUNT = PDDL_FILE_COUNT + 1
    fileMgr.writeCurrentPDDL(new_problem_file_name)
    return new_problem_file_name

def copy_rave_state(or_ll_state):
    from States.OpenRaveLowLevelState import OpenRaveLowLevelState

    new_state = OpenRaveLowLevelState(values=or_ll_state.values)
    for body in or_ll_state.env.GetBodies():
        new_state.env.GetKinBody(body.GetName()).SetTransform(body.GetTransform())

    return new_state

def copy_graph(graph):
    pass

def get_object_limits(obj):
    """Returns the bounding box of an object.

    Returns: min_x, max_x, min_y, max_y, z
    """

    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]

    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]
    z = ab.pos()[2] + ab.extents()[2]

    return min_x, max_x, min_y, max_y, z



def log(message):
    import Config
    import logging
    log_level = Config.LOG_LEVEL
    if log_level == "DEBUG":
        log_level = logging.DEBUG
    elif log_level == "WARNING":
        log_level = logging.WARNING
    elif log_level == "INFO":
        log_level = logging.INFO

    logging.basicConfig(level=log_level,
                        format='%(asctime)s %(levelname)s %(message)s',
                        filename='/tmp/myapp.log',
                        filemode='w')

def blockprint():
    # sys.stdout = open(os.devnull,"w")
    pass

def enablePrint():
    # sys.stdout = sys.__stdout__
    pass

def set_paths():
    # Adds correct path in URDF files
    sedstr = "sed -i \"s|project_directory|"+Config.PROJ_DIR[:-1]+"|g\" "+Config.PROJ_DIR
    # os.system(sedstr +"GeneratedEnvironments/Hanger/UAV.urdf")
    os.system(sedstr +"misc/RobotModels/fetch/URDF/fetch.urdf")
    os.system(sedstr +"misc/RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf")
 
def reset_paths():
    sedstr = "sed -i \"s|"+Config.PROJ_DIR[:-1]+"|project_directory|g\" "+Config.PROJ_DIR
    # os.system(sedstr +"GeneratedEnvironments/Hanger/UAV.urdf")
    os.system(sedstr +"misc/RobotModels/fetch/URDF/fetch.urdf")
    os.system(sedstr +"misc/RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf")
