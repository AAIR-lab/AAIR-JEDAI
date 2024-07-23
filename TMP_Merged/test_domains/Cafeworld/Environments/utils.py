import numpy as np
import openravepy
import re
import math
# import trajoptpy
import time
import StringIO
from functools import wraps
import errno
import signal
import os


pr2_l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417,
                    -0.0962141, -0.0864407]

pr2_r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408,
                    -1.4175, -1.8417, 0.21436]

def pr2_tuck_arm(robot):
    robot.SetDOFValues(pr2_r_arm_tucked,
                       robot.GetManipulator("rightarm").GetArmIndices())
    robot.SetDOFValues(pr2_l_arm_tucked,
                       robot.GetManipulator("leftarm").GetArmIndices())

def get_environment_limits(env, robot=None):
    """Calculates the limits of an environment, If robot is not None then then
    limits are shrunk by the size of the robot.
    Return:
    envmin, envmax: two 1x3 arrays with the environment minimun and maximum
    extension.
    """

    if robot is not None:
        abrobot = robot.ComputeAABB()
        min_r = abrobot.extents()
        max_r = -abrobot.extents()
    else:
        min_r = max_r = 0

    with env:
        envmin = []
        envmax = []
        for b in env.GetBodies():
            ab = b.ComputeAABB()
            envmin.append(ab.pos()-ab.extents())
            envmax.append(ab.pos()+ab.extents())
        envmin = np.min(np.array(envmin),0) + min_r
        envmax = np.max(np.array(envmax),0) + max_r

    return envmin, envmax


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

def get_object_limits_2(obj):
  """
Returns the bounding box of an object.
Returns: min_x, max_x, min_y, max_y, min_z, max_z
"""

  ab = obj.ComputeAABB()
  max_x = ab.pos()[0] + ab.extents()[0]
  min_x = ab.pos()[0] - ab.extents()[0]

  max_y = ab.pos()[1] + ab.extents()[1]
  min_y = ab.pos()[1] - ab.extents()[1]

  max_z = ab.pos()[2] + ab.extents()[2]
  min_z = ab.pos()[2] - ab.extents()[2]

  return min_x, max_x, min_y, max_y, min_z, max_z

def get_object_center(obj):
  min_x, max_x, min_y, max_y, z = get_object_limits(obj)
  return [(min_x + max_x) / 2, (min_y + max_y) / 2, z]

def get_object_height(obj):
  ab = obj.ComputeAABB()
  max_z = ab.pos()[2] + ab.extents()[2]
  min_z = ab.pos()[2] - ab.extents()[2]
  return max_z - min_z

def create_body_at(env, T, name = "crash-test-dummy"):
  # import IPython
  # IPython.embed()
  body = openravepy.RaveCreateKinBody(env, "")
  body.SetName(name)
  body.InitFromBoxes(np.array([[0,0,0, 0.04, 0.04, 0.1]]), True)
  p = openravepy.poseFromMatrix(T)
  p[-1] += 0.101

  body.SetTransform(openravepy.matrixFromPose(p))
  return body


def get_all_collisions(obj, env):
    """Returns a set with all the objects in collision with obj, empty if no
    collision """
    collisions = set()
    report = openravepy.CollisionReport()
    for link in obj.GetLinks():
        if env.CheckCollision(link, report=report):
            collisions.add(report.plink2.GetParent())

    return collisions


def setGoalObject(objName, pddlFile):
    pddlStr = open(pddlFile).read()
    goalStr = "At " + objName + " table6"
    outStr = re.sub(r'At \w* table6', goalStr, pddlStr)
    outf =  pddlFile.replace(".pddl", "_edited.pddl")
    f = open(outf, 'w')
    f.write(outStr)
    return outf


def update_robot(robot, bodypart_traj, timestep):
  bodypart_dofs = {}
  for bodypart, traj in bodypart_traj.items():
    bodypart_dofs[bodypart] = traj[timestep]

  for bodypart, dof_values in bodypart_dofs.items():
    if bodypart == 'base':
      robot.SetTransform(base_pose_to_mat(dof_values))
    elif bodypart in {'leftarm', 'rightarm'}:
      robot.SetDOFValues(constrain_within_joint_limits(robot, dof_values, arm=bodypart), robot.GetManipulator(bodypart).GetArmIndices())
    elif bodypart == 'torso':
      robot.SetDOFValues(dof_values, [robot.GetJoint('torso_lift_joint').GetDOFIndex()])
    else:
      env = robot.GetEnv()
      _, body_name, hinge_name = bodypart.split(':')
      body = env.GetKinBody(body_name)
      body.SetDOFValues(dof_values, [body.GetJoint(hinge_name).GetDOFIndex()])

def update_robot_dofs(robot, bodypart_dofs):
  bodypart_traj = {}
  for bodypart, dofs in bodypart_dofs.items():
    bodypart_traj[bodypart] = [dofs]

  update_robot(robot, bodypart_traj, 0)

def upsample_bodypart_traj(bodypart_traj, n):
  traj_up = {}
  for bodypart, traj in bodypart_traj.items():
    traj_up[bodypart] = mu.interp2d(
      np.linspace(0, 1, n), np.linspace(0, 1, len(traj)), traj)
  return traj_up

def run_trajectory(robot, traj, animationtime=1.0, n=300):
  """
  Crude way of executing trajectories and update the OpenRave environment
  without having to enable OpenRave simulation.
  """
  if animationtime == 0:
    update_robot(robot, traj, -1)
  else:
    traj_up = upsample_bodypart_traj(traj, n)
    sleep_time = float(animationtime) / n
    for i in range(n):
      update_robot(robot, traj_up, i)
      time.sleep(sleep_time)

def extend_dofs(robot, dofs, manip):
  """
  Extends a 7 DOF representation to a 14 DOF joint representation:
  rightarm(7) + leftarm(7)
  """
  if type(dofs) != list:
    dofs = dofs.tolist()

  if len(dofs) == 7:
    if manip == 'rightarm':
      dofs = dofs + robot.GetDOFValues(robot.GetManipulator("leftarm").GetArmIndices()).tolist()
    elif manip == 'leftarm':
      dofs = robot.GetDOFValues(robot.GetManipulator("rightarm").GetArmIndices()).tolist() + dofs

  return dofs

def extend_traj_dofs(robot, traj, manip):
  extended_traj = []
  for dofs in traj:
    dofs = extend_dofs(robot, dofs, manip)
    extended_traj.append(dofs)
  return extended_traj

def traj_to_bodypart_traj(traj, active_bodyparts):
  bodypart_traj = {}
  offset = 0
  for active_bodypart in sorted(active_bodyparts, key=lambda x: x if x != 'base' else '~~~~'):
    if active_bodypart in {'rightarm', 'leftarm'}:
      bodypart_traj[active_bodypart] = traj[:, range(offset, offset+7)]
      offset += 7
    elif active_bodypart == 'base':
      bodypart_traj['base'] = traj[:, range(offset, offset+3)]
      offset += 3
    else:
      bodypart_traj[active_bodypart] = traj[:, range(offset, offset+1)]
      offset += 1

  return bodypart_traj

def bodypart_traj_to_traj(bodypart_traj):
  traj_len = len(bodypart_traj.values()[0])
  traj = []
  for i in range(traj_len):
    bodypart_dofs = {}
    for bodypart, dofs in bodypart_traj.items():
      bodypart_dofs[bodypart] = list(dofs[i])
    traj.append(bodypart_dofs_to_dofs(bodypart_dofs))
  return traj

def dofs_to_bodypart_dofs(dofs, active_bodyparts):
  bodypart_dofs = {}
  offset = 0
  for active_bodypart in sorted(active_bodyparts, key=lambda x: x if x != 'base' else '~~~~'):
    if active_bodypart in {'rightarm', 'leftarm'}:
      bodypart_dofs[active_bodypart] = dofs[offset:offset+7]
      offset += 7
    elif active_bodypart == 'base':
      bodypart_dofs['base'] = dofs[offset:offset+3]
      offset += 3
    else:
      bodypart_dofs[active_bodypart] = dofs[offset:offset+1]
      offset += 1

  return bodypart_dofs

def bodypart_dofs_to_dofs(bodypart_dofs):
  dofs = []
  for bodypart in sorted(bodypart_dofs.keys(), key=lambda x: x if x != 'base' else '~~~~'):
    if bodypart in bodypart_dofs:
      dofs += bodypart_dofs[bodypart]
  return dofs

def remove_movable_objects(env, unmovable_objects, excluded=set()):
  for body in env.GetBodies():
    if not (body.IsRobot()) and (body not in unmovable_objects) and (body not in excluded):
      env.Remove(body)

def base_pose_to_mat(pose):
  x, y, rot = pose
  q = openravepy.quatFromAxisAngle((0, 0, rot)).tolist()
  pos = [x, y, 0]
  matrix = openravepy.matrixFromPose(q + pos)
  return matrix

def mat_to_base_pose(mat):
  pose = openravepy.poseFromMatrix(mat)
  x = pose[4]
  y = pose[5]
  rot = openravepy.axisAngleFromRotationMatrix(mat)[2]
  return x, y, rot

def base_good_enough(target_matrix, robot_matrix):
  target_pose = openravepy.poseFromMatrix(target_matrix)
  robot_pose = openravepy.poseFromMatrix(robot_matrix)
  rot_diff = openravepy.axisAngleFromQuat(target_pose[:4])[2] - openravepy.axisAngleFromQuat(robot_pose[:4])[2]
  x_diff = target_pose[4] - robot_pose[4]
  y_diff = target_pose[5] - robot_pose[5]
  pos_diff = np.sqrt(pow(x_diff, 2) + pow(y_diff, 2))
  print "pos_diff: {}, rot_diff: {}".format(pos_diff, rot_diff)
  return pos_diff <= 0.05 and rot_diff <= 0.04

def plot_transform(env, T, s=0.1):
    """
    Plots transform T in openrave environment.
    S is the length of the axis markers.
    """
    h = []
    x = T[0:3,0]
    y = T[0:3,1]
    z = T[0:3,2]
    o = T[0:3,3]
    h.append(env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
    h.append(env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
    h.append(env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    return h


def get_handoff_pose(robot, manip):
  torso_t = robot.GetLink('torso_lift_link').GetTransform()
  if manip == "leftarm":
    # return robot_t.dot(np.array([[-0.99994827, -0.00978572, -0.00277485, 0.33386002],
                                 # [-0.00984753, 0.9996821, 0.02321041, 0.2],
                                 # [0.00254684, 0.02323653, -0.99972675, 1.21657076],
                                 # [0, 0, 0, 1]]))
    return torso_t.dot(np.array([[-0.99994827, -0.00978572, -0.00277485,  0.38386002],
                                 [-0.00984753,  0.9996821 ,  0.02321041,  0.2       ],
                                 [ 0.00254684,  0.02323653, -0.99972675,  0.0],
                                 [ 0.        ,  0.        ,  0.        ,  1.        ]]))

  elif manip == "rightarm":
    # return robot_t.dot(np.array([[-0.99994827, 0.00978572, -0.00277485, 0.33386002],
                                 # [0.00984753, 0.9996821, -0.02321041, -0.2],
                                 # [0.00254684, -0.02323653, -0.99972675, 1.21657076],
                                 # [0, 0, 0, 1]]))
    return torso_t.dot(np.array([[-0.99994827,  0.00978572, -0.00277485,  0.38386002],
                                 [ 0.00984753,  0.9996821 , -0.02321041, -0.2       ],
                                 [ 0.00254684, -0.02323653, -0.99972675,  0.0],
                                 [ 0.        ,  0.        ,  0.        ,  1.        ]]))

def simulate_redetection(robot, traj):
  """
  Simulates base translation that happens during redetection.
  Amount of offset added is proportional to the length of the base traj
  """
  # TODO: simulate redetection of objects
  if traj is None:
    return

  DIST_ERROR_PER_METER = 0.0
  DIST_CONSTANT_ERROR = 0.01
  ANG_ERROR_PER_METER = 0.0
  ANG_CONSTANT_ERROR = 0.01

  total_distance = 0
  last_x = traj[0][14]
  last_y = traj[0][15]
  for dofs in traj:
    x = dofs[14]
    y = dofs[15]
    x_diff = x - last_x
    y_diff = y - last_y
    total_distance += np.sqrt(pow(x_diff, 2) + pow(y_diff, 2))
    last_x = x
    last_y = y

  print "Simulating redetection over base distance: {}".format(total_distance)

  base_x, base_y, base_rot = mat_to_base_pose(robot.GetTransform())
  dist_error = DIST_ERROR_PER_METER * total_distance + DIST_CONSTANT_ERROR
  random_direction = np.random.uniform(-np.pi, np.pi)
  base_x += dist_error * np.cos(random_direction)
  base_y += dist_error * np.sin(random_direction)

  ang_error = ANG_ERROR_PER_METER * total_distance + ANG_CONSTANT_ERROR
  base_rot += np.random.uniform(-ang_error, ang_error)

  robot.SetTransform(base_pose_to_mat((base_x, base_y, base_rot)))

def to_manip(arm_or_gripper):
  if arm_or_gripper in {'rarm', 'rgripper'}:
    return 'rightarm'
  elif arm_or_gripper in {'larm', 'lgripper'}:
    return 'leftarm'
  else:
    return 'rightarm'  # default to rightarm

def to_manip_better(arm_or_gripper):
  if arm_or_gripper in {'rarm', 'rgripper', 'rightarm'}:
    return 'rightarm'
  elif arm_or_gripper in {'larm', 'lgripper', 'leftarm'}:
    return 'leftarm'
  else:
    return None


def getStringFile(str):
    strPlanFileH = StringIO.StringIO()
    strPlanFileH.write(str)
    strPlanFileH.seek(0)
    return strPlanFileH

class TimeoutError(Exception):
    pass

def timeout(seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)
        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(seconds)
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result
        return wraps(func)(wrapper)
    return decorator

def get_adjusted_pose(robot, manip, adjust_dist):
  """ Returns pose of manip adjusted up or down by adjust_dist. """
  manip = to_manip_better(manip)
  if manip == 'rightarm':
    tool_frame = 'r_gripper_tool_frame'
  elif manip == 'leftarm':
    tool_frame = 'l_gripper_tool_frame'
  else:
    return

  tool_frame_t = robot.GetLink(tool_frame).GetTransform()
  adjust_t = openravepy.matrixFromPose([1, 0, 0, 0, 0, 0, adjust_dist])
  adjusted_pose = adjust_t.dot(tool_frame_t)

  # messy; cleanup. [EF]
  adjusted_pose = openravepy.poseFromMatrix(adjusted_pose)
  adjusted_pose[:4] = openravepy.quatMultiply(
      adjusted_pose[:4], openravepy.quatFromAxisAngle((0, np.pi/2, 0))).tolist()
  adjusted_pose = openravepy.matrixFromPose(adjusted_pose)

  return adjusted_pose

def constrain_within_joint_limits(robot, joints, arm='rightarm'):
  # arm_indices = robot.GetManipulator(arm).GetArmIndices()
  # lower_limits, upper_limits = robot.GetDOFLimits(arm_indices)
  arm_indices = robot.GetManipulator(arm).GetArmIndices()
  lower_limits, upper_limits = robot.GetDOFLimits(arm_indices)

  constrained_joints = []
  for joint, lower, upper in zip(joints, lower_limits, upper_limits):
    if joint > upper:
      constrained_joint = upper
    elif joint < lower:
      constrained_joint = lower
    else:
      constrained_joint = joint

    constrained_joints.append(constrained_joint)
  return constrained_joints

def set_rave_limits_to_soft_joint_limits(robot):
  # borrowed from rapprentice/PR2.py
  # make the joint limits match the PR2 soft limits
  low_limits, high_limits = robot.GetDOFLimits()
  rarm_low_limits = [-2.1353981634, -0.3536, -3.75, -2.1213, None, -2.0, None]
  rarm_high_limits = [0.564601836603, 1.2963, 0.65, -0.15, None, -0.1, None]
  for rarm_index, low, high in zip(robot.GetManipulator("rightarm").GetArmIndices(), rarm_low_limits, rarm_high_limits):
      if low is not None and high is not None:
          low_limits[rarm_index] = low
          high_limits[rarm_index] = high
  larm_low_limits = [-0.564601836603, -0.3536, -0.65, -2.1213, None, -2.0, None]
  larm_high_limits = [2.1353981634, 1.2963, 3.75, -0.15, None, -0.1, None]
  for larm_index, low, high in zip(robot.GetManipulator("leftarm").GetArmIndices(), larm_low_limits, larm_high_limits):
      if low is not None and high is not None:
          low_limits[larm_index] = low
          high_limits[larm_index] = high
  robot.SetDOFLimits(low_limits, high_limits)

def close_gripper(robot, manip):
  _set_gripper(robot, manip, 0.0)

def open_gripper(robot, manip, value=0.54):
  _set_gripper(robot, manip, value)

def _set_gripper(robot, manip, value):
  if manip == 'rightarm':
    joint = 'r_gripper_l_finger_joint'
  elif manip == 'leftarm':
    joint = 'l_gripper_l_finger_joint'
  robot.SetDOFValues([value], [robot.GetJointIndex(joint)])