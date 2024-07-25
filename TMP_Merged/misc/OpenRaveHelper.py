# from openravepy import *
# import numpy as np
# import util
# import random
# import OpenraveUtils
# import pickle
# import os.path
#
# RaveSetDebugLevel(DebugLevel.Error)
#
#
# def get_motion_plan_collisions(env, robot_name, ik_solution, list_non_movable_object_names):
#     colliding_object_names = set()
#     env_clone = env.CloneSelf(openravepy.CloningOptions.Bodies)
#     robot = env_clone.GetRobot(robot_name)
#     for body in env_clone.GetBodies():
#         if body.GetName() in list_non_movable_object_names or 'object' not in body.GetName(): #TODO, Genralize
#             continue
#         env_clone.Remove(body)
#
#     try:
#         trajobj = openravepy.interfaces.BaseManipulation(robot).MoveManipulator(ik_solution, outputtrajobj=True, execute=False)
#         waypoints_count = trajobj.GetNumWaypoints()
#         env_clone = env.CloneSelf(openravepy.CloningOptions.Bodies)
#         robot = env_clone.GetRobot('fetch')
#         for i in range(waypoints_count):
#             cr = openravepy.CollisionReport()
#             robot.SetActiveDOFValues(get_joint_values_from_waypoint(trajobj.GetWaypoint(i)))
#             env_clone.CheckCollision(robot, cr)
#             colliding_links = [cr.plink1, cr.plink2]
#             for link in colliding_links:
#                 if link is None:
#                     continue
#                 name = link.GetParent().GetName()
#                 if name != robot.GetName():
#                     colliding_object_names.add(name)
#     except:
#         return False
#
#     return list(colliding_object_names)
#
#
#
# def getGraspTransforms(openrave_ll_state, robot_name, obj_name, return_count=None, object_top=True, object_middle=True, object_bottom=True):
#
#     env, robot = openrave_ll_state.get_env_and_robot()
#     with env:
#         assert env.GetKinBody(obj_name) is not None, "Could not find kinbody named " + obj_name + " in OpenRave environment"
#         gmodel = generateGraspModel(robot, env.GetKinBody(obj_name))
#         if (return_count):
#             validgrasps, validindices = gmodel.computeValidGrasps(returnnum=return_count)
#         else:
#             validgrasps, validindices = gmodel.computeValidGrasps(returnnum=1)
#
#         count = len(validgrasps)
#
#         # assert (len(validgrasps) > 0), "Could not find "+str(return_count)+" valid grasps using the generated grasp model"
#
#         if (not len(validgrasps) > 0):
#             print "Couldn't find valid grasps for " + str(obj_name)
#             transforms = []
#         else:
#
#             transforms = [gmodel.getGlobalGraspTransform(v, collisionfree=True) for v in validgrasps]
#             if (not (object_top and object_middle and object_bottom)):
#                 sorted_transforms = src.aair_ros_pkg.src.OpenraveUtils.sortGrasps_TranslationZ(transforms)
#                 transforms = []
#                 if (object_top):
#                     transforms.extend(sorted_transforms[:count / 3])
#
#                 if (object_middle):
#                     transforms.extend(sorted_transforms[count / 3:2 * count / 3])
#
#                 if (object_bottom):
#                     transforms.extend(sorted_transforms[count * 2 / 3:])
#
#     return transforms
#
#
# def generateGraspModel(robot,target):
#     "Generates the grasp model for the target which is of type kinbody"
#
#     ikmodel = generateIKModel(robot,IkParameterization.Type.Transform6D)
#     gmodel = databases.grasping.GraspingModel(robot, target)
#
#     raveLogInfo("Try loading  " + str(gmodel))
#
#     if not gmodel.load():
#         raveLogInfo("Generating grasp for " + str(target))
#         gmodel.generate()
#         gmodel.save()
#
#     gmodel.manip.SetIkSolver(ikmodel.iksolver)
#
#     return gmodel
#
#
# def generateIKModel(robot,invkintype):
#     ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=invkintype)
#     if not ikmodel.load():
#         raveLogInfo("Generating IKModel for " + str(robot) + " with type " + str(invkintype))
#         ikmodel.autogenerate()
#
#     return ikmodel
#
# def generate_gripper_state_transforms(openrave_ll_state, state='open'):
#     _, robot = openrave_ll_state.get_env_and_robot()
#     taskmanip = interfaces.TaskManipulation(robot)
#     robot_transforms = []
#     if state == 'open':
#         with robot:
#             taskmanip.ReleaseFingers()
#         robot.WaitForController(0)
#         robot_transforms.append(robot.GetDOFValues())
#     elif state == 'close':
#         with robot:
#             taskmanip.CloseFingers()
#         robot.WaitForController(0)
#         robot_transforms.append(robot.GetDOFValues())
#
#     return robot_transforms
#
# def generate_base_poses_around_object(openrave_ll_state, object_name):
#     # return [np.eye(4)] #TODO testing base no move
#
#
#
#     torso_heights = np.linspace(0.31, 0.31, num=1)
#     body_offset = np.linspace(0.55, 0.75, num=2)  # distance offset from edge of body
#     sampling_dist = 0.5
#     env, robot = openrave_ll_state.get_env_and_robot()
#     body = env.GetKinBody(object_name)
#
#     pose_list = []
#     with env:
#         body = env.GetKinBody(object_name)
#         if body is not None:
#             # do all computation with body at origin
#             # so AABBs actually line up to rectangular bodies
#             # body.SetTransform(np.identity(4))
#             # body.SetDOFValues([0] * len(body.GetDOFValues()))
#
#             min_x, max_x, min_y, max_y, _ = util.get_object_limits(body)
#
#         else:
#             min_x = 0
#             max_x = 0
#             min_y = 0
#             max_y = 0
#
#         for offset in body_offset:
#             min_x2 = min_x - offset
#             max_x2 = max_x + offset
#             min_y2 = min_y - offset
#             max_y2 = max_y + offset
#
#             num_x = np.ceil((max_x2 - min_x2) / sampling_dist)
#             num_y = np.ceil((max_y2 - min_y2) / sampling_dist)
#             x_range = np.linspace(min_x2, max_x2, num=num_x)
#             y_range = np.linspace(min_y2, max_y2, num=num_y)
#
#             coordinates = set()
#             for x in x_range:
#               coordinates.add((x, y_range[0]))
#               coordinates.add((x, y_range[-1]))
#             for y in y_range:
#               coordinates.add((x_range[0], y))
#               coordinates.add((x_range[-1], y))
#
#             min_x3 = min_x - offset / 2
#             max_x3 = max_x + offset / 2
#             min_y3 = min_y - offset / 2
#             max_y3 = max_y + offset / 2
#             for x, y in coordinates:
#               # TODO: figure out a better way to do this...
#               if x >= max_x3 and y >= max_y3:
#                 rot = -3*np.pi/4
#               elif x >= max_x3 and y <= min_y3:
#                 rot = 3*np.pi/4
#               elif x <= min_x3 and y >= max_y3:
#                 rot = -np.pi/4
#               elif x <= min_x3 and y <= min_y3:
#                 rot = np.pi/4
#               elif x >= max_x3:
#                 rot = np.pi
#               elif x <= min_x3:
#                 rot = 0
#               elif y >= max_y3:
#                 rot = -np.pi/2
#               elif y <= min_y3:
#                 rot = np.pi/2
#               else:
#                 continue
#
#               quat = quatFromAxisAngle((0, 0, rot)).tolist()
#               base_t = matrixFromPose(quat + [x, y, 0])
#               robot.SetTransform(base_t)
#               if not env.CheckCollision(robot):
#                 pose_list.append(base_t)
#     random.shuffle(pose_list)
#
#     return pose_list
#
#
# def generate_grasp_pose_for_object(openrave_ll_state, object_name):
#     # generate poses in the object frame
#     env, robot = openrave_ll_state.get_env_and_robot()
#     pickle_str = str(env) + "generate_grasp_pose_for_object"
#     with env:
#         obj = env.GetKinBody(object_name)
#         link = obj.GetLink('base')
#         geom = link.GetGeometries()[0]
#         cylinder_height = geom.GetCylinderHeight()
#
#         orig_t = obj.GetTransform()
#         obj.SetTransform(np.identity(4))
#
#         obj.SetTransform(orig_t)
#         num_grasps = 16
#         height_offset = 0
#         dist_offset = 0.17
#
#         pose_list = []
#         for i in range(2, num_grasps):
#           rot_ang = i * (2 * np.pi) / num_grasps
#
#           t2 = matrixFromPose((1, 0, 0, 0, -dist_offset, 0, cylinder_height))
#           r2 = matrixFromAxisAngle((0, 0, rot_ang))
#           t3 = matrixFromPose((1, 0, 0, 0, 0, 0, height_offset))
#           grasp_pose = t3.dot(r2).dot(t2)
#           grasp_pose = obj.GetTransform().dot(grasp_pose)
#           if OpenraveUtils.has_ik_to(env,robot,grasp_pose):
#             pose_list.append(grasp_pose)
#
#     random.shuffle(pose_list)
#     return pose_list
#
# def generate_pre_grasp_pose(openrave_ll_state, object_name, grasp_pose):
#     env, robot = openrave_ll_state.get_env_and_robot()
#     with env:
#         obj = env.GetKinBody(object_name)
#         orig_t = obj.GetTransform()
#         gp_at_origin = grasp_pose.dot(np.linalg.inv(orig_t))
#
#         approach_dist = 0.03
#         t4 = matrixFromPose((1, 0, 0, 0, -approach_dist, 0, 0))
#         pre_grasp_pose = gp_at_origin.dot(t4)
#         pre_grasp_pose_wrt_obj = pre_grasp_pose.dot(orig_t)
#     return [pre_grasp_pose_wrt_obj]
#
#
# def generate_lift_pose(openrave_ll_state, object_name, grasp_pose):
#     env, robot = openrave_ll_state.get_env_and_robot()
#     with env:
#         obj = env.GetKinBody(object_name)
#         orig_t = obj.GetTransform()
#         gp_at_origin = grasp_pose.dot(np.linalg.inv(orig_t))
#
#         lift_dist = 0.1
#         t5 = matrixFromPose((1, 0, 0, 0, 0, 0, lift_dist))
#         lift_pose = gp_at_origin.dot(t5)
#         lift_pose_wrt_obj = lift_pose.dot(orig_t)
#     return [lift_pose_wrt_obj]
#
#
# def generate_put_down_pose(openrave_ll_state, table_name, cylinder_name):
#     generated_values_list = []
#
#     env, robot = openrave_ll_state.get_env_and_robot()
#     with env:
#         obj = env.GetKinBody(cylinder_name)
#         geom = obj.GetLink('base').GetGeometries()[0]
#
#         o_x, o_y, o_z = geom.GetBoxExtents().tolist()
#         cylinder_radius = o_x
#         cylinder_height = geom.GetCylinderHeight()
#
#         t = env.GetKinBody(table_name)
#         table_transform = t.GetTransform()
#         link = t.GetLink('base')
#         geom = link.GetGeometries()
#         table_top = geom[0]
#         box_extents_xyz = table_top.GetBoxExtents()
#
#         x, y, z = box_extents_xyz.tolist()
#         x_limit = x - cylinder_radius
#         y_limit = y - cylinder_radius
#
#         for i in range(20):
#             obj_x = random.uniform(-x_limit, x_limit)
#             obj_y = random.uniform(-y_limit, y_limit)
#             obj_z = z + cylinder_height
#
#             obj_transform_wrt_origin = matrixFromPose([1, 0, 0, 0, obj_x, obj_y, obj_z])
#             obj_transform_wrt_table = obj_transform_wrt_origin.dot(table_transform)
#
#             if OpenraveUtils.has_ik_to(env, robot, obj_transform_wrt_table):
#                 generated_values_list.append(obj_transform_wrt_table)
#     random.shuffle(generated_values_list)
#     return generated_values_list
#
#
#
#
