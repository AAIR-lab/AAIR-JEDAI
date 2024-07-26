import util
import openravepy


def shiftGraspInObjectFrame(object_, graspT, shiftX=0, shiftY=0, shiftZ=0):
    Object_T_Grasp = getGraspTransformInObjectFrame(object_, graspT)

    pose = poseFromMatrix(Object_T_Grasp)
    pose[4] = pose[4] + shiftX
    pose[5] = pose[5] + shiftY
    pose[6] = pose[6] + shiftZ
    Object_T_Grasp_Dash = matrixFromPose(pose)

    World_T_Grasp = getGraspTransformInWorldFrame(Object_T_Grasp_Dash)

    return World_T_Grasp


def getGraspTransformInObjectFrame(object_, graspTransform):
    World_T_Object = object_.GetTransform()
    Object_T_World = np.linalg.inv(World_T_Object)
    World_T_Grasp = graspTransform
    Object_T_Grasp = Object_T_World.dot(World_T_Grasp)
    return Object_T_Grasp


def getGraspTransformInWorldFrame(Object_T_Grasp):
    """ Input grasp in object frame"""
    Grasp_T_Object = np.linalg.inv(Object_T_Grasp)
    Grasp_T_World = Grasp_T_Object.dot(Object_T_World)
    World_T_Grasp = np.linalg.inv(Grasp_T_World)
    return World_T_Grasp


def sortGrasps_TranslationZ(grasp_transforms):
    poses = [openravepy.poseFromMatrix(t) for t in grasp_transforms]
    poses = util.sortArrays(poses, 6)
    transforms = [openravepy.matrixFromPose(p) for p in poses]
    return transforms


def get_motion_plan_collisions(env, robot_name, ik_solution, list_non_movable_object_names):
    colliding_object_names = set()
    removed_bodies = set()
    with env:
        robot = env.GetRobot(robot_name)
        for body in env.GetBodies():
            if body.GetName() in list_non_movable_object_names or 'object' not in body.GetName(): #TODO, Genralize
                continue
            removed_bodies.add(body)
            env.Remove(body)

    try:
        trajobj = openravepy.interfaces.BaseManipulation(robot).MoveManipulator(ik_solution, outputtrajobj=True, execute=False)
        for b in removed_bodies:
            env.Add(b)
        waypoints_count = trajobj.GetNumWaypoints()
        # env = env.CloneSelf(openravepy.CloningOptions.Bodies)
        robot = env.GetRobot('fetch')
        for i in range(waypoints_count):
            cr = openravepy.CollisionReport()
            with env:
                robot.SetActiveDOFValues(get_joint_values_from_waypoint(trajobj.GetWaypoint(i)))
                env.CheckCollision(robot, cr)
            colliding_links = [cr.plink1, cr.plink2]
            for link in colliding_links:
                if link is None:
                    continue
                name = link.GetParent().GetName()
                if name != robot.GetName():
                    colliding_object_names.add(name)
    except:
        return False

    return list(colliding_object_names)


def get_joint_values_from_waypoint(waypoint):
    return waypoint[0:8]


def has_ik_to(env, robot, goal_transform, check_collisions=False):
    filter_option = openravepy.IkFilterOptions.CheckEnvCollisions
    # filter_option = None
    if not check_collisions:
        filter_option = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        openravepy.raveLogInfo("Generating IKModel for " + str(robot) )
        ikmodel.autogenerate()

    with env:
        sols = ikmodel.manip.FindIKSolutions(goal_transform, filter_option)
        # sols = ikmodel.manip.FindIKSolutions(goal_transform)



    return len(sols) > 0