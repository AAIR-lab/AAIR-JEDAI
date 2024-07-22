from src.MotionPlanners import MotionPlannerBase
import random
import src.OpenraveUtils as OpenraveUtils
import Config
import openravepy as orpy

class OpenRave_NativeMotion_Planner(object):

    def __init__(self, env, robot):
        self.env = env

        collision_checker = orpy.RaveCreateCollisionChecker(self.env, "pqp")
        collision_checker.SetCollisionOptions(orpy.CollisionOptions.AllGeometryContacts)
        self.env.SetCollisionChecker(collision_checker)


    def get_mp_trajectory_to_goal(self, goal_joint_values):
        with self.env:
            with self.env:
                try:
                    trajobj = orpy.interfaces.BaseManipulation(self.robot).MoveManipulator(goal_joint_values, outputtrajobj=True, execute=Config.EXECUTE_MOTION_PLANS)
                except Exception as ex:
                    trajobj = None

        return trajobj.serialize()

    def get_objects_in_collision(self, ik_solutions, list_non_movable_object_names):
        found=[]
        random.shuffle(ik_solutions)
        i=0
        while not found:
            if i >= min(len(ik_solutions), Config.MAX_IKs_TO_CHECK_FOR_MP):
                return []
            ik_solution = ik_solutions[i]
            found =  OpenraveUtils.get_motion_plan_collisions(self.env.CloneSelf(orpy.CloningOptions.Bodies), 'fetch', ik_solution, list_non_movable_object_names)
            i = i + 1

        return found


class OpenRave_OMPL_Base_Planner(OpenRave_NativeMotion_Planner):
    def __init__(self, env, robot, planner='OMPL_RRTConnect'):
        super(OpenRave_OMPL_Base_Planner, self).__init__(env, robot)
        self.planner = orpy.RaveCreatePlanner(self.env, planner)
        self.simplifier = orpy.RaveCreatePlanner(self.env, 'OMPL_Simplifier')
        self.planner_params = orpy.Planner.PlannerParameters()

        # Set the timeout and planner-specific parameters. You can view a list of
        # supported parameters by calling: planner.SendCommand('GetParameters')
        self.planner_params.SetExtraParameters('<range>100</range>')
        self.planner_params.SetExtraParameters('<time_limit>180</time_limit>')

    def get_ik_solutions_to_goal(self, goal_transform, check_collisions=True):
        return super(OpenRave_OMPL_Base_Planner, self).get_mp_trajectory_to_goal(goal_transform)

    def get_mp_trajectory_to_goal(self, robot,goal_transform):
        orpy.raveLogInfo("Attempting to find trajectory to : " + str(goal_transform))

        # Record the initial dofs, TODO: find a way to stop the robot from chaning the model during planning
        inital_robot_dofs = robot.GetDOFValues()

        cg = orpy.ConfigurationSpecification(robot.GetConfigurationSpecification())
        if Config.ROBOT_NAME == "fetch":
            cg.RemoveGroups('affine_transform fetch 39')
        elif Config.ROBOT_NAME == "UAV":
            cg.RemoveGroups('affine_transform UAV 39')# Remove because of not supported error
        elif Config.ROBOT_NAME == 'yumi':
            cg.RemoveGroups('affine_transform yumi 39')
        self.planner_params.SetConfigurationSpecification(self.env, cg)

        self.planner_params.SetRobotActiveJoints(robot)
        # self.planner_params.SetInitialConfig(self.robot.GetDOFValues())
        self.planner_params.SetConfigAccelerationLimit(robot.GetActiveDOFMaxAccel())
        self.planner_params.SetConfigVelocityLimit(robot.GetActiveDOFMaxVel())



        self.planner_params.SetGoalConfig(goal_transform)

        try:
            self.planner.InitPlan(robot, self.planner_params)
        except Exception,e:
            print e
            # import IPython
            # IPython.embed()

        traj = orpy.RaveCreateTrajectory(self.env, '')
        result = self.planner.PlanPath(traj)

        if (result == orpy.PlannerStatus.HasSolution):
            # Shortcut the path.
            self.simplifier.InitPlan(robot, orpy.Planner.PlannerParameters())
            # import IPython
            # IPython.embed()
            result = self.simplifier.PlanPath(traj)
            if (result == orpy.PlannerStatus.HasSolution):
                # Time the trajectory.
                result = orpy.planningutils.RetimeTrajectory(traj)
        else:
            robot.SetDOFValues(inital_robot_dofs)
            # import IPython
            # IPython.embed()
            raise Exception

        # Reset all the dancing around
        robot.SetDOFValues(inital_robot_dofs)

        fail_cause = None
        return (traj.serialize(), result == orpy.PlannerStatus.HasSolution, fail_cause)


# class OpenRave_OMPL_RRT_Connect_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_RRT_Connect_Planner, self).__init__(model, 'OMPL_RRTConnect')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_RRT_Connect_Planner, self).get_mp_trajectory_to_goal(goal_transform)


class OpenRave_OMPL_RRT_Connect_Planner(OpenRave_OMPL_Base_Planner):
    def __init__(self, env, robot):
        super(OpenRave_OMPL_RRT_Connect_Planner, self).__init__( env, robot, 'OMPL_RRTConnect')

    def get_mp_trajectory_to_goal(self,robot, goal_transform):
        return super(OpenRave_OMPL_RRT_Connect_Planner, self).get_mp_trajectory_to_goal(robot,goal_transform)

#
# class OpenRave_OMPL_pRRT_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_pRRT_Planner, self).__init__(model, 'OMPL_pRRT')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_pRRT_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_EST_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_EST_Planner, self).__init__(model, 'OMPL_EST')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_EST_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_FMT_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_FMT_Planner, self).__init__(model, 'OMPL_FMT')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_FMT_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_PRM_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_PRM_Planner, self).__init__(model, 'OMPL_PRM')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_PRM_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_pSBL_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_pSBL_Planner, self).__init__(model, 'OMPL_pSBL')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_pSBL_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_SBL_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_SBL_Planner, self).__init__(model, 'OMPL_SBL')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_SBL_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_TRRT_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_TRRT_Planner, self).__init__(model, 'OMPL_TRRT')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_TRRT_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_SPARS_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_SPARS_Planner, self).__init__(model, 'OMPL_SPARS')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_SPARS_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_SPARStwo_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_SPARStwo_Planner, self).__init__(model, 'OMPL_SPARStwo')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_SPARStwo_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_RRTstar_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_RRTstar_Planner, self).__init__(model, 'OMPL_RRTstar')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_RRTstar_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_BKPIECE1_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_BKPIECE1_Planner, self).__init__(model, 'OMPL_BKPIECE1')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_BKPIECE1_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_LazyPRM_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_LazyPRM_Planner, self).__init__(model, 'OMPL_LazyPRM')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_LazyPRM_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_PRMstar_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_PRMstar_Planner, self).__init__(model, 'OMPL_PRMstar')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_PRMstar_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_KPIECE1_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_KPIECE1_Planner, self).__init__(model, 'OMPL_KPIECE1')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_KPIECE1_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_LBKPIECE1_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_LBKPIECE1_Planner, self).__init__(model, 'OMPL_LBKPIECE1')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_LBKPIECE1_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_LazyRRT_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_LazyRRT_Planner, self).__init__(model, 'OMPL_LazyRRT')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_LazyRRT_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_PDST_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_PDST_Planner, self).__init__(model, 'OMPL_PDST')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_PDST_Planner, self).get_mp_trajectory_to_goal(goal_transform)
#
#
# class OpenRave_OMPL_PDST_Planner(OpenRave_OMPL_Base_Planner):
#     def __init__(self, model):
#         super(OpenRave_OMPL_PDST_Planner, self).__init__(model, 'OMPL_PDST')
#
#     def get_mp_trajectory_to_goal(self, goal_transform):
#         return super(OpenRave_OMPL_PDST_Planner, self).get_mp_trajectory_to_goal(goal_transform)
