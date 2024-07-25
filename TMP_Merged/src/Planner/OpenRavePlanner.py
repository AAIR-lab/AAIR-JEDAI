from src.MotionPlanners import OpenRaveMotionPlanner
import Config
from Planner import Planner
import openravepy
from src.Solution.RobotAction import RobotAction
from src.Solution.LowLevelProblemSolution import LowLevelProblemSolution
from src.States.OpenRaveLowLevelState import OpenRaveLowLevelState
from src.IKSolvers.OpenRaveIKSolver import OpenRaveIKSolver

class OpenRavePlanner(Planner):

    def __init__(self, motion_planner=Config.OPENRAVE_NATIVE_MOTION_PLANNER):
        self.motion_planner = getattr(OpenRaveMotionPlanner, motion_planner)

    def solve(self, list_problems, state):
        combined_solution = None
        if len(list_problems) == 0:
            combined_solution = LowLevelProblemSolution(True,[],state)
            return combined_solution, None

        else:
            for problem in list_problems:
                # If first problem in list
                if combined_solution is None:
                    combined_solution = self.solve_problem(state, problem)
                    if combined_solution.success is False:
                        print "-->FAIL"
                        return combined_solution, problem
                    else:
                        print "-->PASS"
                        pass

                else:
                    prob_soln = self.solve_problem(combined_solution.state, problem)

                    if prob_soln.success is True:
                        print "-->PASS"
                        combined_solution.list_ll_plan.extend(prob_soln.list_ll_plan)
                        combined_solution.state = prob_soln.state
                    else:
                        print "-->FAIL"
                        combined_solution.success = False
                        return combined_solution, problem

        return combined_solution, None

    def solve_problem(self, state, ll_problem):
        print "\t\tProblem :" + str(ll_problem.arg),
        if ll_problem.generated_value_type == "GRIPPER_POSE":
            return self.solve_gripper_pose(state, ll_problem)

        elif ll_problem.generated_value_type == "MANIPULATOR_JOINT_VALUES":
            return self.solve_joint_values(state, ll_problem)

        elif ll_problem.generated_value_type == "ROBOT_BASE_POSE":
            return self.solve_base(state, ll_problem)

        elif ll_problem.generated_value_type == "OBJECT_POSE":
            return self.solve_object_pose(state, ll_problem)

        else:
            return self.motion_planner(state).solve(ll_problem)

    def solve_base(self, state, ll_problem):
        env, robot = state.get_env_and_robot()

        with env:
            robot.SetTransform(ll_problem.generated_value)
            if not env.CheckCollision(robot):
                new_ll_state = OpenRaveLowLevelState.init_from_env(env)
                ra = RobotAction(RobotAction.BASE_POSE_TYPE, ll_problem.generated_value)
                return LowLevelProblemSolution(True, [ra], new_ll_state)
            else:
                return LowLevelProblemSolution(False, [None], None)

    def solve_object_pose(self, state, ll_problem):
        env, robot = state.get_env_and_robot()
        max_itr = Config.MAX_IKs_TO_CHECK_FOR_MP
        iksols = OpenRaveIKSolver(state).get_ik_solutions(ll_problem.generated_value, check_collisions=False)
        trajobj = None
        new_ll_state = None
        if len(iksols) > 0:
            ik_index = 0
            while not trajobj and ik_index < len(iksols) and ik_index < max_itr:

                trajobj = self.motion_planner(state).get_mp_trajectory_to_goal(iksols[ik_index])

                if trajobj or Config.NO_MOTION_PLAN_CHECK:
                    with env:
                        robot.SetActiveDOFValues(iksols[ik_index])
                        new_ll_state = OpenRaveLowLevelState.init_from_env(env)
                    break
                else:
                    print "-NO MP-",

                ik_index = ik_index + 1
        else:
            print ("No IK Found for ll_problem.arg: "+str(ll_problem.arg))

        ra = None
        if trajobj:
            way_points = []
            for i in range(trajobj.GetNumWaypoints()):
                way_points.append(trajobj.GetWaypoint(i))
            ra = RobotAction(RobotAction.TRAJECTORY_TYPE, (way_points,trajobj.GetConfigurationSpecification()))

        result = len(iksols) > 0 and (trajobj is not None or Config.NO_MOTION_PLAN_CHECK)

        return LowLevelProblemSolution(result, [ra], new_ll_state)

    def solve_joint_values(self, state, ll_problem):
        new_ll_state = None
        env, robot = state.get_env_and_robot()

        trajobj = self.motion_planner(state).get_mp_trajectory_to_goal(ll_problem.generated_value)

        if trajobj or Config.NO_MOTION_PLAN_CHECK:
            with env:
                robot_dofs = robot.GetDOFValues()
                ary = ll_problem.generated_value.tolist()
                for i in robot.GetActiveManipulator().GetArmIndices():
                    robot_dofs[i] = ary.pop(0)

                robot.SetDOFValues(robot_dofs)
                new_ll_state = OpenRaveLowLevelState.init_from_env(env)
        else:
            print "-NO MP-",

        result = trajobj is not None or Config.NO_MOTION_PLAN_CHECK
        ra = None
        if result:
            way_points = []
            for i in range(trajobj.GetNumWaypoints()):
                way_points.append(trajobj.GetWaypoint(i))
            ra = RobotAction(RobotAction.TRAJECTORY_TYPE, (way_points, trajobj.GetConfigurationSpecification()))

        return LowLevelProblemSolution(result, [ra], new_ll_state)

    def solve_gripper_pose(self, state, ll_problem):
        env, robot = state.get_env_and_robot()

        object_to_grab = None
        # taskmanip = interfaces.TaskManipulation(self.robot)
        l_ind = robot.GetJointIndex('l_gripper_finger_joint')
        r_ind = robot.GetJointIndex('r_gripper_finger_joint')
        if ll_problem.generated_value == 'open':
            with env:
                # taskmanip.ReleaseFingers()
                robot.SetDOFValues([0.05, 0.05], [l_ind, r_ind])
                robot.ReleaseAllGrabbed()
                robot.WaitForController(0)

        elif ll_problem.generated_value == 'close':
            with env:
                taskmanip = openravepy.interfaces.TaskManipulation(robot)
                taskmanip.CloseFingers()
                # self.robot.SetDOFValues([0.0, 0.0], [l_ind, r_ind])
                object_to_grab = ll_problem.hl_args[0]
                robot.Grab(env.GetKinBody(object_to_grab))
        robot.WaitForController(0)

        # TODO Collision reports wrong collisions
        # if True or  ll_problem.generated_value == 'close' or not self.env.CheckCollision(self.robot):
        with env:
            new_ll_state = OpenRaveLowLevelState.init_from_env(env)

        ra = RobotAction(RobotAction.GRIPPER_STATE_TYPE, (ll_problem.generated_value, object_to_grab))
        return LowLevelProblemSolution(True, [ra], new_ll_state)

        # RaveSetDebugLevel(DebugLevel.Info)
        # # robot.GetEnv().Remove(robot.GetEnv().GetKinBody('terrain'))
        # # with robot.GetEnv():
        # try:
        #     basemanip = interfaces.BaseManipulation(robot)
        #     g = poseFromMatrix(transform)[-3:]
        #     g[-1] = 0
        #     robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
        #     basemanip.MoveActiveJoints(goal=g, maxiter=5000, steplength=0.15, maxtries=2)
        #     robot.WaitForController(0)
        # except:
        #     pass
        #
        # robot.SetTransform(transform)

