
from openravepy import *
import copy


class RobotAction(object):
    TRAJECTORY_TYPE = 'ROBOT_ACTION_TRAJECTORY_TYPE'
    BASE_POSE_TYPE = 'ROBOT_ACTION_BASE_POSE_TYPE'
    GRIPPER_STATE_TYPE = 'ROBOT_ACTION_GRIPPER_STATE_TYPE'
    DOF_TYPE = 'DOF_TYPE'

    def __init__(self, type, value):
        self.type = type
        self.value = value

    def execute(self, openrave_ll_state, env):

        robot = env.GetRobots()[0]

        if self.type == RobotAction.BASE_POSE_TYPE:
            self.move_to_location(robot, self.value)
        elif self.type == RobotAction.TRAJECTORY_TYPE:
            self.move_manipulator(robot, self.value)
        elif self.type == RobotAction.DOF_TYPE:
            self.set_dof(robot, self.value)
        elif self.type == RobotAction.GRIPPER_STATE_TYPE:
            self.actuate_gripper(robot, self.value)
        return env

    def move_to_location(self, robot, transform):
        RaveSetDebugLevel(DebugLevel.Info)
        # robot.GetEnv().Remove(robot.GetEnv().GetKinBody('terrain'))
        # with robot.GetEnv():
        try:
            basemanip = interfaces.BaseManipulation(robot)
            g = poseFromMatrix(transform)[-3:]
            g[-1] = 0
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
            basemanip.MoveActiveJoints(goal=g, maxiter=5000, steplength=0.15, maxtries=2)
            robot.WaitForController(0)
        except:
            pass

        robot.SetTransform(transform)

    def move_manipulator(self, robot, trajectory):
        t = RaveCreateTrajectory(robot.GetEnv(), '')
        t.Init(trajectory[1])
        for i in range(len(trajectory[0])):
            t.Insert(i, trajectory[0][i])
        robot.GetController().SetPath(t)
        robot.WaitForController(0)

    def set_dof(self, robot, dof):
        robot.SetDOFValues(dof)

    def actuate_gripper(self, robot, value):
        taskmanip = interfaces.TaskManipulation(robot)
        state = value[0]
        obj_to_grab = value[1]
        if state == 'open':
            with robot:
                taskmanip.ReleaseFingers()
                robot.ReleaseAllGrabbed()
            robot.WaitForController(0)
        elif state == 'close':
            with robot:
                taskmanip.CloseFingers()
                robot.Grab(robot.GetEnv().GetKinBody(obj_to_grab))
            robot.WaitForController(0)


    def __deepcopy__(self, memodict={}):
        ra = RobotAction(None,None)
        ra.type = copy.deepcopy(self.type)
        ra.value = copy.deepcopy(self.value)
        return ra
