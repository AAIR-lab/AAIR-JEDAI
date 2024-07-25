from src.States.State import State
import copy
import Config
from src.Robots.Models import FetchOpenRaveRobotModel
import openravepy
from src.Simulators.OpenRaveSimulator import OpenRaveSimulator
from openravepy.misc import InitOpenRAVELogging


InitOpenRAVELogging()




class OpenRaveLowLevelState(State):
    cached_env = None
    cached_obj_name_transform_map = {}
    cached_obj_name_instance_map = {}

    def __init__(self, universe=None, functions=None, values=None,battery = 1500):
        super(OpenRaveLowLevelState, self).__init__(universe, functions, values)
        self.universe = universe
        self.functions = functions
        self.simulator = OpenRaveSimulator(Config.OPENRAVE_ENV_XML)
        if values is None:
            self.values = self.get_values_from_env(self.simulator.env)
            self.setup_initial_camera_view()
        else:
            self.values = values
        self.history = []
        self.ll_history = []
        self.ll_variables = {}

    def setup_initial_camera_view(self):
                    
        assert self.values is not None
        assert isinstance(self.values, dict)
        
        assert self.simulator is not None
        assert self.simulator.env is not None
        viewer = self.simulator.env.GetViewer()
        
        if "objects" in self.values and "table_blue" in self.values["objects"]:
        
            # Cafeworld camera angle setup.    
            viewer.SetCamera(
                [[ -1.26668234e-02,   7.10505939e-01,  -7.03577190e-01, 8.80096436e+00],
                 [  9.99905994e-01,   5.30663706e-03,  -1.26428709e-02, -4.04841423e+00],
                 [ -5.24920604e-03,  -7.03671195e-01,  -7.10506366e-01, 7.85888958e+00],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
        elif "robots" in self.values and "yumi" in self.values["robots"]:
            
            # Keva camera angle setup.
            viewer.SetCamera(
                [[ 0.01124211,  0.48109715, -0.8765952 ,  1.10629892],
                 [ 0.99988877,  0.00318416,  0.01457086, -0.05577996],
                 [ 0.00980122, -0.8766615 , -0.48100784,  0.82077068],
                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
        else:
            
            pass


    def set_values(self, vals):
        self.values = vals


    def apply(self, argument, other_generated_values): # other_generated_values is only used to find out which object to Grab
        self.history.append(self.values)
        self.ll_history.append(self.ll_variables)
        argument.apply(self,other_generated_values)
        self.values = self.get_values_from_env(self.simulator.env)

    def execute(self,argument,other_generated_values):
        argument.execute(self,other_generated_values)

    def apply_effect(self,effect,generated_values):
        self.history.append(self.values)
        self.ll_history.append(self.ll_variables)
        effect.apply(self,generated_values)
        self.values = self.get_values_from_env(self.simulator.env)

    def rollback(self):
        try:
            self.values = self.history.pop()
            self.simulator.set_environment(self.values)
            self.ll_variables = self.ll_history.pop()
        except:
            pass

    def sync_simulator(self,old_state = None):
        if old_state is None:
            self.simulator.set_environment(self.values)
        else:
            if type(old_state) == type({}):
                self.simulator.set_environment(old_state)
            else:
                self.simulator.set_environment(old_state.values)



    def get_values_from_env(self, openrave_env = None):
        if openrave_env is None:
            openrave_env = self.simulator.env
        robots = {}
        objects = {}

        with openrave_env:
            robot = None
            for body in openrave_env.GetBodies():
                transform = body.GetTransform()
                name = body.GetName()
                if body.IsRobot():
                    assert name not in robots, "ERROR, Openrave env_json has duplicate robot names"
                    robot = openrave_env.GetRobot(name)
                    robots[name] = {}
                    if name == 'yumi':
                        robots[name]['transform'] = robot.GetTransform()
                    else:
                        robots[name]['transform'] = robot.GetLink('base_link').GetTransform()
                    robots[name]['dof_values'] = robot.GetDOFValues()
                    if len(robot.GetGrabbed()) > 0:
                        robots[name]['grabbed_objects'] = [o.GetName() for o in robot.GetGrabbed()]
                    robots[name]["active_arm"] = robot.GetActiveManipulator().GetName()
                    robots[name]["active_joint_indices"] = robot.GetActiveJointIndices()

                else:
                    assert name not in objects, "ERROR, Openrave env_json has duplicate object names"
                    objects[name] = {}
                    objects[name]['transform'] = transform


        env_json = {}
        env_json['robots'] = robots
        env_json['objects'] = objects
        # env_json['xml'] = Config.OPENRAVE_CREATED_ENV_SAVE_FILE

        return env_json

    def __deepcopy__(self, memodict={}):
        uni_cpy = copy.deepcopy(self.universe)
        fun_cpy = copy.deepcopy(self.functions)
        val_cpy = self.values.copy()
        new_state = OpenRaveLowLevelState(universe=uni_cpy, functions=fun_cpy, values=val_cpy)
        new_state.ll_variables = copy.deepcopy(self.ll_variables)
        return new_state
        # return self.values.copy()



