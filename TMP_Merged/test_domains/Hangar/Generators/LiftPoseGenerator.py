import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.DataStructures.Generator import Generator
from GraspPoseGenerator import GraspPoseGenerator
import numpy as np


class LiftPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['object', 'grasp_pose']
        super(LiftPoseGenerator, self).__init__(known_argument_values, required_values)

        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('object')[0]
        self.grasp_pose = known_argument_values.get('grasp_pose')

        self.simulator = ll_state.simulator
        self.generate_function_state = self.generate_function()


    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        lst = self._compute_lift_pose()
        for i,gt in enumerate(lst):
            print "Lift Pose: "+str(i+1)+"/"+str(len(lst))
            yield gt

    def get_next(self,flag):
        return self.generate_function_state.next()

    def _compute_lift_pose(self):
        object_name = self.object_name
        orig_t = self.simulator.get_transform_matrix(object_name)
        gp_at_origin = self.grasp_pose.dot(np.linalg.inv(orig_t))
        lift_dist = 0.1
        t5 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, lift_dist)
        lift_pose = gp_at_origin.dot(t5)
        lift_pose_wrt_obj = lift_pose.dot(orig_t)
        return [lift_pose_wrt_obj]
