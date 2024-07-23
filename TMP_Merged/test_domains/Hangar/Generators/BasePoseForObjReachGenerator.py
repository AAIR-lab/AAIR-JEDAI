from src.DataStructures.Generator import Generator
import numpy as np


class BasePoseForObjReachGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current']
        super(BasePoseForObjReachGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            print "Generating base pose for reaching: "+str(self.known_argument_values.get('object'))
            yield [np.eye(4)]

    def get_next(self,flag):
        return self.generate_function_state.next()

        # lst = OpenRaveHelper.generate_base_poses_around_object(self.__ll_state, self.__object_name)
        # for gt in lst:
        #     yield gt


