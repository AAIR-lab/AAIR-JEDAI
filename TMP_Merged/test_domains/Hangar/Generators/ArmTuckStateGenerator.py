import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.DataStructures.Generator import Generator
from src.Robots.Models import FetchConfig
import numpy as np

class ArmTuckStateGenerator(Generator):
    def __init__(self, ll_state=None):
        super(ArmTuckStateGenerator, self).__init__()
        self.__ll_state = ll_state
        self.__yielder = None
        self.type = "MANIPULATOR_JOINT_VALUES"
        self.fetch_config = FetchConfig()

    def __deepcopy__(self, memodict={}):
        ll_state_cpy = copy.deepcopy(self.__ll_state)
        atk_cpy = ArmTuckStateGenerator(ll_state_cpy)
        return atk_cpy

    def configure(self, args=None, ll_state=None):
        super(ArmTuckStateGenerator, self).configure()
        self.__ll_state = ll_state

    def reset(self):
        super(ArmTuckStateGenerator, self).reset()
        self.__yielder = None

    def get_next(self, required_values_map=None):
        if self.__yielder is None:
            self.__yielder = self.__generate()

        genvalue = self.__yielder.next()
        self.store(genvalue)
        return genvalue

    def __generate(self):
        for s in [np.asarray(self.fetch_config.armSecureDOFs)]:
            yield s
            # self.robot.SetDOFValues()
