import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *


class ObjectsOnTableGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['table']

        super(ObjectsOnTableGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.table_name = known_argument_values.get('table_name')
        self.generate_function_state = self.generate_function()
        self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            yield None

    def get_next(self,flag):
        return self.generate_function_state.next()






