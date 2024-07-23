import time

class ArgExecutor(object):
    def __init__(self,argument_name):
        self.argumnet_name = argument_name


    def execute(self,low_level_state,value,other_generated_values):
        raise NotImplementedError

    def apply(self,low_level_state,value,other_generated_values):
        raise NotImplementedError

    def wait_for_controller(self, robot, low_level_state,
                            sleep_interval_in_ms=1):

        simulator = low_level_state.simulator
        controller = robot.GetController()

        ll_states_captured = []
        while not controller.IsDone():
            
            ll_state_values = low_level_state.get_values_from_env(simulator.env)
            time.sleep(sleep_interval_in_ms / 1000.0)
            ll_states_captured.append(ll_state_values)

        return ll_states_captured