from src.DataStructures.Predicate import Predicate
import copy
from openravepy import *

class NotPutdownObstructs(Predicate):
    def __init__(self, name, arg_list=None):
        super(NotPutdownObstructs, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return NotPutdownObstructs(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self,**kwargs):
        print "****CALLING NOT OBSTRUCTS*******"
        ll_state = kwargs['low_level_state']
        arg_map = kwargs["arg_map"]
        colliding_object_names = set()
        traj = arg_map["t"]
        trajobj = RaveCreateTrajectory(ll_state.simulator.env,'')
        Trajectory.deserialize(trajobj,traj)

        waypoints_count = trajobj.GetNumWaypoints()
        fail_strings = []


        for i in range(waypoints_count):
            resulting_env = ll_state.simulator.set_robot_active_dof_values_to_waypoint_values(ll_state, trajobj.GetWaypoint(i))
            # cr = ll_state.simulator.get_collision_report(resulting_env)

            ####
            colliding_objects = ll_state.simulator.get_colliding_objects(resulting_env)
            for o in colliding_objects:
                if o != arg_map["obj"] and o != "table6":
                    colliding_object_names.add(o)
            ###
        if len(colliding_object_names) > 0:
            fail_strings.append("(putdownobstructs {} {})".format(arg_map["trajectory"],arg_map["obj"]))

        if len(colliding_object_names) > 0:
            pass
        else:
            # import IPython
            # IPython.embed()
            pass

        return len(colliding_object_names) == 0, fail_strings

    def get_failure_strings(self):
        pass