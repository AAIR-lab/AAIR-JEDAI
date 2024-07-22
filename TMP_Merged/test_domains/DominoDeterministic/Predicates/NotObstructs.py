from src.DataStructures.Predicate import Predicate
import copy
from openravepy import *


class NotObstructs(Predicate):
    def __init__(self, name, arg_list=None):
        super(NotObstructs, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return NotObstructs(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self, **kwargs):
        ll_state = kwargs['low_level_state']
        arg_map = kwargs["arg_map"]
        flag = kwargs["flag"]
        if flag:
            return True, []
        traj = arg_map["obj"]
        trajobj = RaveCreateTrajectory(ll_state.simulator.env,'')
        Trajectory.deserialize(trajobj,traj)
        arg_map = kwargs['arg_map']

        colliding_object_names = list()
        waypoints_count = trajobj.GetNumWaypoints()
        fail_strings = []


        for i in range(waypoints_count):
            resulting_env = ll_state.simulator.set_robot_active_dof_values_to_waypoint_values(ll_state, trajobj.GetWaypoint(i))
            # cr = ll_state.simulator.get_collision_report(resulting_env)

            ####
            colliding_objects = ll_state.simulator.get_colliding_objects(resulting_env)
            for o in colliding_objects:
                if o != arg_map["obj"] and o != "table6":
                    if o not in colliding_object_names:
                        colliding_object_names.append(o)

        for name in colliding_object_names:
            # (obstructs arm_gp_object0 object1 object0)
            if name != arg_map["obj"]:
                print "Obstructed By : {}".format(name)
                fail_strings.append("(obstructs {} {} {})".format(arg_map["trajectory"],name,arg_map["obj"]))
            # for oname in prev_objects:
            #     fail_strings.append("(obstructs {} {} {})".format(arg_map["trajectory"][0],oname,name))
            # prev_objects.append(name)
        # fail_strings.append("(obstructs {} {} {})".format(arg_map["trajectory"][0], colliding_object_names[-1], arg_map["object"][0]))

        return len(colliding_object_names) == 0, fail_strings

    def get_failure_strings(self):
        pass