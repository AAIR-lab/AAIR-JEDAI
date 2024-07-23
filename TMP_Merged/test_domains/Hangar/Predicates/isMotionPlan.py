from src.DataStructures.Predicate import Predicate
import copy


class isMotionPlan(Predicate):
    def __init__(self, name, arg_list=None):
        super(isMotionPlan, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return isMotionPlan(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self, pose_current, pose_end, trajectory_arg, **kwargs):
        return True, []
        # ll_state = kwargs['low_level_state']
        # colliding_object_names = set()
        # trajobj = trajectory_arg.value
        #
        # waypoints_count = trajobj.GetNumWaypoints()
        # fail_strings = []
        #
        #
        # for i in range(waypoints_count):
        #     resulting_env = ll_state.simulator.set_robot_active_dof_values_to_waypoint_values(ll_state, trajobj.GetWaypoint(i))
        #     cr = ll_state.simulator.get_collision_report(resulting_env)
        #     if len(cr.contacts) == 0:
        #         continue
        #     else:
        #         print "Collision"
        #
        #     colliding_links = [cr.plink1, cr.plink2]
        #
        #     for link in colliding_links:
        #         if link is None:
        #             continue
        #
        #         name = ll_state.simulator.get_body_name_associated_with_link(link)
        #
        #         if name != 'fetch':
        #             colliding_object_names.add(name)
        #
        # for name in colliding_object_names:
        #     # (obstructs arm_gp_object0 object1 object0)
        #
        #     fail_strings.append("(obstructs )")
        # return len(colliding_object_names) == 0, fail_strings

    def get_failure_strings(self):
        pass