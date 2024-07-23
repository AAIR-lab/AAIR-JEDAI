from src.DataStructures.Predicate import Predicate
from src.IKSolvers.OpenRaveIKSolver import OpenRaveIKSolver

non_movable = []
class IsObstructionFree(Predicate):
    def __init__(self, name, arg_obj_list=None):
        super(IsObstructionFree, self).__init__(name, arg_obj_list)
        self.objects_in_collision = None
        self.object_being_picked = None
        self.obstructing_objects = set()

    def evaluate(self, ll_state, planner, arg_value_map, hl_args):

        # has_ik, iksols = planner.get_ik_solutions(arg_value_map[self.arg_list[0].name], ll_state)
        iksols = OpenRaveIKSolver(ll_state).get_ik_solutions(arg_value_map[self.arg_list[0].name], check_collisions=False)

        hl_args.extend(['table6'])
        # global non_movable
        # non_movable.extend(hl_args)
        self.objects_in_collision = planner.motion_planner(ll_state).get_objects_in_collision(iksols, list_non_movable_object_names=hl_args)
        # self.objects_in_collision = planner.get_objects_in_mp_trajectory(iksols, list_non_movable_object_names=hl_args, state=ll_state)#TODO list_non_movable_hack
        self.object_being_picked = hl_args[0]

        self.obstructing_objects = set()
        for o in self.objects_in_collision:
            if o not in hl_args:
                self.obstructing_objects.add(o)

        # else:
        #     assert False, "Can't do anything about immovable objects in collision:"+str(self.objects_in_collision)

        if self.objects_in_collision:
            return False
        else:
            return True

    def get_proposition_strings(self):
        # (obstructs arm_gp_object1 object18 object1)
        temp_map = {'pgp':'gp', 'pdp':'gp'}
        propositions = []
        if self.arg_list[0].name in temp_map:
            pose_arg = temp_map.get(self.arg_list[0].name)
        else:
            pose_arg = self.arg_list[0].name
        for o in self.obstructing_objects:
            prep =  "(" + "obstructs" + " " + "arm_" + pose_arg + "_"+ self.object_being_picked + " " + o + " " + self.object_being_picked + ")"
            propositions.append(prep)
        return propositions

