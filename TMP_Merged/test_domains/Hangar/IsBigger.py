from src.DataStructures.Predicate import Predicate
import copy


class IsBigger(Predicate):
    def __init__(self, name, arg_list=None):
        super(IsBigger, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return IsBigger(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self, **kwargs):
        print("****CALLING IS BIGGER*******")
        ll_state = kwargs['low_level_state']
        arg_map = kwargs['arg_map']

        fail_strings = []
        loc_type = 'box'
        if "loc" in arg_map["object3"]:
            loc_type = 'loc'

        ret = True
        print("DEEE: ",arg_map["object1"],arg_map["object3"])
        if loc_type == 'box':
            length_box, breadth_box, height_box = ll_state.simulator.get_obj_name_box_extents(object_name=arg_map["object1"])
            length_loc, breadth_loc, height_loc = ll_state.simulator.get_obj_name_box_extents(object_name=arg_map["object3"])
            if length_box > length_loc:
                ret = False
                fail_strings.append("(bigger {} {})".format(arg_map["object1"], arg_map["object3"]))
                if kwargs['arg_map']["current_hl_state"] is not None:
                    next_ll_state_predicates = kwargs['arg_map']["current_hl_state"].getTrueProps()
                    for prop in next_ll_state_predicates:
                        if "bigger {}".format(arg_map["object3"]) in prop:
                            smaller_box = prop[1:-1].split(' ')[-1]
                            apstr = "(bigger {} {})".format(arg_map["object1"], smaller_box)
                            if apstr not in prop:
                                fail_strings.append("(bigger {} {})".format(arg_map["object1"], smaller_box))


        print("DEEE: ",ret, "  ", fail_strings)
        return ret, fail_strings

    def get_failure_strings(self):
        pass
