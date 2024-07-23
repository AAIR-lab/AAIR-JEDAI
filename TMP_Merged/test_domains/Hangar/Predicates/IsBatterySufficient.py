from src.DataStructures.Predicate import Predicate
import copy

class IsBatterySufficient(Predicate):
    def __init__(self,name,arg_list = None):
        super(IsBatterySufficient,self).__init__(name,arg_list)

    def __deepcopy__(self, memodict={}):
        return IsBatterySufficient(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self,**kwargs):
        ll_state = kwargs["low_level_state"]
        arg_map = kwargs["arg_map"]
        if arg_map["endLoc"] == "recharge_station":
            return True,[]
        else:
            traj = arg_map["trajectory"]
            trajobj = RaveCreateTrajectory(ll_state.simulator.env, '')
            Trajectory.deserialize(trajobj, traj)
            length = trajobj.GetNumWaypoints()
            print "Old Battery:", ll_state.ll_variables["battery"]
            new_battery = ll_state.ll_variables["battery"] - length
            print "New Battery:", new_battery
            if new_battery < 30:
                return False, ["(not (hasBattery {} {}))".format(arg_map["agent"][0], arg_map["trajectory"][0])]
            else:
                ll_state.ll_variables["battery"] = new_battery
                return True, []
        # else:
        #     print "why this..!!"
        #     ll_state.battery = 1500
        #     return True,[]
