from src.DataStructures.Generator import Generator
import Config
import numpy as np

class TargetPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["endLoc"]
        super(TargetPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=required_values)
        self.target_location = known_argument_values["endLoc"]
        self.generate_move_target_state = self.generate_move_target_function()
        self.generate_inspect_target_state = self.generate_inspect_target_function()
        self.known_args = known_argument_values
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR


    def reset(self):
        self.generata_move_target_state = self.generate_move_target_function()
        self.generate_inspect_target_state = self.generate_inspect_target_function()

    def generate_inspect_target_function(self):
        # while True:
        #     if self.target_location == "left_wing":
        #         t =  [7.09084182e-01,-4.37932002e-03,4.30201002e-03,-7.05097112e-01,7.20000000e+00,1.00000000e+00,2.50000000e+00]
        #     elif self.target_location == "right_wing":
        #         t = [7.09084182e-01, -4.37932002e-03, 4.30201002e-03, -7.05097112e-01, 9.10000000e+00, 1.00000000e+00,2.50000000e+00]
        #     yield t
        i = 0
        while True:
            if i > 5:
                raise StopIteration
            env = self.simulator.env
            if self.target_location == "left_wing":
                left_wing = env.GetKinBody('airplane_left_wing')
                mesh = env.Triangulate(left_wing)
                mesh_points = mesh.vertices
                selected_indices = np.random.choice(range(mesh_points.shape[0]),50)
                selected_points = mesh_points[selected_indices,:]
                sorted_points = np.array(sorted(selected_points.tolist()))
                z = env.GetRobot(Config.ROBOT_NAME).GetLink('base_link').GetTransform()[2,3]
                sorted_points[:,2] = z
            elif self.target_location == "right_wing":
                right_wing = env.GetKinBody('airplane_right_wing')
                mesh = env.Triangulate(right_wing)
                mesh_points = mesh.vertices
                selected_indices = np.random.choice(range(mesh_points.shape[0]), 50)
                selected_points = mesh_points[selected_indices, :]
                sorted_points = np.array(sorted(selected_points.tolist(),reverse = True))
                z = env.GetRobot(Config.ROBOT_NAME).GetLink('base_link').GetTransform()[2,3]
                sorted_points[:, 2] = z
            i += 1
            yield sorted_points

    def generate_move_target_function(self):
        # while True:
        #     if self.target_location == "left_wing":
        #         t = [3.7,-0.67,2.5]
        #     elif self.target_location == "right_wing":
        #         t = [ 13.2, -0.67, 2.5]
        #     elif self.target_location == "rechage_station":
        #         t = [3.70000005e+00,1.00000000e+01,9.00000000e-01]
        #     yield t
        i = 0
        while True:
            if i > 5:
                raise StopIteration
            env = self.simulator.env
            if self.target_location == "left_wing":
                left_wing = env.GetKinBody('airplane_left_wing')
                geom = left_wing.GetLinks()[0].GetGeometries()[0]
                aabb = geom.ComputeAABB(left_wing.GetTransform())
                target_pose = aabb.pos()
                target_pose[0] -= aabb.extents()[0]
                target_pose[1] -= aabb.extents()[1]
                target_pose[2] += aabb.extents()[2] + 1
            elif self.target_location == "right_wing":
                right_wing = env.GetKinBody('airplane_right_wing')
                geom = right_wing.GetLinks()[0].GetGeometries()[0]
                aabb = geom.ComputeAABB(right_wing.GetTransform())
                target_pose = aabb.pos()
                target_pose[0] += aabb.extents()[0]
                target_pose[1] -= aabb.extents()[1]
                target_pose[2] += aabb.extents()[2] + 1
            elif self.target_location == "recharge_station":
                target_pose = np.array([13.5,6,1.7])
            i+=1
            yield target_pose


    def get_next(self,extras):
        arg_type = extras["type"]
        if arg_type == "MoveTarget":
            return self.generate_move_target_state.next()
        elif arg_type == "InspectTarget":
            return self.generate_inspect_target_state.next()
        # return self.generate_function_state.next()
