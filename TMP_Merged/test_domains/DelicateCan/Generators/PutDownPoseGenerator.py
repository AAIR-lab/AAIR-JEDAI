import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenRaveUtils
import numpy as np
import math


class PutDownPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['obj']
        # required_values = ['table', 'object']


        super(PutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.generate_function_state = self.generate_function()
        self.table_name = 'table6'
        self.object_name = known_argument_values.get('obj')

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt




    def get_next(self,flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):
        generated_values_list = []

        table_transform = self.simulator.get_transform_matrix(self.table_name)
        next_hl_state_true_prop = self.known_argument_values["current_hl_state"].getTrueProps()

        flag = False
        for prop in next_hl_state_true_prop:
            if "crushed" in prop and self.object_name in prop:
                flag = True

        if flag:
            dustbin_pose = self.simulator.env.GetKinBody("dustbin").GetTransform()
            t1 =  self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
            r1 = self.simulator.get_matrix_from_axis_angle(math.pi/2.0, 0, 0)
            t2 = self.simulator.get_matrix_from_pose(1,0,0,0,0,0,0.4)
            t = t1.dot(t2).dot(r1)
            pdp = dustbin_pose.dot(t)


        o_x, o_y, o_z = self.simulator.get_obj_name_box_extents(object_name=self.object_name)
        cylinder_radius = o_x
        base_width, base_length, cylinder_height = self.simulator.get_object_link_dimensions(object_name=self.object_name, link_name='base')

        # geom = self.simulator.get_object_link_geometries(self.table_name, link_name='base')
        # table_top = geom[0]
        x,y,z= self.simulator.get_obj_name_box_extents(self.table_name)
        # x, y, z = box_extents_xyz.tolist()
        x_limit = x - 2 * cylinder_radius
        y_limit = y - 2 * cylinder_radius

        i = 0
        j = 0
        while i < 3 and j < 50:
            obj_x = random.uniform(-x_limit, x_limit)
            obj_y = random.uniform(-y_limit, 0)
            obj_z = z + cylinder_height + 0.001

            obj_transform_wrt_origin = self.simulator.get_matrix_from_pose(1, 0, 0, 0, obj_x, obj_y, obj_z)
            obj_transform_wrt_table = obj_transform_wrt_origin.dot(table_transform)
            # obj_transform_wrt_robot = obj_transform_wrt_table.dot(self.simulator.env.GetRobots()[0].Getform opTransform())
            #obj_transform_wrt_table = obj_transform_wrt_origin

            # cylinder_hight_offset = np.eye(4)
            # cylinder_hight_offset[2,3] += cylinder_height
            # robot = self.simulator.env.GetRobot('fetch')
            # ot = robot.GetGrabbed()[0].GetTransform() + cylinder_hight_offset
            # wrt = robot.GetLink('wrist_roll_link').GetTransform()
            # diff = ot - wrt
            #
            # updated_pdp =  obj_transform_wrt_table - diff


            robot = self.simulator.env.GetRobot('fetch')
            wrt = robot.GetLink('wrist_roll_link').GetTransform()
            try:
                ot = robot.GetGrabbed()[0].GetTransform()
            except Exception,e:
                print "Something Wrong"
            ot[2,3] += cylinder_height
            pose_gripper_object = np.linalg.inv(ot).dot(wrt)

            updated_pdp = obj_transform_wrt_table.dot(pose_gripper_object)



            # obj_transform_wrt_table_back = obj_transform_wrt_table.dot(back)
            # gripper_transform = self.simulator.env.GetRobot('fetch').GetLink('gripper_link').GetTransform()
            # final_transform = gripper_transform.dot(np.linalg.inv(obj_transform_wrt_table))
            if self.simulator.robots[self.robot_name].get_ik_solutions(updated_pdp,True):

                generated_values_list.append(updated_pdp)
                i+=1
                # self.simulator.visualize_transform(obj_transform_wrt_table_back)
                # import IPython
                # IPython.embed()
            j += 1

        random.shuffle(generated_values_list)
        return generated_values_list



def generate_pre_grasp_pose(openrave_ll_state, object_name, grasp_pose):
    env, robot = openrave_ll_state.get_env_and_robot()
    with env:
        obj = env.GetKinBody(object_name)
        orig_t = obj.GetTransform()
        gp_at_origin = grasp_pose.dot(np.linalg.inv(orig_t))

        approach_dist = 0.03
        t4 = matrixFromPose((1, 0, 0, 0, -approach_dist, 0, 0))
        pre_grasp_pose = gp_at_origin.dot(t4)
        pre_grasp_pose_wrt_obj = pre_grasp_pose.dot(orig_t)
    return [pre_grasp_pose_wrt_obj]
