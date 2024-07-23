import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenRaveUtils
import numpy as np
import math


class PutDownPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['location', 'obj']
        # required_values = ['table', 'object']


        super(PutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.generate_function_state = self.generate_function()
        self.table_name = known_argument_values.get('location')
        self.object_name = known_argument_values.get('obj')

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self,flag):
        return self.generate_function_state.next()
    
    def _compute_put_down_pose_for_counter(self):
        
        
        # HACK: We cannot put stuff back on the counter!
        # The hack here is to manually just place them back
        # in their original positions in the environment.
        
        assert self.table_name == "counter"
        
        # Get the transforms of the cans in their
        # initial position in the environment.
        #
        # This is manually done.
        if self.object_name == "canred":
            
            transform = np.array(
                [[ 1.        ,  0.        ,  0.        ,  1.6       ],
                 [ 0.        ,  1.        ,  0.        ,  0.29003298],
                 [ 0.        ,  0.        ,  1.        ,  1.15      ],
                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
            
        elif self.object_name == "cangreen":
            
            transform = np.array(
                [[ 1.        ,  0.        ,  0.        ,  1.6       ],
                [ 0.        ,  1.        ,  0.        ,  0.87259936],
                [ 0.        ,  0.        ,  1.        ,  1.15      ],
                [ 0.        ,  0.        ,  0.        ,  1.        ]])
        elif self.object_name == "canblue":
            
            transform = np.array(
                [[ 1.       ,  0.       ,  0.       ,  1.6      ],
                [ 0.       ,  1.       ,  0.       ,  0.4627594],
                [ 0.       ,  0.       ,  1.       ,  1.15     ],
                [ 0.       ,  0.       ,  0.       ,  1.       ]])
        elif self.object_name == "canbrown":
            
            transform = np.array(
                [[ 1.        ,  0.        ,  0.        ,  1.6       ],
                [ 0.        ,  1.        ,  0.        ,  0.72042121],
                [ 0.        ,  0.        ,  1.        ,  1.15      ],
                [ 0.        ,  0.        ,  0.        ,  1.        ]])
        else:
            
            transform = None
            raise Exception("Incorrect can!")
    
        generated_values_list = []
    
        # Most of the code here has been taken from
        # the grasp pose generator. Since the initial
        # grasp was always from the counter, this is
        # simply that code to get the transform for
        # the counter.
        height_offset = -0.1    
        cylinder_height = 0.2
        dist_offset = 0.04
        
        robot = self.simulator.env.GetRobots()[0]
        wrist_roll_pose = robot.GetLink('wrist_roll_link').GetTransform()
        gripper_pose = robot.GetLink('gripper_link').GetTransform()
        wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)
        
        num_grasps = 4
        for i in range(num_grasps):
    
            rot_ang = (i * (2 * np.pi) / num_grasps)
            t2 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, -dist_offset, 0, cylinder_height + height_offset)
    
            r2 = self.simulator.get_matrix_from_axis_angle(0, 0, rot_ang)
            t3 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
            grasp_pose = t3.dot(t2).dot(r2)
            grasp_pose = np.matmul(transform,grasp_pose)
            grasp_pose = np.matmul(grasp_pose, wrist_pose_wrt_gripper)
            
            if len(self.simulator.robots[self.robot_name].get_ik_solutions(grasp_pose, True)) > 0:
                generated_values_list.append(grasp_pose)
            
        return generated_values_list

    def _compute_put_down_pose_list_for_colored_tables(self):
        
        generated_values_list = []

        table_transform = self.simulator.get_transform_matrix(self.table_name)
        next_hl_state_true_prop = self.known_argument_values["current_hl_state"].getTrueProps()


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

        while i < 5 and j < 50:
            obj_x = random.uniform(-x_limit, x_limit)
            obj_y = random.uniform(-y_limit, y_limit)
            obj_z = z + cylinder_height/2 + 0.001
            # obj_z = z+0.001

            obj_transform_wrt_origin = self.simulator.get_matrix_from_pose(1, 0, 0, 0, obj_x, obj_y, obj_z)
            obj_transform_wrt_table = obj_transform_wrt_origin.dot(table_transform)


            robot = self.simulator.env.GetRobot('fetch')
            wrist_roll_pose = robot.GetLink('wrist_roll_link').GetTransform()
            gripper_pose = robot.GetLink('gripper_link').GetTransform()
            wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)
            try:
                ot = robot.GetGrabbed()[0].GetTransform()
            except Exception,e:
                print "Something Wrong"

            ot[2,3] += cylinder_height
            pose_gripper_object = np.linalg.inv(ot).dot(wrist_roll_pose)

            # updated_pdp = obj_transform_wrt_table.dot(pose_gripper_object)
            updated_pdp = obj_transform_wrt_table.dot(wrist_pose_wrt_gripper)

            if len(self.simulator.robots[self.robot_name].get_ik_solutions(updated_pdp, True)) > 0:

                generated_values_list.append(updated_pdp)
                i+=1
                # self.simulator.visualize_transform(obj_transform_wrt_table_back)
                # import IPython
                # IPython.embed()
            j += 1
        

        random.shuffle(generated_values_list)
        return generated_values_list

    def _compute_put_down_pose_list(self):
        
        if self.table_name == "counter":
            
            return self._compute_put_down_pose_for_counter()
        else:
            return self._compute_put_down_pose_list_for_colored_tables()

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
