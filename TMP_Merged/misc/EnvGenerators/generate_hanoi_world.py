#! /usr/bin/python

import sys
sys.path.append('../src')
from Robots.Models import FetchOpenRaveRobotModel

import random
import openravepy
import numpy as np
import utils
import time
import getopt
import object_models
import os

OBJECT_PREFIX = "object"
TABLE_NAMES = {"rll":"rll_table", "small":"table6"}

def execute(num_objects, box_dims=(0.05,0.05,0.05), max_radial_distance=0.73, robot_dist_from_table=0.15, seed=int(time.time()), envFile='generated_env.dae', tabletype='rll', viewer=False):
    # Spawns PR2, table, and num_objects cylinders.
    # cylinder_dims: (radius, height)
    # max_radial_distance: x, y distance from PR2 which you want cylinders to be within
    # robot_dist_from_table: distance between front edge of PR2 and short edge of table (this is approximated)
        
    assert max_radial_distance > robot_dist_from_table, "Robot too far from table"
    env = openravepy.Environment()
    if viewer:
        env.SetViewer('qtcoin')

    spawn_table_pr2(env, robot_dist_from_table, tabletype)
    # rand = np.random.RandomState(seed=seed)
    num_spawned = generate_world(env, TABLE_NAMES[tabletype], num_objects, box_dims)
    # on_table(env, TABLE_NAMES[tabletype])

    env.Save(envFile)
    with open("seed.txt", 'w+') as f:
        f.write(str(seed))
    print("Environment %s created; seed for %d objects attempted (%d actual): %d" %(envFile, num_objects, num_spawned, seed))
    return env

def generate_world(env, table_name, num_objects, box_dims):
    # collision_checker = openravepy.RaveCreateCollisionChecker(env, "fcl_")
    # collision_checker.SetCollisionOptions(openravepy.CollisionOptions.Contacts)
    # env.SetCollisionChecker(collision_checker)
    table = env.GetKinBody(table_name)
    count = 0
    length, breadth, height = box_dims
    for obj_num in range(num_objects):
        obj_name = "box" + repr(num_objects-obj_num)
        if obj_num % 2  == 0:
            color = [0.2, 0.2, 0.2]
        else:
            color = [0, 0.8, 0.8]
        create_box(env, table, length - 0.003*obj_num, breadth- 0.003 * obj_num, height, obj_name, obj_num, color=color)
        body = env.GetKinBody(obj_name)
            
    return count

def create_box(env, table, length, breadth, height, body_name,obj_num, color):
    DIFF = 0.02
    min_x, max_x, min_y, max_y, table_height = utils.get_object_limits(table)

    x = min_x +0.2
    y = min_y + 0.2
    rand = np.random.RandomState(seed=obj_num)
    # x = rand.uniform(min_x+DIFF , max_x-DIFF )
    # y = rand.uniform(min_y+DIFF , max_y-DIFF)
    tmp = 0
    if obj_num > 0:
        tmp = 1/obj_num
    z = table_height + obj_num*(2*height+0.001) + height/2 + 0.001
    print(x,y,z)
    t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
    print(length, breadth , height,body_name)
    box = object_models.create_box(env, body_name, t, [length, breadth , height], color)
    env.Add(box, False)
    return box

def spawn_table_pr2(env, robot_dist_from_table, tabletype='rll'):
    if tabletype == 'rll':
        thickness = 0.2
        legheight = 0.4
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 2.235, 0.94, thickness, 1.3, 0.6, legheight)
    elif tabletype == 'small':
        thickness = 0.2
        legheight = 0.4
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 0.90, 0.90, thickness,0.2 - robot_dist_from_table, 0.2, legheight)
        #old dims : 0.90 small table dims : 0.30
    x = -utils.get_object_limits(table)[0] + 0.35 + robot_dist_from_table
    y = 0
    z = thickness / 2 + legheight
    table.SetTransform(openravepy.matrixFromPose([1, 0, 0, 0, x, y, z]))
    env.AddKinBody(table)
    # FR = FetchOpenRaveRobotModel(env, False)
    #
    # robot = FR.getRobot()
    # env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
    # env.Add(robot)

def on_table(env, table_name):
    table = env.GetKinBody(table_name)
    if table:
      print("Transforming objects onto table")
      for obj in env.GetBodies():
          if obj.GetName().startswith(OBJECT_PREFIX):
              object_models.on_table(obj, table)


def create_env(num_objects, cylinder_dims, seed=int(time.time()), envFile=None,tabletype='small', viewer=False):
    if envFile is None:
        dir = os.path.abspath(os.path.dirname(__file__))
        dir = "/".join(dir.split('/')[:-1]) + "/GeneratedEnvironments/"
        envFile = dir + 'hanoi_world_'+str(num_objects)+"_boxes.dae"
    execute(num_objects, cylinder_dims, seed=seed, envFile=envFile,tabletype=tabletype, viewer=viewer)
    if viewer:
        raw_input("press enter to quit")
        pass

if __name__ == "__main__":
    create_env(4,(0.035,0.035,0.02),viewer=True)

    #seed=1548030364

