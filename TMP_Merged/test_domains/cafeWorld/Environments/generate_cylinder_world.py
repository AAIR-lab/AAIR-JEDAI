import sys

# from src.Robots.Models import FetchOpenRaveRobotModel
# from misc.RobotModels.fetch import FetchRobot


import random
import openravepy
import numpy as np
import utils
import time
import getopt
import object_models
import sys

TARGET_OBJECT = "object1"
OBJECT_PREFIX = "object"
TABLE_NAMES = {"rll": "rll_table", "small": "table6"}


def execute(num_objects, cylinder_dims=(0.05, 0.2), max_radial_distance=0.73, robot_dist_from_table=0.15,
            seed=int(time.time()), envFile=None, tabletype='rll', viewer=False):
    # Spawns PR2, table, and num_objects cylinders.
    # cylinder_dims: (radius, height)
    # max_radial_distance: x, y distance from PR2 which you want cylinders to be within
    # robot_dist_from_table: distance between front edge of PR2 and short edge of table (this is approximated)

    assert max_radial_distance > robot_dist_from_table, "Robot too far from table"
    env = openravepy.Environment()
    if viewer:
        env.SetViewer('qtcoin')

    spawn_table_pr2(env, robot_dist_from_table, tabletype)
    rand = np.random.RandomState(seed=seed)
    num_spawned = generate_world(env, TABLE_NAMES[tabletype], num_objects, cylinder_dims, max_radial_distance, rand)
    on_table(env, TABLE_NAMES[tabletype])

    dustbin = env.ReadKinBodyXMLFile("model.dae")
    dustbin.SetName("dustbin")
    t = dustbin.GetTransform()
    t[1,3] = 0.6
    env.Add(dustbin)
    dustbin.SetTransform(t)

    if viewer:
        raw_input("press enter to quit")
        pass

    env.Save(envFile)
    with open("seed.txt", 'w+') as f:
        f.write(str(seed))
    print("Environment %s created; seed for %d objects attempted (%d actual): %d" % (
    envFile, num_objects, num_spawned, seed))
    return env


def generate_world(env, table_name, num_objects, cylinder_dims, max_radial_distance, rand):
    collision_checker = openravepy.RaveCreateCollisionChecker(env, "fcl_")
    collision_checker.SetCollisionOptions(openravepy.CollisionOptions.Contacts)
    env.SetCollisionChecker(collision_checker)
    table = env.GetKinBody(table_name)
    count = 0

    for obj_num in range(num_objects):
        obj_name = "object" + repr(obj_num)
        if obj_name == TARGET_OBJECT:
            color = [0.2, 0.2, 0.2]
        else:
            color = [0, 0.8, 0.8]
        create_collision_free_random_cylinder(env, table, cylinder_dims, obj_name, max_radial_distance, rand,
                                              color=color)
        body = env.GetKinBody(obj_name)
        if body is not None:
            print("Object %s created on surface %s" % (body.GetName(), table_name))
            count += 1
        else:
            print("Could not generate collision free cylinder for %s" % obj_name)

    return count


def create_collision_free_random_cylinder(env, table, dimensions, obj_name, max_radial_distance, rand, color,
                                          num_trials=50):
    radius, height = dimensions

    for i in xrange(num_trials):
        cylinder = create_cylinder(env, table, radius, height, obj_name, max_radial_distance, rand, color)
        collision = False
        for body in env.GetBodies():
            if body != table and env.CheckCollision(body, cylinder):
                collision = True
                break
        if not collision:
            return
        env.Remove(cylinder)


def create_cylinder(env, table, radius, height, body_name, max_radial_distance, rand, color):
    DIFF = 0.02
    min_x, max_x, min_y, max_y, table_height = utils.get_object_limits(table)
    x = rand.uniform(min_x + DIFF + radius, max_x - DIFF - radius)
    y = rand.uniform(min_y + DIFF + radius, max_y - DIFF - radius)
    # while (x - robot_pos[0])**2 + (y - robot_pos[1])**2 > max_radial_distance:
    #     x = rand.uniform(min_x + radius, max_x - radius)
    #     y = rand.uniform(min_y + radius, max_y - radius)
    z = table_height + height / 2

    t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
    cylinder = object_models.create_cylinder(env, body_name, t, [radius, height], color)
    env.Add(cylinder, False)

    return cylinder


def spawn_table_pr2(env, robot_dist_from_table, tabletype='rll'):
    if tabletype == 'rll':
        thickness = 0.2
        legheight = 0.4
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 2.235, 0.94, thickness, 1.3, 0.6, legheight)
    elif tabletype == 'small':
        thickness = 0.2
        legheight = 0.4
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 0.90, 0.90, thickness,
                                           0.2 - robot_dist_from_table, 0.2, legheight)
        # old dims : 0.90 small table dims : 0.30
    x = -utils.get_object_limits(table)[0] + 0.35 + robot_dist_from_table
    y = 0
    z = thickness / 2 + legheight
    table.SetTransform(openravepy.matrixFromPose([1, 0, 0, 0, x, y, z]))
    env.AddKinBody(table)
    # FetchOpenRaveRobotModel(env, False)

    # robot = env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
    # env.Add(robot)

def set_cylinders(env, rand, name, color):
    cylinder_dims=(0.05, 0.2)
    radius, height = cylinder_dims
    DIFF = 0.05
    x = rand.uniform(3.5 + DIFF + radius, 3.7 - DIFF - radius)
    y = rand.uniform(16 + DIFF + radius, 17 - DIFF - radius)
    z = 1.15

    t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])

    cylinder = object_models.create_cylinder(env, "can_"+str(name), t, [radius, height], color)
    env.Add(cylinder)

def set_tables(env, rand):
    thickness = 0.1
    legheight = 0.55
    pose = [4, 5, 0.63]
    tables = [[4, 8, 0.63], [6, 8, 0.63], [4, 10, 0.63], [6, 10, 0.63]]
    colors = [(230, 25, 75),(60, 180, 75),(70, 240, 240),(170, 110, 40)]
    color_names = ['red','green','blue','brown']
    cylinders = []
    for i, pose in enumerate(tables):
        color = [x/255.0 for x in colors[i]]
        table = object_models.create_table(env,
                                           'table_'+color_names[i],
                                           0.90,
                                           0.90,
                                           thickness,
                                           0.1,
                                           0.1,
                                           legheight,
                                           pose,
                                           color)
        env.AddKinBody(table)
        set_cylinders(env, rand, color_names[i], color)

def on_table(env, table_name):
    table = env.GetKinBody(table_name)
    if table:
        print("Transforming objects onto table")
        for obj in env.GetBodies():
            if obj.GetName().startswith(OBJECT_PREFIX):
                object_models.on_table(obj, table)


def create_env(num_objects, cylinder_dims, seed=int(time.time()), envFile=None, tabletype='small', viewer=False):
    if envFile is None:
        dir = './'
        envFile = dir + 'can_world_' + str(num_objects) + "_cans.dae"
    execute(num_objects, cylinder_dims, seed=seed, envFile=envFile, tabletype=tabletype, viewer=viewer)



if __name__ == "__main__":
    # try:
    #     seed = int(sys.argv[2])
    #     cans = int(sys.argv[1])
    # except:
    #     seed = int(time.time())
    #     cans = 25
    # create_env(cans, (0.03, 0.2), seed = seed, viewer=False)
    # seed=1548030364
    rand = np.random.RandomState(seed=13)
    env = openravepy.Environment()
    env.Load('./world_final.stl')
    env.SetViewer('qtcoin')
    set_tables(env, rand)
    print(env.GetBodies())
    envFile = './4_orders.dae'
    env.Save(envFile)
    while True:
        pass