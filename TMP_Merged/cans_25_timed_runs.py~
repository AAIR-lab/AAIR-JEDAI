from TMP import TMP
import pickle
from EnvGenerators import generate_cylinder_world
import Config
import subprocess
import os
import time
import multiprocessing

def run_cmd():
    TMP()

Config.SHOW_VIEWER = True
times = []
seeds = []
success = []
Config.RUN_TRAJ = False
Config.PLOT = True
Config.NUM_CANS = 25
Config.RESULTS_FILE = "results_"+str(Config.NUM_CANS)+"_10.csv"
Config.DEFAULT_PDDL_FILE = Config.PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+str(Config.NUM_CANS)+'_cans_10_mdp.pddl'
Config.DEFAULT_PROBLEM_FILE = Config.PROJ_DIR+"SampleTasks/can_world_delicate_cans_"+str(Config.NUM_CANS)+'_cans_problem.pddl'
Config.PORTNO = 1234
Config.POLICY_OUTPUT_FILE = Config.PROJ_DIR + "graph2.gv"
Config.COMBINED_FILE = Config.PROJ_DIR + "combined_file2.pddl"
Config.OPENRAVE_ENV_XML = Config.PROJ_DIR+'GeneratedEnvironments/can_world_'+str(Config.NUM_CANS)+'_cans.dae'

for ii in range(30):
    # print "Starting Run No : {}".format(ii)
    generated_seed = int(time.time())
    print "Seed : ",generated_seed
    seeds.append(generated_seed)
    # generate_cylinder_world.create_env(Config.NUM_CANS,(0.03, 0.2),seed=generated_seed)
    # p1 = subprocess.Popen(["python","TMP.py"])
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
        print "PID: ",p1.pid
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            # print time.time()
            if time.time() - start_time > 24000:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
            times.append(time.time() - start_time)
    pickle.dump(times,open("ICRA_Results/Delicate_cans/canworld_finalrun_times_"+str(Config.NUM_CANS)+"_cans_10.p","wb"))
    pickle.dump(seeds,open("ICRA_Results/Delicate_cans/canworld_finalrun_seeds_"+str(Config.NUM_CANS)+"_cans_10.p","wb"))
    pickle.dump(success,open("ICRA_Results/Delicate_cans/canworld_finalrun_success_"+str(Config.NUM_CANS)+"_cans_10.p","wb"))


times = []
seeds = []
success = []
Config.RESULTS_FILE = "results_"+str(Config.NUM_CANS)+"_50.csv"
Config.DEFAULT_PDDL_FILE = Config.PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+str(Config.NUM_CANS)+'_cans_50_mdp.pddl'


for ii in range(30):
    print "Starting Run No : {}".format(ii)
    # generated_seed = int(time.time())
    # print "Seed : ",generated_seed
    # seeds.append(generated_seed)
    # generate_cylinder_world.create_env(Config.NUM_CANS,(0.03, 0.2),seed=generated_seed)
    # p1 = subprocess.Popen(["python","TMP.py"])
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            # print time.time()
            if time.time() - start_time > 24000:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
            times.append(time.time() - start_time)
    pickle.dump(times,open("ICRA_Results/Delicate_cans/canworld_finalrun_times_"+str(Config.NUM_CANS)+"_cans_50.p","wb"))
    pickle.dump(seeds,open("ICRA_Results/Delicate_cans/canworld_finalrun_seeds_"+str(Config.NUM_CANS)+"_cans_50.p","wb"))
    pickle.dump(success,open("ICRA_Results/Delicate_cans/canworld_finalrun_success_"+str(Config.NUM_CANS)+"_cans_50.p","wb"))

times = []
seeds = []
success = []
Config.RESULTS_FILE = "results_"+str(Config.NUM_CANS)+"_90.csv"
Config.DEFAULT_PDDL_FILE = Config.PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+str(Config.NUM_CANS)+'_cans_90_mdp.pddl'


for ii in range(30):
    print "Starting Run No : {}".format(ii)
    # generated_seed = int(time.time())
    # print "Seed : ",generated_seed
    # seeds.append(generated_seed)
    # generate_cylinder_world.create_env(Config.NUM_CANS,(0.03, 0.2),seed=generated_seed)
    # p1 = subprocess.Popen(["python","TMP.py"])
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            # print time.time()
            if time.time() - start_time > 24000:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
            times.append(time.time() - start_time)
    pickle.dump(times,open("ICRA_Results/Delicate_cans/canworld_finalrun_times_"+str(Config.NUM_CANS)+"_cans_90.p","wb"))
    pickle.dump(seeds,open("ICRA_Results/Delicate_cans/canworld_finalrun_seeds_"+str(Config.NUM_CANS)+"_cans_90.p","wb"))
    pickle.dump(success,open("ICRA_Results/Delicate_cans/canworld_finalrun_success_"+str(Config.NUM_CANS)+"_cans_90.p","wb"))
