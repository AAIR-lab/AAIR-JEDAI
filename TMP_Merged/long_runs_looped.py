from stitch_runs import StitchedRuns
import pickle
import subprocess
import os
import time
import multiprocessing
import random

def run_cmd():
    n = 3
    StitchedRuns.stitch_runs_sequential(n, 'test_domains/KevaLooped/')

times = []
seeds = []
success = []


for ii in range(15):
    os.system("rm -rf /home/local/ASUAD/npshah4/TMP_Merged/test_domains/KevaLooped/run_count.txt")
    time.sleep(2)
    print "Starting run no {}".format(ii)
    generated_seed = int(time.time())
    print "seed: ",generated_seed
    random.seed(generated_seed)
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            if time.time() - start_time > 2400:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
        times.append(time.time() - start_time)
    pickle.dump(times,open("ICRA_Results/Keva/keva_finalrun_keva_spiral_tower_times.p", "wb"))
    pickle.dump(seeds,open("ICRA_Results/Keva/keva_finalrun_keva_spiral_tower_seeds.p", "wb"))
    pickle.dump(success,open("ICRA_Results/Keva/keva_finalrun_keva_spiral_tower_success.p","wb"))

