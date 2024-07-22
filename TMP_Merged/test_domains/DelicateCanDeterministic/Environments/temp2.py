import os
import subprocess

l = os.listdir("./")
for name in l:
    if "can_world_" in name:
        p = subprocess.Popen(["python temp1.py {}".format(name)],shell=True)
        p.wait()