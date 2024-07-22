import sys
from openravepy import *
env = Environment()
env_name = sys.argv[1]
env.Load(env_name)
for i in env.GetBodies():
    t = i.GetTransform()
    t[0,3] -= 0.29777083
    t[1,3] -= -0.00088151
    t[2,3] -= 0.60100429
    i.SetTransform(t)
env.Save('../'+env_name)
