import sys
from openravepy import *
env = Environment()
env_name = sys.argv[1]
env.Load(env_name)

for i in env.GetBodies():
    if 'yumi' not in str(i) and 'table' in str(i):
        body = i
        pose = body.GetTransform()
#         pose[0][3]+=0.0005
        pose[0][3]-=0.038
        body.SetTransform(pose)
table6 = env.GetKinBody('table6')
for i in env.GetBodies():
    if 'yumi' not in str(i) and i != table6:
        body = i
        pose = body.GetTransform()
        pose[2][3]+=0.082
        body.SetTransform(pose)
env.Save('../'+env_name)