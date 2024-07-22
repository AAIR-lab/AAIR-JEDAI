from ScanWorld import ScanWorld
from openravepy import Environment
env = Environment()
env.Load('/usr/local/share/openrave-0.9/data/lab1.env.xml')



ScanWorld.instance(env)

print ScanWorld.instance().get_object_name_transform_map()

while True:
    pass
