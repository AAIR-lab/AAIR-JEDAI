from openravepy import *
import env_build_helper

env = Environment()

env2 = Environment()

env.Load("NEW/keva_tilted_pillar_triple_pi_structure.dae")

for body in env.GetBodies():
	env_build_helper.create_plank(env2,body.GetName(),body.GetTransform())
env2.Save("new_pic/keva_tilted_pillar_triple_pi_structure.dae")