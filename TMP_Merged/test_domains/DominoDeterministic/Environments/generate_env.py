from openravepy import *
import Config
import math

def gen_env(number_of_dominos):
    env = Environment()
    env.Load(Config.DOMAIN_DIR + "/Environments/domino_table.dae")
    domino_1 = env.GetKinBody("domino1")
    for i in range(2,number_of_dominos+1):
        temp_domino = env.ReadKinBodyURI(Config.DOMAIN_DIR + "/Environments/model.dae")
        t = domino_1.GetTransform()
        if i % 2 == 0:
            t[1,3] += int(i / 2) * 0.04
        else:
            t[1,3] -= int(i / 2) * 0.04
        temp_domino.SetTransform(t)
        temp_domino.SetName("domino"+str(i))
        env.Add(temp_domino)
    env.Save("domino_env_"+str(number_of_dominos)+".dae")

if __name__ == "__main__":
    number_of_dominos = 15
    gen_env(number_of_dominos)
