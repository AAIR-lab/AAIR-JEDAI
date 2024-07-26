from openravepy import *
import sys

f = sys.argv[1]

e = Environment()
e.Load(f)
dustbin = e.ReadKinBodyXMLFile("model.dae")
dustbin.SetName("dustbin")
t = dustbin.GetTransform()
t[1,3] = 0.6
e.Add(dustbin)
dustbin.SetTransform(t)
e.Save(f)
