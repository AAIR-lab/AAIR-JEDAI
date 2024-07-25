import openravepy
import numpy as np
from operator import truediv

def create_dest(env, destname, t, vert=False):
  dim = 0.1
  THICKNESS = 0.001

  surface = openravepy.KinBody.GeometryInfo()
  surface._type = openravepy.GeometryType.Box
  if vert:
    surface._vGeomData = [THICKNESS/2, dim/2, dim/2]
  else:
    surface._vGeomData = [dim/2, dim/2, THICKNESS/2]
  surface._vDiffuseColor = [1, 0, 0]

  dest = openravepy.RaveCreateKinBody(env, '')
  dest.InitFromGeometries([surface])
  dest.SetName(destname)

  dest.SetTransform(t)

  return dest

def on_table(obj, table):
  T = obj.GetTransform()

  table_ab = table.ComputeAABB()
  table_top_z = table_ab.pos()[2] + table_ab.extents()[2]

  if "cloth" in obj.GetName():
    T[2, 3] = table_top_z + 0.03
    obj.SetTransform(T)
  else:
    obj_ab = obj.ComputeAABB()
    obj_min_z = obj_ab.pos()[2] - obj_ab.extents()[2]

    diff_z = obj_min_z - table_top_z -0.01
    T[2, 3] -= diff_z-0.001
    obj.SetTransform(T)

def create_box(env, body_name, t, dims, color=[0,1,1]):
  infobox = openravepy.KinBody.GeometryInfo()
  infobox._type = openravepy.GeometryType.Box
  infobox._vGeomData = dims
  infobox._bVisible = True
  infobox._vDiffuseColor = color
  infobox._t[2, 3] = dims[2] / 2

  box = openravepy.RaveCreateKinBody(env, '')
  box.InitFromGeometries([infobox])
  box.SetName(body_name)
  box.SetTransform(t)

  return box

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
  infocylinder = openravepy.KinBody.GeometryInfo()
  infocylinder._type = openravepy.GeometryType.Cylinder
  infocylinder._vGeomData = dims
  infocylinder._bVisible = True
  infocylinder._vDiffuseColor = color
  infocylinder._t[2, 3] = dims[1] / 2

  cylinder = openravepy.RaveCreateKinBody(env, '')
  cylinder.InitFromGeometries([infocylinder])
  cylinder.SetName(body_name)
  cylinder.SetTransform(t)

  return cylinder

def cylinder_collisionfree_model(env, body_name, pos, dims):
  iv_str = '#Inventor V2.1 ascii\nSeparator {\nCylinder {\nparts ALL\nradius %f\nheight %f\n}\n}'%(dims[0], dims[1]) # hardcode collision-free disk dimensions
  with open("cylinder.iv", "w+") as f:
    f.write(iv_str)
  xml_str = '<KinBody name="%s"> <Body name="surface" type="dynamic"> <Geom type="sphere"> <Render>cylinder.iv</Render> <Radius>0.0001</Radius> \
<Translation>0 0 %f</Translation> <RotationAxis>1 0 0 90></RotationAxis> </Geom> </Body> </KinBody>'%(body_name, dims[1] / 2)
  with open("temp_cylinder.xml", "w+") as f:
    f.write(xml_str)
  env.Load("temp_cylinder.xml")
  cylinder = env.GetKinBody(body_name)
  x, y, z = pos
  cylinder_t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
  cylinder.SetTransform(cylinder_t)

  return cylinder

def create_table(env,
                 table_name,
                 dim1,
                 dim2,
                 thickness,
                 legdim1,
                 legdim2,
                 legheight,
                 pose,
                 color):
  
  x, y, z = pose

  tabletop = openravepy.KinBody.GeometryInfo()
  tabletop._type = openravepy.GeometryType.Box
  tabletop._vGeomData = [dim1/2, dim2/2, thickness/2]
  tabletop._t[0,3] = 0 #x
  tabletop._t[1,3] = 0 #y
  tabletop._t[2,3] = 0
  tabletop._vDiffuseColor = color

  leg = openravepy.KinBody.GeometryInfo()
  leg._type = openravepy.GeometryType.Box
  leg._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg._t[0,3] = 0 #x
  leg._t[1,3] = 0 #y
  leg._t[2, 3] = (-legheight/2 - thickness/2)
  leg._vDiffuseColor = [0.5, 0.2, 0.1]

  tablebottom = openravepy.KinBody.GeometryInfo()
  tablebottom._type = openravepy.GeometryType.Box
  tablebottom._vGeomData = [dim1/4, dim2/4, thickness/8]
  tablebottom._t[0,3] = 0 #x
  tablebottom._t[1,3] = 0 #y
  tablebottom._t[2, 3] = (-legheight - thickness/2)
  tablebottom._vDiffuseColor = [0.5, 0.2, 0.1]

  table = openravepy.RaveCreateKinBody(env, '')
  table.InitFromGeometries([tabletop, leg, tablebottom])
  table.SetName(table_name)
  table.SetTransform(openravepy.matrixFromPose([1,0,0,0,x,y,z]))
  print(table.GetTransform())
  print(table.ComputeAABB())

  return table

if __name__ == "__main__":
  env = openravepy.Environment()
  env.SetViewer('qtcoin')
  env.Load('./world_final.stl')
  print(env.GetBodies())
  thickness = 0.1
  legheight = 0.55
  pose = [4, 5, 0.63]
  tables = [[4, 8, 0.63], [6, 8, 0.63], [4, 10, 0.63], [6, 10, 0.63]]
  for i, pose in enumerate(tables):
    env.Add(create_table(env, 'table_'+str(i), 0.90, 0.90, thickness, 0.1, 0.1, legheight, pose))
  while True:
    pass
