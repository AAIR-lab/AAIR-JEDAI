from openravepy import *
import numpy as np
# Create new plank

HALF_PLANK_LENGTH = 0.05884
HALF_PLANK_BREADTH = 0.01162
HALF_PLANK_HEIGHT = 0.003875

def shift_table_height(env,table_type,height_diff):
    for i in env.GetBodies():
        if table_type in i.GetName():
            t = i.GetTransform()
            t[2,3] += height_diff
            i.SetTransform(t)
            
def create_plank(env,plankname,plank_transform=np.eye(4)):
    with env:
        # body = RaveCreateKinBody(env,'')
        # body.SetName(plankname)
        # body.GeometryInfo._vDiffuseColor=[0.76,0.70,0.50]
        # body.InitFromBoxes(numpy.array([[0.,0.,0.,HALF_PLANK_LENGTH,HALF_PLANK_BREADTH,HALF_PLANK_HEIGHT]]),True)
        # env.AddKinBody(body)
		        # body.SetTransform(plank_transform)
		infobox = KinBody.GeometryInfo()
		infobox._type = GeometryType.Box
		infobox._vGeomData = [HALF_PLANK_LENGTH,HALF_PLANK_BREADTH,HALF_PLANK_HEIGHT]
		infobox._bVisible = True
		infobox._vDiffuseColor = [0.76,0.60,0.50]
		# infobox._t[2, 3] = dims[2] / 2

		box = RaveCreateKinBody(env, '')
		box.InitFromGeometries([infobox])
		box.SetName(plankname)
		box.SetTransform(plank_transform)
		env.AddKinBody(box)
    return box

def create_lego(env,block_type,block_number,placed):
    df=pd.read_csv('Lego_models.csv', sep=',',header=0)
    block = df[df['Name']=='rect_6_thick']
    length,breadth = block['length'].sum()/2., block['breadth'].sum()/2.
    if placed:
        height = block['height'].sum()/2.
    else:
        height = block['height+adjusted'].sum()/2.
    print(length,breadth,height)
    with env:
        body = RaveCreateKinBody(env,'')
        body.SetName(block_type+str(block_number))
        body.InitFromBoxes(numpy.array([[length,breadth,height,length,breadth,height]]),True)
        env.AddKinBody(body)
        return body

# Load Env
def load_env(env_file):
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(env_file)
    return env

def get_structure_planks(env):
    selected_planks=[]
    for i in env.GetBodies():
        if 'plank' in str(i):
            selected_planks.append(i)
    return selected_planks


def get_station_planks(env):
    selected_planks=[]
    for i in env.GetBodies():
        if 'station' in str(i):
            selected_planks.append(i)
    return selected_planks

def get_plank_body_from_num(env,num):
    body =  env.GetKinBody('plank'+str(num))
    return body, body.GetTransform()

def create_station_base(env,plankname,plank_transform):
    with env:
        body = RaveCreateKinBody(env,'')
        body.SetName(plankname)
        body.InitFromBoxes(numpy.array([[0.,0.,0.,HALF_PLANK_LENGTH,HALF_PLANK_BREADTH*5,HALF_PLANK_HEIGHT]]),True)
        env.AddKinBody(body)
        body.SetTransform(plank_transform)
        return body

def create_station(env,plank_base_tx,direction):
    plank_base_tx[2,3] += 0.0001 #comment this line

    # base
    base = create_station_base(env,direction+'_station_base',plank_base_tx)

    #vertical
    R = np.eye(4)
    R[:3,:3] = rotationMatrixFromAxisAngle([np.pi/2,0,0])

    left = np.matmul(plank_base_tx,R)
    left[2,3] += 2*HALF_PLANK_HEIGHT
    left[1,3] += (2*HALF_PLANK_BREADTH) *2.5 + HALF_PLANK_HEIGHT + 0.001
    p_v_left = create_plank(env,direction+'_station_v_left',left)

    right = np.matmul(plank_base_tx,R)
    right[2,3] += 2*HALF_PLANK_HEIGHT
    right[1,3] -= (2*HALF_PLANK_BREADTH) *2.5 + HALF_PLANK_HEIGHT + 0.001
    p_v_right = create_plank(env,direction+'_station_v_right',right)

    # level1
    right = plank_base_tx.copy()
    right[1,3] -= (2*HALF_PLANK_BREADTH) *2
    right[2,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    p_l1_right = create_plank(env,direction+'_station_l1_right',right)

    left = plank_base_tx.copy()
    left[1,3] += (2*HALF_PLANK_BREADTH) *2
    left[2,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    p_l1_left = create_plank(env,direction+'_station_l1_left',left)

    # level2
    right[2,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    right[0,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    p_l2_right = create_plank(env,direction+'_station_l2_right',right)

    left[2,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    left[0,3] += (2*HALF_PLANK_HEIGHT+0.00001) *1
    p_l2_left = create_plank(env,direction+'_station_l2_left',left)

