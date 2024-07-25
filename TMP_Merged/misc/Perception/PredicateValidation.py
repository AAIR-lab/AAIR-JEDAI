from ScanWorld import ScanWorld

def validate(predicate_str):
    '''Returns whether true or false'''
    pass



def on(name_object_on_top, name_object_at_bottom):
    '''
    REQUIRES that the origin of the object_on_top be on the top surface
    AND that the origin of the name_object_at_bottom be on the bottom surface
    '''

    ###TEST
    # env = Environment()
    # aair_lab_xml = '/home/midhun/Documents/Fetch-Rave/res/lab.env.xml'
    # env.Load(aair_lab_xml)
    # env.SetViewer('qtcoin')
    # object_on_top = env.GetKinBody(name_object_on_top)
    # object_at_bottom = env.GetKinBody(name_object_at_bottom)
    # z_object_on_top = object_on_top.GetTransformPose()[-1]
    # z_object_at_bottom = object_at_bottom.GetTransformPose()[-1]
    ##TEST

    z_object_on_top = ScanWorld.instance().get_object_name_transform_map().get(name_object_on_top)[-1]
    z_object_at_bottom = ScanWorld.instance().get_object_name_transform_map().get(name_object_at_bottom)[-1]

    return z_object_on_top > z_object_at_bottom



if __name__ == "__main__":
    on('clorox', 'drawers')
