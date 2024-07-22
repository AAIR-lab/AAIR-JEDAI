import re
import os

# project_dir = '/home/local/ASUAD/asrinet1/AAIR/ENV/Models/machine_base/'
# base_file = "Models/machine_base/machine_base.dae"
# output_dir = 'current_env/'
# sdf_template = 'Models/template.sdf'

def generate_model_files(model_name, base_file_path, output_dir='current_env/', models_dir='Models/'):
    '''
    Generates .dae(v1.4) .dae(v1.5) and .sdf files for loading in the environment
    .dae v1.4 and .sdf files are used to load the models in gazebo
    .dae v1.5 is used to load the models in openrave
    '''

    sdf_template = models_dir + 'template.sdf'
    dae15_file = output_dir + "{}.dae".format(model_name)
    with open(base_file_path, "r") as infile, open(dae15_file, "w") as outfile:
        data = infile.read()
        data = data.replace('xmlns="http://www.collada.org/2005/11/COLLADASchema"', 
                            'xmlns="http://www.collada.org/2008/03/COLLADASchema"')
        data = data.replace('version="1.4.1"', 'version="1.5.0"')
        data = data.replace('<node name="SketchUp"', '<node name="{}"'.format(model_name))
        outfile.write(data)
    
    dae14_file = output_dir + "{}_14.dae".format(model_name)
    with open(base_file_path, "r") as infile, open(dae14_file, "w") as outfile:
        data = infile.read()
        data = data.replace('<unit name="meter" meter="0.00000645"/>', '<unit name="meter" meter="1"/>')
        data = data.replace('<node name="SketchUp"', '<node name="{}"'.format(model_name))
        outfile.write(data)
    
    dae14_full_path = os.path.abspath(dae14_file)
    sdf_file = output_dir + "{}.sdf".format(model_name)
    with open(sdf_template, "r") as infile, open(sdf_file, "w") as outfile:
        data = infile.read()
        data = re.sub('(?<=<uri>).*?(?=<\/uri>)', 'file://' + dae14_full_path, data, flags=re.DOTALL)
        outfile.write(data)
    
    return dae15_file, dae14_file, sdf_file