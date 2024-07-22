# env_generator.ipynb

There are 4 configurations of the 2 rooms. The location of the first room is fixed, the second room is created based on the config value  
Config - Location  
1 - WEST  
2 - NORTH  
3 - EAST  
4 - SOUTH  

The walls for the rooms are rotated accordingly to the location of the second room.  

The environment consists of 16 machines aligned in a 4x2 grid in each rooom. A tool shelf is also added to the environment on which the ply object is placed initially. There are 4 types of machines.   

All model files are placed in the "Models" folder.  

Outputs a `env.dae` and `env_config.json` file to be used with TMP.  

# Load environment to gazebo

Run the following on separate terminals  
`roscore`  
`rosrun gazebo_ros gazebo` (you will have to delete the ground plane loaded by default)  
Run the cell titled `Load Openrave Environment to Gazebo` in `env_generator.ipynb`  


# model_file_generator.py  

- Modifies the model .dae files so they can loaded into the openrave and gazebo environment.  
Generates 3 files for each model: .dae(v1.5), .dae(v1.4), .sdf  
.dae v1.5 is used to load the model in openrave  
.dae v1.4 and .sdf is used to load the model in gazebo  
- Generating the .sdf uses the `template.sdf` file  
The files are generated in the `output_dir` folder. The texture files must be present in this folder by default, they are not generated from the script.  
