# JEDAI Explains Decision-Making AI


This repository contains the code for the tool JEDAI. System Demonstration of JEDAI is accepted at AAMAS 2022, and will appear as:

JEDAI: A System for Skill-Aligned Explainable Robot Planning.<br/>
[Naman Shah](https://www.namanshah.net/)\*, 
[Pulkit Verma](https://pulkitverma.net)\*, 
[Trevor Angle](http://trevorangle.com), and 
[Siddharth Srivastava](http://siddharthsrivastava.net/). <br/>
21st International Conference on Autonomous Agents and MultiAgent Systems, 2022. <br/>

[Paper](https://aair-lab.github.io/Publications/svas_aamas22.pdf) | [Demo Talk](https://www.youtube.com/watch?v=MQdoikcnhbY) | [Tutorial](https://www.youtube.com/watch?v=57os8Ap1N5U)
<br />



## Virtual Machine Image

The recommended way of using JEDAI is to use pre-configured Virtual Machine image that is available here: [https://bit.ly/2WccU4K](https://bit.ly/2WccU4K)

To setup the system manually, you can use the steps given below:

## Running JEDAI

Use this command to start JEDAI from the JEDAI source directory (~/JEDAI/ in VM Image).

`./start_jedai.sh`

Alternatively execute this command:

`python3 manage.py runserver`

The output of this command includes a link to the development server hosting the frontend. 

You can stop the execution anytime using this command from the JEDAI source directory (~/JEDAI/ in VM Image):


`./stop_jedai.sh`




## Installing JEDAI on a new system

### Requirements

- Ubuntu 18.04
- Python 2 and 3
- Validate: https://github.com/KCL-Planning/VAL
  1. Retrieve and enter the repo:
     
     `git clone https://github.com/KCL-Planning/VAL`
     
     `cd VAL`
     
  2. Build the binary:
    
     `./scripts/linux/build_linux64.sh all Release`
  
     - This will put `Validate` in `<PARENT_DIR>/VAL/build/linux64/Release/bin`
    
 NOTE: JEDAI is tested extensively with Chromium (including Edge, Vivaldi, and Google Chrome). Support on other browsers is not guaranteed.  

    
### Setup

1. Retrieve the `TMP_Merged` submodule by running the following in the project root
   (unless you already have `TMP_Merged` somewhere else on your system and want to use that,
    in which case you can try a symlink):

   `git clone https://github.com/AAIR-lab/ATAM.git TMP_Merged`
    
    1. You must then install the dependencies for the submodule (this will probably take a while):
    
        `bash TMP_Merged/install_tmp_dependencies.sh`
    
    2. Also make sure to check out the correct branch of the submodule:
        
        `cd TMP_Merged`
         
        `git checkout origin/TMP_JEDAI`


2. Install the web framework:

    `pip3 install django`


3. Install the YAML library:

   `pip3 install PyYAML`


4. Install the PDDL library:

    `pip3 install pddlpy`
    
    - If you get an error while running the code about a missing module named `__builtin__` in the `antlr4` library, then running this should help:
        
        `pip3 install antlr4-python3-runtime==4.7`

5. Install the imaging library:
    
    `pip3 install Pillow`


4. Check that `PYTHON_2_PATH` and `VAL_PATH` in `config.py` are pointing to the corresponding binaries on your system.


You are required to submit a domain and problem file, as well as a .dae environment file. See the `test_domains` directory for examples.

### TMP submodule

After installing its dependencies, the TMP submodule should work out of the box, with environments popping up and giving a demonstration of successful plans.
If you get any strange import errors from TMP despite packages seeming to be installed correctly, double-check your
all your environment variables (especially if using an IDE like PyCharm).


## Contributors

[Naman Shah](https://www.namanshah.net/)<br/>
[Pulkit Verma](https://pulkitverma.net)<br/>
[Trevor Angle](http://trevorangle.com) <br/>
[Kiran Prasad](https://github.com/kiranprasad)<br/>
[Amruta Tapadiya](https://github.com/amy88amy)<br/>
[Kyle Atkinson](https://github.com/KyleTheEpic)<br/>
[Chirav Dave](https://chiravdave.github.io/)<br/>
[Judith Rosenke](https://www.linkedin.com/in/judith-rosenke/)<br/>
[Rushang Karia](https://github.com/RushangKaria)<br/>
[Siddharth Srivastava](https://siddharthsrivastava.net/)




# Citation
```
@inproceedings{shah_2022_jedai,
    author = {Shah, Naman and Verma, Pulkit and Angle, Trevor and Srivastava, Siddharth},
    title = {{JEDAI: A System for Skill-Aligned Explainable Robot Planning}},
    booktitle = {Proceedings of the 21st International Conference on Autonomous Agents and MultiAgent Systems},
    year={2022}
}
```
