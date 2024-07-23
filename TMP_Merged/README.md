# Task and Motion Policies for Stochastic Environments

In order to solve complex, long-horizon tasks,
intelligent robots need to be able to carry out high-level,
abstract planning and reasoning in conjunction with motion
planning. However, abstract models are typically lossy and
plans or policies computed using them are often not executable
in practice. These problems are aggravated in more realistic
situations with stochastic dynamics, where the robot needs to
reason about, and plan for multiple possible contingencies.
We present a new approach for integrated task and motion
planning in stochastic settings.

Full paper is available at: https://aair-lab.github.io/atam/atam_full_version.pdf

The repository servers as an implementation of the paper mentioned above providing a framework for task and motion planning based on abstraction. It takes the abstract specification, concrete specification, and the means of concretizing the abstraction as the input and produces a task and motion plan which can be directly executed on a real robot.

## Installation

Requirements
- Python Version    : 2.7.9 to 2.7.17
- Ubuntu OS Version : 16.04 / 18.04

To install all the dependencies related to TMP please run the install_tmp_dependencies.sh script.

To run the code, run **_TMP.py_**


## Inputs
### 1. Abstract High level domain and problem file.

Abstract high level domain can be provided in the form of PDDL or PPDDL. Two different planners FF and LAO* solver are available to solve PDDL and PPDDL problems respectively. 

For adding a new domain, create a new directory in test_domains/ directory, e.g. test_domains/ExampleDomain, where 'ExampleDomain' is the name of the new domain.
Provide paths for domain and problem files in test_domains/ExampleDomain/DomainConfig.py and choose the appropriate task planner (FF for deterministic planning and LAO* Solver for stochastic problems) here.

Config.py contains default configurations for executing the code. You can select the domain you want to execute here. It provides default configuration arguments, which can be overwritten by test_domains/ExampleDomain/DomainConfig.py.


### 2. ActionConfig

This is the main part of the inputs. It maps high level abstract actions with their low level counterparts. This file defines the configuration of  low level actions including low level arguments, preconditions, and effects which are abstracted in the high-level. The example of this file can be found at test_domains/DemoActionfig.json, which specifies an ActionConfig for a pick and place domain.

**config_map** in the file specifies the actions which requires low level refinements. All the actions which do not require any low level refinements should be places in **ignore_hl_actions** in the ActionConfig file.

##### Config_map
For each action in **config_map** following properties need to be defined.
- name: The name of the action should match the name of the PDDL/PPDDL action. 
- HL_ARGS: This specifies the high-level arguments. The sequence of these arguments should match the sequence of arguments (parameters) defined in PDDL/PPDDL. 
- LL_ARGS: This specifies the low-level arguments, it's generators, and the type of the low level arguments.For example, 
```
LL_ARGS : {
...
    "traj" : [MotionPlanGenerator, "ManipTrajectory"],
... 
}
```
This specifies that the low-level argument **traj** can be generated using the generator **MotionPlanGenerator** and it is of type **ManipTrajectory**. The generator needs to be placed in **test_domains/ExampleDomain/Generators**.

```
HL_ARGS: ["obj",...]
}
```

High-level arguments do not require any generators, and will be instantiated with values from computed high level policy. 

- Precondition: The low-level preconditions are mentioned here. The sequence of precondition is important. Every precondition uses LL_ARGS. The sequence of arguments in precondition is also important.
```
"precondition" : [
    ...
    "IsValidGraspPose(robot,object,grasp_pose)",
    ...
] 
```
All the predicates in preconditions are evaluated sequentially. When it encounters a low-level argument, it uses it's generator to generate a value for that argument. 

If already generated value for some argument needs to be reused, an alias can be used. For example,
```
"precondition" : [
    ...
    "IsValidMotionPlan(robot,pose_current:cpose,pose_end:grasp_pose,trajectory)",
    "ObstructionFree(traj:trajectory)",
    ...
]
```
Here the IsValidMotionPlan uses the pregenerated values for *cpose* and *grasp_pose* using aliases **pose_current** and **pose_end**. 

For action to be executed in the low-level, all the predicates in low-level precondition should evaluate to be true in the given state. 

- Execution_sequence: The list specifies the sequence in which the low-level generated arguments needs to be executed. The code will look at the type of the argument and call the action executor will be executed accordingly. 

- Effect (optional): This specifies all the low level effects. It has the same format as precondition, but aliases are not required here. The sequence of the predicates in the effect is also important. All the predicates mentioned here needs to be placed at **test_domains/ExampleDomain/Effects**. This can be an empty list. 



##### ignore_hl_actions

This specifies which high-level actions do not need any low-level refinement and needs to be ignored while the policy is being refined.

##### non_removable_bodies

This specifies which bodies should not be removed to find motion plans with collisions in case of no collision free motion plans are available.

### 3. Generators

A Generator is required for all the abstracted low-level arguments. It concretizes the abstracted argument with one of the valid values for the argument from it's domain. It is implemented as a python iterator which returns a different value whenever invoked. 

Place the generators in the **test_domains/ExampleDomain/Generators** and add the class in the **test_domains/ExampleDomain/\_\_init\_\_.py**. The generator class needs extend the **src/DataStructures/Generator.py** and provide the following methods.

    def __init__(self, ll_state, known_argument_values):
            ll_state: This is an instance of current low level state and object of LowLevelState (or the one selected in the Config.py)
            known_argument_values: This is a python dictionary containing values for all the HL_ARGS for the current action, values for LL_ARGS generated till the point, and high level states before and after the action.
        The method should call the __init__ for the super class Generator.
       
    def get_next(self, flag):
            flag: This is a boolean variable indicating whether the framework is in model update mode or not.
        This method method is called by the framework every time a new value is required. It should return a new value when called. 
     
    def reset(self):
        This method should reset the current state of the iterator to yield values from the first value.


An example generator is available at test/Generators/GraspPoseGenerator.py. This generator is used to instantiate grasp poses for the objects. More information on witting a generator is available in the example file.

### 4. Predicates

Predicates are used in precondition and effects. All the predicates needs to be placed in **test_domains/ExampleDomain/Predicates** and should inherit **DataStructure/Predicate**. 

If the predicate is being used as an effect, it should have an **apply** method.


If the predicate is being used as a precondition, it needs to be evaluated true for action to be executed in the state. There are two ways to do so,
- One way to do so is to generate the arguments for the predicate in a way such that it is always true. For example, the grasp pose generators generates only valid grasping poses, removing the need to externally evaluate the predicate.
- Another way to do so is to have a class for predicate and a method that evaluates the arguments in current state and decide if the predicate holds true or not. For example test_domains/ExampleDomain/Predicates/NotObstructs.py, externally evaluates the predicate in the given low-level state. In this case, the class should be named same as the predicate name used in the actionConfig and have **\_\_call\_\_** method. 


More information on writing a predicate is available at demo predicate file.


### 5. Executors

Different types of low-level arguments might need to be executed differently in low-level. To execute the arguments specified in **exec_sequence** in **ActionConfig**,  a class needs to be written for the type of that argument inheriting **ArgExecutor** from **src/DataStructures/ArgExecutor.py**. The class should be placed at **test_domains/ExampleDomain/Executors**. A demo file is available at **test_domains/ExampleDomain/Executors/DemoExecutor** which provides and executor for arguments of type *ManipTrajectory*. The name of the class should match the argument type. For e.g,

```
"LL_ARG : {
...
"traj": ["MotionPlanGenerator", "ManipTrajectory"],
...
}
``` 
In this case, the file *ManipTrajectory.py* containing class *ManipTrajectory* needs to be present at **test_domains/ExampleDomain/Executors**, which specifies how arguments of type *ManipTrajectory* need to be handled/executed.

### 6. Low-level Environment

Provide the 3D environment for the OpenRave simulator in the form of .dae or .xml file. Generate this file using OpenRave and store it using **env.Save()**. The robot will always be spawned at (0,0,0), so keep this in mind while designing the environment.

Provide path to the 3D environment in test_domains/ExampleDomain/DomainConfig.py 

### 7. Executing Trajectories

After TMP is completed, you can view the execution of trajectories by calling 'python src/Simulators/OpenRaveTrajectoryExecutor.py'. Please note that for this to work, trajectories need to be saved. You can do this by setting STORE_TRAJ flag to be true in your domain configuration file. If your domain 'ExampleDomain' contains stochastic choices, include the file test_domains/ExampleDomain/StochiasticPolicy.py to choose the outcome nodes based on propositions.