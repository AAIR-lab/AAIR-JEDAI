Explanations
===================================

JEDAI supports two kinds of explanations to help users create valid plans.
These explanations leverage heuristics for how difficult different concepts are to understand so that the content of an
explanation is as intelligible as possible.

Explanations of Failed Preconditions
------------------------------------
Plans that include actions with unmet preconditions are invalid.
JEDAI explains such problems by stating what action failed at what step, along with the unmet precondition.


For example:
    If the robot is directed to pick up a plank with its left gripper that can only be reached with its right gripper,
    then an explanation will be generated that looks something like:

        The action at step 3 (Pick up plank_vi with the left gripper) could not be executed because the left gripper is not able to reach plank_vi.


Explanations of Failed Goals
----------------------------
Plans that do not meet all goal conditions are invalid.
JEDAI explains such problems by stating one or more of the unmet goals.

For example:
    If the goal is to build a pi structure from planks but the submitted plan only places two of the three required planks, then an explanation will be generated that looks something like:

        Plank_iii is not on top of plank_i and plank_ii. The goal state is not achieved!

|
|
