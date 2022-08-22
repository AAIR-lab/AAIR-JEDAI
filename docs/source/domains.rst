Domains and Problems
===================================

Default Domains
-------------------

JEDAI comes with two default domains

.. toctree::
   :maxdepth: 2

   domains/keva
   domains/hanoi

Creating new Domains
----------------------

.. warning ::

   This feature is recommended for advanced users only

To create new domains you must follow certain guidelines for your custom domains to work with JEDAI.


Each domain has a separate folder in 'modules' directory. The name of that folder would be the name of that domain. E.g., for the domain "Keva Planks" the name of the folder would be "Keva Planks". The structure for each domain would be as following:

::

   | <Name of the domain>
   |-- domain.pddl
   |-- <Name of problem 1>
         |-- problem.pddl
         |-- env.dae
         |-- img.png
   |-- <Name of problem 2> 
         |-- problem.pddl 
         |-- env.dae
         |-- img.png 


Domain File
*************

The domain.pddl file contains semantics of the planning problem and actions descriptions. Each action contains an action name, a list of parameters a set of preconditions and a set of effects. Rename your domain file to "domain.pddl" and place it in your domain folder as shown in the directory structure above. 

Important points to remember while making the domain file:
* The action names must be in the lowercase. 
* The parameter list must be in the alphabetical order. E.g., if your action has three parameters ?robot, ?from, and ?to, then the order must be ?from, ?robot, and ?to. 
* Disjunctive preconditions are not supported yet. 
* Conditional effects are not supported yet. 
* Quantified preconditions and/or effects are not supported yet. 
* No comments are allowed in the domain file.
| 


Problem File
***************
A problem file defines a universe of objects, an initial state, and the goal condition for a planning problem. Each problem would need a separate problem file. For each new problem, create a sub directory under the domain directory. Rename your problem file to "problem.pddl" and place it in the problem directory as shown in the directory structure above. 

Important points to remember while making the problem file:
* The object names should be all lowercase. 
* No comments are allowed in the problem file.

|

Environment File
******************
An environment file contains the low-level 3d setup for the problem. A separate environment file is required for each problem. Rename your environment file to "env.dae" and place it in the problem sub directory as shown in the directory structure above. 

Important points to remember while making the environment file:
* The environment should either be a collada file in a dae (version 1.5) format or an xml file compatible with OpenRave. 
* The object names in the environment file should exactly match to object names in the problem file. 
* For more information on making an environment file, visit https://github.com/AAIR-lab/Anytime-Task-and-Motion-Policies/blob/master/README.md and http://openrave.org/docs/latest_stable/ 

|

Goal Image
*************
A goal images helps the users to visualize what goal they are planning for. Rename the goal image to "img.png" and place it in the problem sub directory as shown in the directory structure above. 

Semantics File
****************

The semantics file is used to generate "natural language" versions of actions and predicates. The general structure of the semantics file is something like this:

.. code-block:: xml

   {
      "predicates": [
         {
            "name": "<predicate_name>",
            "subjectIndex": <subject_index>,
            "display": "<predicate_display>"
         },
         ...
      ],
      "actions": [
         {
            "category": "<category_name>"
            "name": "<action_name>",
            "display": "<action_display>",
            "tooltip": "<info_to_show_on_hovering>",
            "parameters": [
               {
                   "index": <parameter_index>,
                   "display": "<text_to_show_before_dropdown>",
                   "sentenceDisplay": "<text_to_show_in_explanations>"
               },
               ...
            ]
         },
         ...
      ]
   }

Each template element that needs to be filled in is explained below:


1. **<predicate_name>**: This is the name of the predicate used in the corresponding domain and problem files. The user will not necessarily see this.
2. **<subject_index>**: This gives the (zero-based) index of the "subject" of the predicate. The subject of the predicate is whichever parameter would come before the "is" in a sentence describing that predicate.
3. **<predicate_display>**: This is the text that would come after the "is" in a sentence describing the predicate. For example, if you wanted a full phrase like "plank1 is standing vertically", **<predicate_display>** would need to be "standing vertically". If there are more objects in your predicate, you should add those to this field as well. To make it so that the object for parameter at index **<index>** gets substituted into **<predicate_display>**, include that index between a pair of double percent signs in the string, like so: "%%\ **<index>**\ %%".
4. **<action_category>**: This is the category of the action used to bin the actions in the action selection list. The user will see this name in the list.
5. **<action_name>**: This is the name of the action used in the corresponding domain and problem files. The user will not necessarily see this.
6. **<action_display>**: This is the initial part of any natural language meant to encapsulate this action. It should consist of anything needed to indicate which action is being discussed before the parameters are introduced. For example, to get the sentence "Pick up plank plank1 with robot yumi using gripper left", the **<action_display>** should be "Pick up".
7. **<info_to_show_on_hovering>**: This is a longer-form generic description of the action that pops up when a user hovers their mouse over an action block.
8. **<parameter_index>**: This gives the (zero-based) index of this parameter in the domain and problem files. Note that the order of parameters in the action blocks and generated natural language text does not look at this - instead it uses the order of the parameters in the semantics file array.
9. **<text_to_show_before_dropdown>**: This is the text that is shown just to the left of the dropdown for this parameter in the action block. For the first parameter in the array (and thus in the action block) this will usually look something like "this **<parameter_type>**\ ", where **<parameter_type>** is the type of the first parameter. Every other parameter should include some text before the type that explains how the object is involved in the action. See the included domains for clarifying examples.
10. **<text_to_show_in_explanations>**: This is the text that immediately precedes the name of the object that has been instantiated for this parameter when the action shows up in an explanation. Will usually be similar to **<text_to_show_before_dropdown>**, but without any "this" in the string. Again, see the included domains for clarification.


|

Other Requirements
********************

Apart from these requirements, the domain has to be setup in task and motion planning framework with the same name used to define the domain in the domain.pddl under "TMP_Merged/test_domains/" directory. For steps to setup this domain, visit: https://github.com/AAIR-lab/Anytime-Task-and-Motion-Policies/blob/master/README.md 

|
|
