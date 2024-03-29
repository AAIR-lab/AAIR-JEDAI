Getting Started with JEDAI
===================================

The goal of this system to introduce AI planning concepts using mobile manipulator robots. It uses a visual programming interface to make these concepts easier to grasp. Users can get the robot to accomplish desired tasks by connecting jigsaw-like blocks that encode the robot's possible actions. This allows users to carry out navigation, planning and manipulation by dragging and dropping blocks instead of writing code.

The following steps will help you get started:

Step 1: Start the System
------------------------------

Open the home page using the following command in Terminal from the root directory of JEDAI:

.. code-block:: bash

  ./start_jedai.sh

.. note::

  You can stop the system anytime using the following command:

  .. code-block:: bash

    ./stop-jedai.sh

|

Step 2: Choose the domain and problem
------------------------------------------

You will need to choose a domain and a problem within that domain to use JEDAI.

.. note::
  You can learn more about the domains `here`_.

Follow these steps to pick a domain and problem:

1. Choose a domain from the dropdown menu to play around with. There are six default options to choose from: `Cafe World`_, `Delicate Cans`_, `Dominos`_, `Keva Planks`_, `Stack of Towers`_, and `Tower of Hanoi`_.

.. image:: ./images/choose_domain.png
  :width: 300
  :alt: Choosing the domain

|

2. Choose a problem from the next dropdown. The problems shown will all correspond to the domain chosen in the previous step.

.. image:: ./images/choose_problem.png
  :width: 300
  :alt: Choosing the problem

|

Step 3: Learn to Plan
----------------------

In the field of automated planning, a "plan" is defined as a sequence of actions, where an action represents something
the robot can do to change the state of the world. In JEDAI, you will learn how these plans work and the kind of requirements
that are imposed on you when dealing with a system that does not have its own common sense to guess at what you really mean.

|

1. Once you have entered the JEDAI training area, you will see something like this:

.. image:: ./images/plan_page.png
  :width: 700
  :alt: Plan input page

|

The white space in the middle of your browser window (it has just the Start block for now) is the planning area,
called the workspace. To start building a plan, simply click the Actions tab in the toolbox on the left of the workspace
and drag actions out into the blank area. Actions snap together when you line them up vertically, and the first action
must be attached to the Start block.

|

A separate window with the 3D simulation environment will also appear as shown below. 
You can zoom in/zoom out, or move the camera angle to view the objects properly in this 3D environment.


.. image:: ./images/openrave_example.png
  :width: 400
  :alt: The 3D simulation environment window

|

You will also be presented with "Goal state" above the workspace and the goal image or goal
condition on the right side of your screen as shown below.
These tell you what you
are trying to accomplish for the current domain and problem. For example, in the Keva Planks domain, the 3D environment
window will show a table with planks neatly organized in rows. The goal image shows how the planks should be configured
by the end of your plan.

.. image:: ./images/plan_area.png
  :width: 700
  :alt: Plan input page

|

1. You must choose a sequence of actions that the YuMi robot should
execute to reach the goal configuration using a drag-and-drop interface as shown below.

.. image:: ./images/drag_drop_the_action.png
  :width: 700
  :alt: Dragging and Dropping an Action to create a plan

|

If you are ever unsure about what an action does, just hover your mouse over the action block and a tooltip will appear
with more information.

.. image:: ./images/hover_action.png
  :width: 700
  :alt: Hovering over an action to see what it does

|

3. You must choose the correct parameters of the actions using one of the parameters from
each of the dropdown menu available in each action.

.. image:: ./images/choose_parameter.png
  :width: 300
  :alt: Choosing an action parameter

|

4. Submit the plan using the ``Submit Plan`` button on the top left of your workspace.

|

So get planning! Once you have a series of actions with selected parameters that you think should reach the goal state,
go ahead and click the green Submit Plan button. If your plan doesn't quite work, JEDAI will tell you `why`_. If your plan
successfully reaches the goal, then TMP (the system that turns a high-level plan into a series of low-level
instructions that the robot uses to move its individual joints) starts working. Once you are alerted that TMP has finished
computing the solution to the plan, you can open the 3D environment window to watch the robot follow the steps lined out
in your plan!

|
|


.. _here : ./domains.html
.. _Cafe World : ./domains/cafe.html
.. _Delicate Cans : ./domains/delicateCans.html
.. _Dominos : ./domains/domino.html 
.. _Keva Planks : ./domains/keva.html
.. _Stack of Towers : ./domains/towers.html
.. _Tower of Hanoi : ./domains/hanoi.html
.. _why : ./explanations.html