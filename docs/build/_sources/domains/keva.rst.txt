Keva Planks
==============

This domain features an ABB YuMi Robot, and multiple Keva planks as shown below.

.. image:: ../images/keva/keva_domain.png
  :width: 600
  :alt: Domain with YuMi robot and Keva planks

|

.. note::

  You can learn more about what to do and how to do `here`_.


In all the problems for this domain, the YuMi robot must
build structures as shown in the goal configuration using the Keva planks. 
Keva planks are laser cut wooden planks with uniform geometry. 
Planks can be picked up one after another from the table
in front of YuMi.

The actions that the YuMi robot can take in this environment are:

|

1. **Pick up a plank from a table**:
   Use this action to pick up the selected plank from the table with the selected gripper. 
   A gripper can only hold one plank at a time!

.. image:: ../images/keva/keva_pickup.png
  :width: 200
  :alt: Pick up a plank from a table

|

2. **Put on table vertically**:
Use this action to pick up the selected plank from the table with the selected gripper. 
A gripper can only hold one plank at a time!

.. image:: ../images/keva/keva_puton_vertically.png
  :width: 200
  :alt: Put on table vertically

|

3. **Put on table sideways**:
Use this action to put the selected plank on the table, such that it lies on its side.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_puton_sideways.png
  :width: 200
  :alt: Put on table sideways

|

4. **Stack horizontally**:
Use this action to put the selected plank on top of another plank, such that it lies horizontally.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_horizontally.png
  :width: 200
  :alt: Stack horizontally

|

5. **Stack vertically**:
Use this action to put the selected plank on top of another plank, such that it stands vertically.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_vertically.png
  :width: 200
  :alt: Stack vertically

|

6. **Stack sideways**:
Use this action to put the selected plank on top of another (single) plank, such that the placed plank lies on its side.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_sideways.png
  :width: 200
  :alt: Stack sideways

|

7. **Stack horizontally on two horizontal planks**:
Use this action to put the selected plank on top of two other planks that are horizontal, 
such that the placed plank lies horizontally. 
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_horizontal_on_two_planks.png
  :width: 325
  :alt: Stack horizontally on two horizontal planks

|

8. **Stack sideways on two horizontal planks**:
Use this action to put the selected plank on top of two other planks that are horizontal,
such that the placed plank lies on its side.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_sideways_on_two_horizontal_planks.png
  :width: 325
  :alt: Stack horizontally on two horizontal planks

|

9. **Stack horizontally on two vertical planks**:
Use this action to put the selected plank on top of two other planks that are vertical,
such that the placed plank lies horizontally.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_horizontal_on_two_vertical_planks.png
  :width: 325
  :alt: Stack horizontally on two vertical planks

|

10. **Stack sideways on two vertical planks**:
Use this action to put the selected plank on top of two other planks that are vertical,
such that the placed plank lies on its side.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_sideways_on_two_vertical_planks.png
  :width: 325
  :alt: Stack sideways on two vertical planks

|

11. **Stack horizontally on two sideways planks**:
Use this action to put the selected plank on top of two other planks that are sideways,
such that the placed plank lies horizontally.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_horizontally_on_two_sideways_planks.png
  :width: 325
  :alt: Stack horizontally on two sideways planks

|

12. **Stack sideways on two sideways planks**:
Use this action to put the selected plank on top of two other planks that are sideways,
such that the placed plank lies on its side as well.
The selected gripper has to be holding the plank before it can place it!

.. image:: ../images/keva/keva_stack_sideways_on_two_sideways_planks.png
  :width: 325
  :alt: Stack sideways on two sideways planks

|
|

.. _here : ../getting_started.html#step-3-learn-to-plan
