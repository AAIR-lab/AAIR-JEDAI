Tower of Hanoi
================

This domain is inspired from the classic `Tower of Hanoi`_ puzzle.
This version consists of three locations and a number of boxes of various sizes as shown below.

.. image:: ../images/hanoi/hanoi_domain.png
  :width: 600
  :alt: Domain with YuMi robot and Keva planks

|

The initial state has the boxes stacked on Location I in order of decreasing size, the smallest at the top.
The goal is to move the entire stack to Location III, obeying the following rules:

1. Only one box may be moved at a time.
2. Each move consists of taking the upper box from one of the stacks and placing it on top of another stack or on an empty location.
3. No box may be placed on top of a box that is smaller than it.

|

The actions that the Fetch robot can take in this environment are:

|

1. **Move a box from top of another box to an empty location**:
Use this action to pick up a box that is sitting on top of another box and place it in an empty location.

.. image:: ../images/hanoi/hanoi_move_box_box_loc.png
  :width: 200
  :alt: Move a box from top of another box to an empty location

|

2. **Move a box from top of one box to top of another box**:
Use this action to pick up a box that is sitting on top of another box and place it on top of another box.

.. image:: ../images/hanoi/hanoi_move_box_box_box.png
  :width: 200
  :alt: Move a box from top of one box to top of another box

|

3. **Move a box from one location to top of another box**:
Use this action to pick up a box that is sitting at a location and place it on another box.

.. image:: ../images/hanoi/hanoi_move_box_loc_box.png
  :width: 200
  :alt: Move a box from one location to top of another box

|

4. **Move a box from one location to another location**:
Use this action to pick up a box that is sitting at a location and place it in an empty location.

.. image:: ../images/hanoi/hanoi_move_box_loc_loc.png
  :width: 200
  :alt: Move a box from one location to another location

|


.. _Tower of Hanoi : https://en.wikipedia.org/wiki/Tower_of_Hanoi
