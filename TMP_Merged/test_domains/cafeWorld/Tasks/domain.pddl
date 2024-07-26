(define (domain cafeWorld)
    (:requirements :typing :strips :adl :equality)
    (:types
        object
        gripper
        pose
        trajectory
        robot
        base_pose
        location
    )
    (:predicates
        (empty ?gripper - gripper)
        (ingripper ?obj - object ?gripper - gripper)
        (at ?loc - location ?r - robot)
        (order ?obj - object ?loc - location)
    )

    (:action move
        :parameters(?loc - location ?r - robot)
        :precondition(
            not (at(?loc ?r))
            )
        :effect(
            at (?loc ?r)
            )
    )
    (:action grasp
        :parameters(?g - gripper ?loc - location ?obj - object ?r - robot)
        :precondition(and
            (empty ?g)
            not (order ?obj - object ?loc - location)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
        )
    )
    (:action put
        :parameters(?g - gripper ?loc - location ?obj - object ?r - robot)
        :precondition(and
            (ingripper ?obj ?g)
            not (order ?obj - object ?loc - location)
        )
        :effect(and
            (not (ingripper ?obj ?g))
            (empty ?g)
            (order ?obj - object ?loc - location)
        )
    )
)