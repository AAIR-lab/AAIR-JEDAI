(define (domain cafeWorld)
    (:requirements :typing :strips)
    (:types
        can
        manipulator
        robot
        location
    )

    (:predicates
        (empty ?gripper - manipulator)
        (ingripper ?obj - can ?gripper - manipulator)
        (at ?loc - location ?r - robot)
        (order ?obj - can ?loc - location)
    )


    (:action move
        :parameters(?fromLoc - location  ?r - robot ?toLoc - location)
        :precondition(and
            (at ?fromLoc ?r)
            (not (at ?toLoc ?r))
        )

        :effect(and
            (at ?toLoc ?r)
            (not (at ?fromLoc ?r))
        )
    )

    (:action grasp
        :parameters(?g - manipulator ?loc - location ?obj - can ?r - robot)
        :precondition(and
            (empty ?g)
            (order ?obj ?loc)
            (at ?loc ?r)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
            (not (order ?obj ?loc))

        )
    )
    (:action put
        :parameters(?g - manipulator ?loc - location ?obj - can ?r - robot)
        :precondition(and
            (ingripper ?obj ?g)
            (at ?loc ?r)
        )
        :effect(and
            (not (ingripper ?obj ?g))
            (empty ?g)
            (order ?obj ?loc)
        )
    )
)