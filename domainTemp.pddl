(define (domain cafeWorld)
    (:requirements :typing :strips :adl :equality)
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
        :parameters(?loc - location ?r - robot)
        :precondition(
        )
        :effect(and
            (at ?loc ?r)
        )
    )
    (:action grasp
        :parameters(?g - manipulator ?loc - location ?obj - can ?r - robot)
        :precondition(and
            (empty ?g)
            (at ?loc ?r)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
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

