;Header and description

(define (domain fow_domain)

;remove requirements that are not needed
    (:requirements :strips :adl :typing :equality)

    (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
        part
        robot
        gripper
    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates ;todo: define predicates here

        (fits ?part1 ?part2 - part)
        (placed ?part - part)
        (on ?part1 ?part2 - part) 
        (ingripper ?robot - robot ?gripper - gripper ?part1 - part)
        (gripper_empty ?robot - robot ?gripper - gripper)
        (human_adjusted ?part - part)
        (human_requested ?part - part)
        (depends ?part1 ?part2 - part)
        (can_reach ?robot - robot ?gripper - gripper ?part - part)
    )
    (:action grasp
        :parameters (?gripper - gripper ?part - part ?robot - robot)
        :precondition (and 
            (gripper_empty ?robot ?gripper)
            (can_reach ?robot ?gripper ?part)
            (not (placed ?part))
        )
        :effect (and
            (not (gripper_empty ?robot ?gripper))
            (ingripper ?robot ?gripper ?part)
        )

    )

    (:action human_request
        :parameters (?gripper - gripper ?part - part ?robot - robot )
        :precondition (and
            (ingripper ?robot ?gripper ?part)
            (forall (?part - p)
                (or (not (depends ?part ?p))(human_adjusted ?p))
            )
            (forall (?part - p)
                (or (not (fits ?part ?p))(human_adjusted ?p))
            )
        )
        :effect(and
            (human_requested ?part)
        )

    )

    (:action human_adjust
        :parameters (?gripper - gripper ?part - part)
        :precondition (and 
            (not (human_adjusted ?part))
            (placed ?part)
        )
        :effect(and 
            (human_adjusted ?part)
        )
    )

    (:action place
        :parameters(?gripper -gripper ?part1 ?part2 - part ?robot - robot )
        :precondition (and 
            (fits ?part1 ?part2)
            (ingripper ?robot ?gripper ?part1)
            (human_requested ?part1)
            (placed ?part2)
            (forall (?p - part)
                (or (not (depends ?part1 ?p))(human_adjusted ?p))
            )
        )
        :effect(and 
            (on ?part1 ?part2)
            (not (human_requested ?part1))
            (placed ?part1)
            (not (ingripper ?robot ?gripper ?part1))
            (gripper_empty ?robot ?gripper)
        )
    )

)