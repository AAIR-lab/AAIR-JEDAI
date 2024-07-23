;move action works but with a hack for about_locations
(define (domain hanger)
  (:requirements :typing :strips :equality :probabilistic-effects :rewards)
  (:types
    agent
    location
    trajectory
    sensor
    issue
  )
  (:constants
    right_wing about_right_wing - location
    left_wing about_left_wing - location
    success fault_exists joint_fault_exists - issue
    recharge_station - location

  )

  (:predicates
    (hasBattery ?a - agent ?tr - trajectory)
    (at ?a - agent ?l - location)
    (initialized)
    (terminated)
    (inspecting ?a - agent ?l - location)
    (confirmed ?l - location)
    (fault_detected ?l - location)
    (available ?a - agent)
    (hasSensor ?a - agent ?s - sensor)
    (informed ?l - location)
    (pair_status ?loc1 ?loc2 - location ?i - issue)
    (hand_shaken ?a1 ?a2 - agent)
    (status ?l - location ?i - issue)
    (is_adjacent ?loc1 - location)
    (joint_inspected)
  )

  (:action move
    :parameters ( ?a - agent ?tr -trajectory ?startLoc ?finalLoc - location )
    :precondition (and
	(init)
        (available ?a)
        (at ?a ?startLoc)
        (valid ?a ?tr ?startLoc ?finalLoc)
        (hasBattery ?a ?tr)
        (not (is_adjacent ?finalLoc))

    )

    :effect (and

        (when (not (is_adjacent ?startLoc))(not (at ?a ?startLoc)))
        (when (= ?finalLoc left_wing)
            (probabilistic
                0.90 (and (at ?a ?finalLoc) (not (at ?a ?startLoc)))
                0.10 (at ?a about_left_wing)))
        (when (= ?finalLoc right_wing)
            (probabilistic
                0.90 (and (at ?a ?finalLoc) (not (at ?a ?startLoc)))
                0.10 (at ?a about_right_wing)))
        (when (= ?finalLoc recharge_station) (at ?a ?finalLoc))

    )
  )

  (:action recharge

    :parameters (?a - agent)
    :precondition (and
	(init)
        (at ?a recharge_station)
        (available ?a)

    )

    :effect (and

        (hasBattery ?a defaultTraj)

    )

  )

  (:action inspect_part1
    :parameters ( ?a - agent  ?s - sensor ?loc - location ?tr - trajectory )
    :precondition (and
	(init)
        (available ?a)
        (hasSensor ?a ?s)
        (valid ?a ?tr ?loc ?loc)
        (hasBattery ?a ?tr)
        (not (adjacent ?loc))
        (at ?a ?loc)

    )

    :effect (and
        (inspecting ?a ?loc)
        (not (available ?a))

    )

  )

  (:action inspect_part2
    :parameters( ?a - agent ?s - sensor ?loc - location ?tr - trajectory )
    :precondition (and
	(init)
        (not (available ?a))
        (at ?a ?loc)
        (valid ?a ?tr ?loc ?loc)
        (hasSensor ?a ?s)
        (hasBattery ?a ?tr)
        (not (adjacent ?loc))
        (inspecting ?a ?loc)
    )

    :effect (and

        (available ?a)
        (not (inspecting ?a ?loc))
        (when (status ?loc fault_exists) ( probabilistic 0.90 ( fault_detected ?loc)))
        (when (status ?loc joint_fault_exists) ( probabilistic 0.05 ( fault_detected ?loc)))
        (probabilistic 0.1 (and (when (= ?loc left_wing) (and (not (at ?a ?loc))(at ?a about_left_wing)))
                                (when (= ?loc right_wing)(and (not (at ?a ?loc))(at ?a about_right_wing)))))


    )


  )

  (:action alert
    :parameters (?a - agent ?loc - location)
    :precondition (and
	(init)
        (at ?a ?loc)
        (fault_detected ?loc)

    )

    :effect (and


        (informed ?loc)

    )


  )


  (:action human_confirm
    :parameters (?loc - location )
    :precondition (and
	(init)
        (informed ?loc)
        (not (confirmed ?loc))

    )

    :effect (and

        (confirmed ?loc)

    )

  )

  (:action handshake

   :parameters ( ?a1 ?a2 - agent )
   :precondition (and
	(init)
        (not (= ?a1 ?a2))
        (available ?a1)
        (available ?a2)
        (not (hand_shaken ?a1 ?a2))
        (not (hand_shaken ?a2 ?a1))

   )

    :effect
        (probabilistic 0.7 (and (hand_shaken ?a1 ?a2) (hand_shaken ?a2 ?a1)))

  )

  (:action joint_inspect
    :parameters ( ?a1 ?a2 - agent ?sense1 ?sense2 - sensor ?tr1 ?tr2 - trajectory ?loc1 ?loc2 - location  )
    :precondition (and
       	(init)
        (available ?a1)
        (hasSensor ?a1 ?sense1)
        (at ?a1 ?loc1)
        (valid ?a1 ?tr1 ?loc1 ?loc1)
        (available ?a2)
        (hasSensor ?a2 ?sense2)
        (at ?a2 ?loc2)
        (valid ?a2 ?tr2 ?loc2 ?loc2)
        (hand_shaken ?a1 ?a2)
        (hand_shaken ?a2 ?a1)
        (hasBattery ?a1 ?tr1)
        (hasBattery ?a2 ?tr2)
        (not (is_adjacent ?loc1))
        (not (is_adjacent ?loc2))
    )

    :effect (and

        ;if single fault is present
        (when (and (= ?loc1 ?loc2) (status ?loc1 fault_exists)) (probabilistic 0.99 (fault_detected ?loc1)))
        ;if joint fault is located at single location
        (when (and (= ?loc1 ?loc2) (status ?loc1 joint_fault_exists))(probabilistic 0.9 (fault_detected ?loc1)))
        ;if joint fault is located at two different location
        (when (or (pair_status ?loc1 ?loc2 joint_fault_exists)(pair_status ?loc2 ?loc1 joint_fault_exists))
            (probabilistic 0.9 (and (fault_detected ?loc1)(fault_detected ?loc2))))
        (not (hand_shaken ?a1 ?a2))
        (not (hand_shaken ?a2 ?a1))
        (probabilistic 0.1 (and (when (= ?loc1 left_wing) (and (not (at ?a1 ?loc1)) (at ?a1 about_left_wing)))

                                (when (= ?loc1 right_wing)(and (not (at ?a1 ?loc1)) (at ?a1 about_right_wing)))))
        (probabilistic 0.1 (and (when (= ?loc2 left_wing) (and (not (at ?a2 ?loc1)) (at ?a2 about_left_wing)))
                                (when (= ?loc2 right_wing)(and (not (at ?a2 ?loc1)) (at ?a2 about_right_wing)))))

    )


  )


  (:action done
  :precondition (and (fault_detected left_wing)(fault_detected right_wing))
  :effect (terminated)

  )

    (:action init
        :precondition (not (init))
        :effect (and
            (init)
            (hasBattery a1 defaultTraj)
            (valid a1 defaultTraj left_wing left_wing)
            (valid a1 defaultTraj right_wing right_wing)
            (valid a1 defaultTraj recharge_station recharge_station)
            (valid a1 defaultTraj recharge_station left_wing)
            (valid a1 defaultTraj right_wing left_wing)
            (valid a1 defaultTraj recharge_station right_wing)
            (valid a1 defaultTraj left_wing right_wing)
            (valid a1 defaultTraj left_wing recharge_station)
            (valid a1 defaultTraj right_wing recharge_station)
            (valid a1 defaultTraj about_left_wing left_wing)
            (valid a1 defaultTraj about_right_wing right_wing)
            (is_adjacent about_left_wing)
            (is_adjacent about_right_wing)
            (hasSensor a1 camera)

        )
    )
)


