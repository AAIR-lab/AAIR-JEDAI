(define (problem fow_problem) (
    :domain fow_domain)
    (:objects 
        l_housing r_housing - part
        l_upright r_upright - part
        l_ring_gear l_spur_gear_1 l_spur_gear_2 l_spur_gear_3 r_spur_gear_1 r_spur_gear_2 r_spur_gear_3  l_worm_gear r_worm_gear - part
        l_axle r_axle - part
        l_shaft_1 l_shaft_2 l_shaft_3 - part
        r_shaft_1 r_shaft_2 r_shaft_3 - part
        l_wheel r_wheel - part 
        main_base base - part
        yumi fetch - robot
        left right main - gripper

    )

    (:init
        (fits l_upright main_base)
        (fits r_upright main_base)
        (fits l_ring_gear l_upright)
        (fits l_housing l_ring_gear)
        (fits r_housing r_upright)
        (fits l_spur_gear_1 l_housing)
        (fits l_spur_gear_2 l_housing)
        (fits l_spur_gear_3 l_housing)
        (fits r_spur_gear_1 r_housing)
        (fits r_spur_gear_2 r_housing)
        (fits r_spur_gear_3 r_housing)
        (fits l_worm_gear l_housing)
        (fits r_worm_gear r_housing)
        (fits l_shaft_1 l_housing)
        (fits l_shaft_2 l_housing)
        (fits l_shaft_3 l_housing)
        (fits r_shaft_1 r_housing)
        (fits r_shaft_2 r_housing)
        (fits r_shaft_3 r_housing)
        (fits l_axle l_upright)
        (fits r_axle r_upright)
        (fits base main_base)
        (fits l_wheel l_axle)
        (fits r_wheel r_axle)

        (depends l_shaft_1 l_housing)
        (depends l_spur_gear_1 l_shaft_1)
        (depends l_shaft_2 l_spur_gear_1)
        (depends l_spur_gear_2 l_shaft_2)
        (depends l_shaft_3 l_spur_gear_2)
        (depends l_spur_gear_3 l_shaft_3)
        (depends l_worm_gear l_spur_gear_3)

        (depends r_shaft_1 r_housing)
        (depends r_spur_gear_1 r_shaft_1)
        (depends r_shaft_2 r_spur_gear_1)
        (depends r_spur_gear_2 r_shaft_2)
        (depends r_shaft_3 r_spur_gear_2)
        (depends r_spur_gear_3 r_shaft_3)
        (depends r_worm_gear r_spur_gear_3)

        (depends l_shaft_2 r_spur_gear_1)
        (depends r_shaft_2 l_spur_gear_2)
        (depends l_shaft_3 r_spur_gear_2)
        (depends r_shaft_3 l_spur_gear_2)


        (depends r_spur_gear_3 r_shaft_3)
        (depends base r_worm_gear)
        (depends base l_worm_gear)
        (depends l_wheel base)
        (depends r_wheel base)
        
        (depends l_housing l_axle)
        (depends r_housing r_axle)

        (depends l_ring_gear l_axle)

        (gripper_empty yumi left)
        (gripper_empty yumi right)
        (gripper_empty fetch main)
        (placed main_base)
        (human_adjusted main_base)

        (can_reach yumi left l_spur_gear_1)
        (can_reach yumi left l_spur_gear_2)
        (can_reach yumi left l_spur_gear_3)
        (can_reach yumi left l_wheel)
        (can_reach yumi left l_shaft_1)
        (can_reach yumi left l_shaft_2)
        (can_reach yumi left l_shaft_3)
        (can_reach yumi left l_axle)
        (can_reach yumi left l_wheel)

        (can_reach yumi right r_spur_gear_1)
        (can_reach yumi right r_spur_gear_2)
        (can_reach yumi right r_spur_gear_3)
        (can_reach yumi right r_wheel)
        (can_reach yumi right r_shaft_1)
        (can_reach yumi right r_shaft_2)
        (can_reach yumi right r_shaft_3)
        (can_reach yumi right r_axle)
        (can_reach yumi right r_wheel)

        (can_reach fetch main r_housing)
        (can_reach fetch main l_housing)
        (can_reach fetch main l_upright)
        (can_reach fetch main r_upright)
        (can_reach fetch main l_ring_gear)
        (can_reach fetch main l_worm_gear)
        (can_reach fetch main r_worm_gear)
        (can_reach fetch main base)
        

    )

    (:goal (and
        (human_adjusted l_housing)
        (human_adjusted r_housing)
        (human_adjusted l_upright)
        (human_adjusted r_upright)
        (human_adjusted l_ring_gear)
        (human_adjusted l_spur_gear_1)
        (human_adjusted l_spur_gear_2)
        (human_adjusted l_spur_gear_3)
        (human_adjusted r_spur_gear_1)
        (human_adjusted r_spur_gear_2)
        (human_adjusted r_spur_gear_3)
        (human_adjusted l_worm_gear)
        (human_adjusted r_worm_gear)
        (human_adjusted base)
        (human_adjusted r_wheel)
        (human_adjusted l_wheel)
    ))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
