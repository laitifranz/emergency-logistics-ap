(define (problem problem2)
    (:domain problem2)

    (:objects
        location0 location1 location2 location3 - location
        carrier1 - carrier
        box1 box2 box3 box4 - box
        robot1 - robotic_agent
        med1 - medicine
        food1 food2 - food
        tools1 - tools
        per1 per2 per3 - person
    )

    (:init
        (location_robot robot1 location0)
        (location_person per1 location1)
        (location_person per2 location2)
        (location_person per3 location3)
        (location_box box1 location0)
        (location_box box2 location0)
        (location_box box3 location0)
        (location_box box4 location0)
        (location_supply med1 location0)
        (location_supply food1 location0)
        (location_supply food2 location0)
        (location_supply tools1 location0)
        (location_carrier carrier1 location1)
        (carrier_has_no_robot carrier1)
        (robot_has_no_carrier robot1)
        (box_is_empty box1)
        (box_is_empty box2)
        (box_is_empty box3)
        (box_is_empty box4)
        (= (num-boxes carrier1) 0)
        ; closed world assumptions
    )
    
    (:goal
        (and
            (delivered per1 med1)
            (delivered per1 tools1)
            (or (and (delivered per2 food1) (delivered per3 food2))
                (and (delivered per2 food2) (delivered per3 food1))
            )
        )
    )
)