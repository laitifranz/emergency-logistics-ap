; -----------------------------------------------------------------------------------------
; an extra problem to see what the planner returns as plan with at disposition only 3 boxes
; -----------------------------------------------------------------------------------------

(define (problem problem2-extra)
    (:domain domain2)

    (:objects
        depot location1 location2 location3 - location
        carrier1 - carrier
        box1 box2 box3 - box
        robot1 - robotic_agent
        med1 - medicine
        food1 food2 - food
        tools1 - tools
        per1 per2 per3 - person
    )

    (:init
        ; initial locations of people
        (located_at per1 location1)
        (located_at per2 location2)
        (located_at per3 location3)
        ; depot is the initial location of the robot and the supplies
        (located_at robot1 depot)
        (located_at box1 depot)
        (located_at box2 depot)
        (located_at box3 depot)
        (located_at med1 depot)
        (located_at food1 depot)
        (located_at food2 depot)
        (located_at tools1 depot)
        (located_at carrier1 depot)
        ; initial state of objects
        (carrier_has_no_robot carrier1)
        (carrier_has_no_boxes carrier1) ; assumption that the carrier is initially empty
        (robot_has_no_carrier robot1)
        (box_is_empty box1)
        (box_is_empty box2)
        (box_is_empty box3)
        ; closed world assumptions
    )
    
    (:goal
        (and
            (delivered per1 med1)
            (delivered per1 tools1)
            (delivered per2 food1) 
            (delivered per3 food2)
        )
    )
)