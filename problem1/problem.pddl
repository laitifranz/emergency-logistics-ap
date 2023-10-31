(define (problem problem1)
    (:domain domain1)

    ; --- OBJECTS ---
    (:objects
        ; as required, we have the depot where everything (a part from the injured people) is located
        depot location1 location2 location3 - location
        ; boxes to store the supplies
        box1 box2 box3 box4 - box
        ; food supplies for person1 and person2
        food1 food2  - food
        ; medicine supplies for person2
        medicine1 - medicine
        ; tools for person2
        tools1 - tools
        ; injured people
        per1 per2 per3 - person
        ; robot to deliver the supplies
        robot1 - robotic_agent
    )

    ; --- INITIAL STATE ---
    (:init
        ; initial locations of people
        (located_at per1 location1)
        (located_at per2 location2)
        (located_at per3 location3)
        ; depot is the initial location of the robot and the supplies
        (located_at robot1 depot)
        (located_at food1 depot)
        (located_at food2 depot)
        (located_at medicine1 depot)
        (located_at tools1 depot)
        (located_at box1 depot)
        (located_at box2 depot)
        ; initial state of objects
        (box_is_empty box1)
        (box_is_empty box2)
        (robot_has_no_box robot1)
        
        ; closed world assumptions
    )
    
    ; --- GOAL ---
    (:goal
        (and
            ; as specified in the assignment, it doesn't matter whether the food1 is delivered to person2 or person3 and the same for food2
            (or (and (delivered per2 food1) (delivered per3 food2)) (and (delivered per2 food2) (delivered per3 food1))) ; satisfy remark note of the assignment
            (delivered per1 tools1)
            (delivered per1 medicine1)
        )
    )
)
