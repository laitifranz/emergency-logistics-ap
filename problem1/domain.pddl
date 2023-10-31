(define (domain domain1)
    (:requirements :strips :typing :equality :negative-preconditions :disjunctive-preconditions)

    ; --- TYPES ---
    (:types
        person robotic_agent location box - object ; declare the types of the supplies
        supply - object
        food medicine tools - supply ; in case we want to add more supplies, we can just add them here
    )

    ; --- PREDICATES ---
    (:predicates
        (located_at ?x - (either robotic_agent box supply person) ?loc - location)
        
        (robot_has_box ?r - robotic_agent ?b - box)
        (robot_has_no_box ?r - robotic_agent)

        (box_with_supply ?b - box ?s - supply)
        (box_is_empty ?b - box) 

        (delivered ?p - person ?s - supply)
    )

    ; --- ACTIONS ---
    ; move robot to a location, if it is not already there
    (:action move_robot
        :parameters (?r - robotic_agent ?l1 ?l2 - location)
        :precondition (and (located_at ?r ?l1) (not (= ?l1 ?l2)))
        :effect (and (not (located_at ?r ?l1)) (located_at ?r ?l2))
    )

    ; pick up a box at the current location, if the robot has no box
    (:action take_box
        :parameters (?r - robotic_agent ?l1 - location ?b - box)
        :precondition (and (located_at ?r ?l1) (located_at ?b ?l1) (not (robot_has_box ?r ?b)) (robot_has_no_box ?r))
        :effect (and (not (located_at ?b ?l1)) (robot_has_box ?r ?b) (not (robot_has_no_box ?r)))
    )

    ; drop a box at the current location, not used; can be useful if the final goal requires everything to be back at the depot
    (:action drop_box
        :parameters (?r - robotic_agent ?l1 - location ?b - box)
        :precondition (and (located_at ?r ?l1) (robot_has_box ?r ?b) (not(robot_has_no_box ?r)))
        :effect (and (not (robot_has_box ?r ?b)) (located_at ?b ?l1) (robot_has_no_box ?r))
    )

    ; fill a box with a content, if the box is empty (box and agent are in the same location)
    (:action fill_box
        :parameters (?r - robotic_agent ?l1 - location ?b - box ?s - supply)
        :precondition (and (located_at ?r ?l1) (located_at ?s ?l1) (robot_has_box ?r ?b)  (box_is_empty ?b))
        :effect (and (not (located_at ?s ?l1)) (box_with_supply ?b ?s) (not (box_is_empty ?b)))
    )
    
    ; empty a box by leaving the content to the current location
    (:action empty_box ; not used action, here in the case we want to empty a box that is somehow filled with a supply we don't need to deliver anymore
        :parameters (?r - robotic_agent ?l1 - location ?b - box ?s - supply)
        :precondition (and (located_at ?r ?l1) (robot_has_box ?r ?b) (box_with_supply ?b ?s) (not (box_is_empty ?b)))
        :effect (and (not (box_with_supply ?b ?s)) (located_at ?s ?l1) (box_is_empty ?b))
    )
    
    ; if a supply is delivered, we remove the supply from the box without giving it any location, to consider it as consumed and not available for pick up anymore
    (:action deliver
        :parameters (?r - robotic_agent ?l1 - location ?p - person ?s - supply ?b - box)
        :precondition (and (located_at ?r ?l1) (located_at ?p ?l1) (robot_has_box ?r ?b) (box_with_supply ?b ?s) (not (box_is_empty ?b)))
        :effect (and (not (robot_has_box ?r ?b)) (not (box_with_supply ?b ?s)) (delivered ?p ?s) (located_at ?b ?l1) (box_is_empty ?b) (robot_has_no_box ?r))
    )

)