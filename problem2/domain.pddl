(define (domain domain2)
    (:requirements :strips :typing :equality :negative-preconditions :disjunctive-preconditions :conditional-effects :existential-preconditions :universal-preconditions)

    ; --- TYPES ---
    (:types
        person robotic_agent location box carrier supply - object
        food medicine tools - supply
    )

    ; --- PREDICATES ---
    (:predicates
        (located_at ?x - (either robotic_agent box supply person carrier) ?l - location)

        (box_on_carrier ?b - box ?c - carrier)
        (box_loaded ?b - box)
        (box_is_empty ?b - box)
        (box_with_supply ?b - box ?s - supply)

        (carrier_has_no_robot ?c - carrier)
        (robot_has_no_carrier ?r - robotic_agent)
        (robot_carrier_attached ?r - robotic_agent ?c - carrier)

        ; since a carrier can load only up to 4 boxes, we create all the predicates
        ; because :numeric-fluents is not fully supported in many PDDL solvers (we tested using LAMA and other planners)
        (carrier_has_no_boxes ?c - carrier)
        (carrier_has_one_box ?c - carrier)
        (carrier_has_two_boxes ?c - carrier)
        (carrier_has_three_boxes ?c - carrier)
        (carrier_has_four_boxes ?c - carrier)

        (delivered ?p - person ?s - supply)
    )

    ; --- ACTIONS ---
    ; move the robot
    (:action move_robot
        :parameters (?r - robotic_agent ?l1 ?l2 - location)
        :precondition (and (located_at ?r ?l1) (not (= ?l1 ?l2)) (robot_has_no_carrier ?r))
        :effect (and
            (not (located_at ?r ?l1))
            (located_at ?r ?l2)
        )
    )

    ; move a robot with a carrier attached
    (:action move_robot_with_carrier
        :parameters (?r - robotic_agent ?c - carrier ?l1 ?l2 - location)
        :precondition (and
            (located_at ?r ?l1)
            (located_at ?c ?l1)
            (robot_carrier_attached ?r ?c)
            (not (= ?l1 ?l2))
        )
        :effect (and
            (not (located_at ?r ?l1))
            (located_at ?r ?l2)
            (not (located_at ?c ?l1))
            (located_at ?c ?l2)
        )
    )

    ; attach a carrier to a robot
    (:action attach_carrier_to_robot
        :parameters (?r - robotic_agent ?l1 - location ?c - carrier)
        :precondition (and (located_at ?r ?l1) (located_at ?c ?l1) (carrier_has_no_robot ?c) (robot_has_no_carrier ?r) (carrier_has_no_boxes ?c))
        :effect (and (not (carrier_has_no_robot ?c)) (not (robot_has_no_carrier ?r)) (robot_carrier_attached ?r ?c)) 
    )

    ; detach a carrier from a robot
    (:action detach_carrier_from_robot
        :parameters (?r - robotic_agent ?l1 - location ?c - carrier)
        :precondition (and (located_at ?r ?l1) (robot_carrier_attached ?r ?c) (carrier_has_no_boxes ?c))
        :effect (and (carrier_has_no_robot ?c) (robot_has_no_carrier ?r) (not (robot_carrier_attached ?r ?c)))
    )

    ; load the box on carrier under certain preconditions
    (:action load_box_on_carrier
        :parameters (?c - carrier ?b - box ?l - location ?r - robotic_agent)
        :precondition (and
            (located_at ?c ?l)
            (located_at ?b ?l)
            (located_at ?r ?l)
            (not (box_loaded ?b))
            (not (box_on_carrier ?b ?c))
            (or 
                (carrier_has_no_boxes ?c)
                (carrier_has_one_box ?c)
                (carrier_has_two_boxes ?c)
                (carrier_has_three_boxes ?c)) 
                ; if (carrier_has_four_boxes ?c) == TRUE, means we reach max of boxes to load, thus no more boxes can be loaded
        )
        :effect (and
            (box_on_carrier ?b ?c)
            (box_loaded ?b)
            (not (located_at ?b ?l))
            (when (carrier_has_no_boxes ?c) ; :conditional-effects required. when usage helps to simplify the understanding
                (and
                    (not (carrier_has_no_boxes ?c))
                    (carrier_has_one_box ?c)))
            (when (carrier_has_one_box ?c)
                (and
                    (not (carrier_has_one_box ?c))
                    (carrier_has_two_boxes ?c)))
            (when (carrier_has_two_boxes ?c)
                (and
                    (not (carrier_has_two_boxes ?c))
                    (carrier_has_three_boxes ?c)))
            (when (carrier_has_three_boxes ?c)
                (and
                    (not (carrier_has_three_boxes ?c))
                    (carrier_has_four_boxes ?c)))
        )
    )

    ; unload a box from the carrier
    (:action unload_box_from_carrier
        :parameters (?c - carrier ?b - box ?l - location ?r - robotic_agent)
        :precondition (and 
            (located_at ?c ?l)
            (located_at ?r ?l)
            (box_on_carrier ?b ?c)
            (box_loaded ?b)
            (or 
                (carrier_has_one_box ?c)
                (carrier_has_two_boxes ?c)
                (carrier_has_three_boxes ?c)
                (carrier_has_four_boxes ?c))
        )
        :effect (and
            (not (box_on_carrier ?b ?c))
            (not (box_loaded ?b))
            (located_at ?b ?l)
            (when (carrier_has_one_box ?c) ; same usage here
                (and
                    (not (carrier_has_one_box ?c))
                    (carrier_has_no_boxes ?c)))
            (when (carrier_has_two_boxes ?c)
                (and
                    (not (carrier_has_two_boxes ?c))
                    (carrier_has_one_box ?c)))
            (when (carrier_has_three_boxes ?c)
                (and
                    (not (carrier_has_three_boxes ?c))
                    (carrier_has_two_boxes ?c)))
            (when (carrier_has_four_boxes ?c)
                (and
                    (not (carrier_has_four_boxes ?c))
                    (carrier_has_three_boxes ?c)))
        )
    )

    ; fill the box with a supply. The box is required to not be on a carrier to be filled
    (:action fill_box
        :parameters (?r - robotic_agent ?l1 - location ?b - box ?s - supply)
        :precondition (and (located_at ?r ?l1) (located_at ?s ?l1) (box_is_empty ?b) (located_at ?b ?l1) (not (box_loaded ?b)))
        :effect (and (not (located_at ?s ?l1)) (box_with_supply ?b ?s) (not (box_is_empty ?b)))
    )

    ; deliver the supply to the person. The box is required to not be on a carrier to have its supply delivered
    (:action deliver
        :parameters (?r - robotic_agent ?l1 - location ?p - person ?s - supply ?b - box)
        :precondition (and (located_at ?r ?l1) (located_at ?p ?l1) (located_at ?b ?l1) (box_with_supply ?b ?s) (not (box_is_empty ?b)) (not (box_loaded ?b)))
        :effect (and (not (box_with_supply ?b ?s)) (delivered ?p ?s) (box_is_empty ?b))
    )

)