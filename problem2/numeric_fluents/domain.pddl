(define (domain problem2)
    (:requirements :strips :typing :equality :negative-preconditions :disjunctive-preconditions :conditional-effects :existential-preconditions :universal-preconditions)
    (:types
        person robotic_agent location box carrier supply - object
        food medicine tools - supply
    )
    (:functions
        (num-boxes ?c - carrier)
    )
    (:predicates
        (location_robot ?r - robotic_agent ?l - location)
        (location_box ?b - box ?l - location)
        (location_carrier ?c - carrier ?l - location)
        (location_supply ?s - supply ?l - location)
        (location_person ?p - person ?l - location)
        (box_on_carrier ?b - box ?c - carrier)
        (box_loaded ?b - box)
        (box_is_empty ?b - box)
        (box_with_supply ?b - box ?s - supply)
        (carrier_has_no_robot ?c - carrier)
        (robot_has_no_carrier ?r - robotic_agent)
        (robot_carrier_attached ?r - robotic_agent ?c - carrier)
        (delivered ?p - person ?s - supply)
    )

    ; move the robot
    (:action move_robot
        :parameters (?r - robotic_agent ?l1 - location ?l2 - location)
        :precondition (and (location_robot ?r ?l1) (not (= ?l1 ?l2)) (robot_has_no_carrier ?r))
        :effect (and
            (not (location_robot ?r ?l1))
            (location_robot ?r ?l2)
        )
    )

    ; attach a carrier to a robot
    (:action attach_carrier_to_robot
        :parameters (?r - robotic_agent ?l1 - location ?c - carrier)
        :precondition (and (location_robot ?r ?l1) (location_carrier ?c ?l1) (carrier_has_no_robot ?c) (robot_has_no_carrier ?r))
        :effect (and (not (carrier_has_no_robot ?c)) (not (robot_has_no_carrier ?r)) (robot_carrier_attached ?r ?c))
    )
    ; move a robot with a carrier attached
    (:action move_robot_with_carrier
        :parameters (?r - robotic_agent ?l1 - location ?l2 - location ?c - carrier)
        :precondition (and (location_robot ?r ?l1) (location_carrier ?c ?l1) (not (= ?l1 ?l2)) (robot_carrier_attached ?r ?c))
        :effect (and
            (not (location_robot ?r ?l1))
            (location_robot ?r ?l2)
            (not (location_carrier ?c ?l1))
            (location_carrier ?c ?l2)
            ; move every box
            (forall (?b - box)
                (when (box_on_carrier ?b ?c)
                    (and
                        (not (location_box ?b ?l1))
                        (location_box ?b ?l2)
                    )
                     
                )
            )
        )
    )

    ; detatch a carrier from a robot
    (:action detatch_carrier_from_robot
        :parameters (?r - robotic_agent ?l1 - location ?c - carrier)
        :precondition (and (location_robot ?r ?l1) (robot_carrier_attached ?r ?c))
        :effect (and (carrier_has_no_robot ?c) (robot_has_no_carrier ?r)(not (robot_carrier_attached ?r ?c)))
    )

    ; assumption: a box can be loaded by a robot even if it is not attached to the carrier
    (:action load_box_on_carrier
        :parameters (?c - carrier ?b - box ?l - location ?r - robotic_agent)
        :precondition (and (location_carrier ?c ?l) (location_box ?b ?l) (location_robot ?r ?l) (not (box_loaded ?b)) (< (num-boxes ?c) 4))
        :effect (and (box_on_carrier ?b ?c) (box_loaded ?b) (increase (num-boxes ?c) 1))
    )

    ; unload a box from a carrier
    (:action unload_box_from_carrier
        :parameters (?c - carrier ?b - box ?l - location ?r - robotic_agent)
        :precondition (and (location_carrier ?c ?l) (location_box ?b ?l) (box_on_carrier ?b ?c) (box_loaded ?b) (location_robot ?r ?l))
        :effect (and (not (box_on_carrier ?b ?c)) (not (box_loaded ?b)) (decrease (num-boxes ?c) 1) (location_box ?b ?l))
    )

    (:action fill_box ; box is not required to be on the carrier to be filled
        :parameters (?r - robotic_agent ?l1 - location ?b - box ?s - supply)
        :precondition (and (location_robot ?r ?l1) (location_supply ?s ?l1) (box_is_empty ?b) (location_box ?b ?l1))
        :effect (and (not (location_supply ?s ?l1)) (box_with_supply ?b ?s) (not (box_is_empty ?b)))
    )

    (:action deliver ; box is required to not be on a carrier to have its supply delivered
        :parameters (?r - robotic_agent ?l1 - location ?p - person ?s - supply ?b - box)
        :precondition (and (location_robot ?r ?l1) (location_person ?p ?l1) (location_box ?b ?l1) (box_with_supply ?b ?s) (not (box_is_empty ?b)) (not (box_loaded ?b)))
        :effect (and (not (box_with_supply ?b ?s)) (delivered ?p ?s) (box_is_empty ?b) )
    )


)