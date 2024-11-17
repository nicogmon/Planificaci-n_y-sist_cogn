(define (domain rover)
 (:requirements  :typing :fluents :equality)
 (:types robot waypoint - object)

 (:functions 
    (battery ?r - robot)
 )
 
 (:predicates
 (robot_at ?r - robot ?wp - waypoint)
 (connected ?c1 ?c2 - waypoint)
 (visit ?wp - waypoint)
 (wp_charge ?wp -waypoint)
 (battery_empty ?r - robot)
 )



 (:action charge
 :parameters (?r - robot ?wp - waypoint)
 :precondition (and 
    (robot_at ?r ?wp)
    (wp_charge ?wp)
    (< (battery ?r) 1)
 )
 :effect (and
    (increase (battery ?r) 5)
 )
 )

 (:action move
 :parameters (?r - robot ?from ?to - waypoint)
 :precondition (and
 (connected ?from ?to)
 (robot_at ?r ?from)
 (not (battery_empty ?r))
 (> (battery ?r) 0)
 )
 :effect (and
 (not (robot_at ?r ?from))
 (robot_at ?r ?to)
 (visit ?from)
 (visit ?to)
 (decrease (battery ?r) 1)
 )
 )
)