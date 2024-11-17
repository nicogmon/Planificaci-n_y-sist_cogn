(define (domain rover_np)
 (:requirements  :typing :fluents :equality :negative-preconditions)
 (:types robot waypoint - object)


 
 (:predicates
 (robot_at ?r - robot ?wp - waypoint)
 (connected ?c1 ?c2 - waypoint)
 (visit ?wp - waypoint)
 (battery_empty ?r - robot)
 )





 (:action move
 :parameters (?r - robot ?from ?to - waypoint)
 :precondition (and
 (connected ?from ?to)
 (robot_at ?r ?from)
 ;(not (battery_empty ?r))
 )
 :effect (and
 (not (robot_at ?r ?from))
 (robot_at ?r ?to)
 (visit ?from)
 (visit ?to)

 )
 )
)