(define (domain recycling_example)
(:requirements :strips :typing :negative-preconditions :universal-preconditions :existential-preconditions)


; Types definition
(:types
  location
  robot
  item
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (item_at ?it - item ?l - location)
  (gripper_free ?r - robot)
  (robot_carry ?r - robot ?it - item)
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location)
  :precondition
    (and 
      (robot_at ?r ?from)
    )
  :effect
    (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)

; Pick-up action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
;     - The item is no longer at the given location.
;     - The robot is now carrying the object and its gripper is not free.
(:action pick
  :parameters (?it - item ?l - location ?r - robot)
  :precondition 
    (and
      (item_at ?it ?l)
      (exists (?r - robot ?l - location) (robot_at ?r ?l)) ;exitencial conditions
      (robot_at ?r ?l)
      (gripper_free ?r)
    )
:effect
  (and
    (robot_carry ?r ?it)
    (not (item_at ?it ?l))
    (not (gripper_free ?r)))
  )

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
;     - The item is now at the given location.
;     - The robot is no longer carrying the object and its gripper is free.
(:action drop
:parameters (?it - item ?l - location ?r - robot)
:precondition
  (and 
    (robot_at ?r ?l)
    (robot_carry ?r ?it)
    (not (gripper_free ?r))
  )
:effect 
  (and 
    (item_at ?it ?l)
    (gripper_free ?r)
    (not (robot_carry ?r ?it))
  )
)

)
