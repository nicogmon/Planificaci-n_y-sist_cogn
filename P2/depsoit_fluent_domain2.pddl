(define (domain deposit2)
(:requirements :strips :typing :fluents )


; Types definition
(:types
  location
  robot
  item
  gripper
  container
)
(:functions
    (container_max_capacity ?r - robot ?c - container)
    (container_current_load ?r - robot ?c - container)
    (weight ?it - item)


)
(:predicates
  (robot_at ?r - robot ?l - location)
  (item_at ?it - item ?l - location)
  (gripper_free ?r - robot ?g - gripper)
  (gripper_carry ?r - robot ?g - gripper ?it - item)
  (gripper_from  ?r - robot ?g - gripper)
  (container_not_full ?r - robot ?c - container)
  (object_in_container ?it - item ?r - robot ?c - container)
  (container_from ?r - robot ?c - container)
  
)

; Move action. The robot moves from one location (A) to another (B).
; The only precondition is that the robot must be in the initial location.
; Consequence: The robot is now at B and not at A.
(:action move
  :parameters (?r - robot ?from ?to - location ?g - gripper)
  :precondition
    (and 
      (gripper_free ?r ?g)
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
  :parameters (?it - item ?l - location ?r - robot ?g - gripper)
  :precondition 
    (and
      (item_at ?it ?l)
      (robot_at ?r ?l)
      (gripper_from  ?r ?g)
      (gripper_free ?r ?g)
    )
:effect
  (and
    (gripper_carry ?r ?g ?it)
    (not (item_at ?it ?l))
    (not (gripper_free ?r ?g)))
  )

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
;     - The item is now at the given location.
;     - The robot is no longer carrying the object and its gripper is free.
(:action drop
  :parameters (?it - item ?l - location ?r - robot ?g - gripper)
  :precondition
    (and 
      (robot_at ?r ?l)
      (gripper_from  ?r ?g)
      (gripper_carry ?r ?g ?it)
    )
  :effect 
    (and 
      (item_at ?it ?l)
      (gripper_free ?r ?g)
      (not (gripper_carry ?r ?g ?it))
    )
)

; Load action. The robot loads an object into a container.
; The robot must be carrying the object and the container must not be full.
; Consequences:
;     - The robot is no longer carrying the object.
;     - The container now has the object.
(:action load
  :parameters (?it - item  ?r - robot ?g - gripper ?c - container)
  :precondition
    (and 
      (gripper_carry ?r ?g ?it)
      (< (+ (container_current_load ?r ?c) (weight ?it)) (container_max_capacity ?r ?c))
    )
  :effect
    (and 
      (not (gripper_carry ?r ?g ?it))
      (gripper_free ?r ?g)
      (object_in_container ?it ?r ?c)
      (increase (container_current_load ?r ?c) (weight ?it))
    )
)

(:action unload
  :parameters (?it - item   ?r - robot ?g - gripper ?c - container ?l - location)
  :precondition
    (and 
      (container_from ?r ?c)
      (gripper_from  ?r ?g)
      (object_in_container ?it ?r ?c)
      (gripper_free ?r ?g)
    )
  :effect
    (and 
      (not (object_in_container ?it ?r ?c))
      (item_at ?it ?l)
      (decrease (container_current_load ?r ?c) (weight ?it))
      (gripper_free ?r ?g)
    )   
)

)