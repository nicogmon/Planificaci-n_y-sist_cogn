(define (domain deposit_durative_extra)
(:requirements :strips :typing :fluents :durative-actions )


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
    (distance ?l1 - location ?l2 - location)
    


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
  (connected ?l1 - location ?l2 - location)
  
)



(:durative-action move
  :parameters (?r - robot ?from ?to - location ?g - gripper)
  :duration (= ?duration (distance ?from ?to))
  :condition
    (and 
      (at start (robot_at ?r ?from))
      ;(over all (gripper_free ?r ?g))  
      (over all (connected ?from ?to))
    )
  :effect
    (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
)
; Pick-up action. The robot picks an object at a location.
; Both the robot and the object must be in that location.
; The robot's gripper must be free.
; Consequences:
;     - The item is no longer at the given location.
;     - The robot is now carrying the object and its gripper is not free.
(:durative-action pick
  :parameters (?it - item ?l - location ?r - robot ?g - gripper)
  :duration (= ?duration 1)
  :condition 
    (and
      (at start (item_at ?it ?l))
      (at start (gripper_free ?r ?g)) 
      (over all (robot_at ?r ?l))
      (over all (gripper_from  ?r ?g))
      
    )
      
    
  :effect
    (and
      (at end(gripper_carry ?r ?g ?it))
      (at start(not (item_at ?it ?l)))
      (at start(not (gripper_free ?r ?g)))
    )
)
(:durative-action load
 :parameters (?it - item  ?r - robot ?g - gripper ?c - container)
 :duration (= ?duration 2)
 :condition
   (and 
      (at start(gripper_carry ?r ?g ?it))
      (at start(< (+ (container_current_load ?r ?c) (weight ?it)) (container_max_capacity ?r ?c)))
      (over all(container_from ?r ?c))
      (over all(gripper_from  ?r ?g))
 
   )
 :effect
   (and 
      (at start(not (gripper_carry ?r ?g ?it)))
      (at start(gripper_free ?r ?g))
      (at end(object_in_container ?it ?r ?c))
      (at end (increase (container_current_load ?r ?c) (weight ?it)))
   )
)
(:durative-action unload
:parameters (?it - item   ?r - robot ?g - gripper ?c - container ?l - location)
:duration (= ?duration 2)
:condition
  (and 
    (over all(container_from ?r ?c))
    (over all(gripper_from  ?r ?g))
    (at start(object_in_container ?it ?r ?c))
    (at start(gripper_free ?r ?g))
    (over all (robot_at ?r ?l))
  )
:effect
  (and 
    (at start(not (object_in_container ?it ?r ?c)))
    (at start(gripper_carry ?r ?g ?it))
    (at start(not (gripper_free ?r ?g)))
    (at end(decrease (container_current_load ?r ?c) (weight ?it)))
    (at end (gripper_free ?r ?g))
    (at end(not (gripper_carry ?r ?g ?it)))
    (at end (item_at ?it ?l))

  )   
)

)