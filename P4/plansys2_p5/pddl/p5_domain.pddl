(define (domain p5_domain)
(:requirements :strips :typing :fluents :durative-actions :conditional-effects)


; Types definition
(:types
  deposit - location
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
  (object_in_container ?it - item ?r - robot ?c - container)
  (container_from ?r - robot ?c - container)
  
  
)



(:durative-action move
  :parameters (?r - robot ?from ?to - location ?g - gripper)
  :duration (= ?duration 10)
  :condition
    (and 
        (at start (robot_at ?r ?from))
        ;(over all (gripper_free ?r ?g))  
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

; Drop-off action. The robot drops an object at a location.
; The robot must be in that location and must be carrying that object.
; Consequences:
;     - The item is now at the given location.
;     - The robot is no longer carrying the object and its gripper is free.
; (:durative-action drop
; :parameters (?it - item ?l - location ?r - robot ?g - gripper)
; :duration (= ?duration 1)
; :condition
;   (and 
;     (at start (gripper_carry ?r ?g ?it))
;     (over all (robot_at ?r ?l))
;     (over all (gripper_from  ?r ?g))
;   )
; :effect 
;   (and    
;     (at start (gripper_free ?r ?g))
;     (at start (not (gripper_carry ?r ?g ?it)))
;     (at end (item_at ?it ?l))
;   )
; )


; Load action. The robot loads an object into a container.
; The robot must be carrying the object and the container must not be full.
; Consequences:
;     - The robot is no longer carrying the object.
;     - The container now has the object.
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
    (at end(not (gripper_carry ?r ?g ?it)))
    (at end(gripper_free ?r ?g))
    (at end(object_in_container ?it ?r ?c))
    (at end (increase (container_current_load ?r ?c) (weight ?it)))
  )
)

; Unload action. The robot unloads an object from a container.

(:durative-action unload
:parameters (?it - item   ?r - robot ?g - gripper ?c - container ?l - deposit)
:duration (= ?duration 2)
:condition
  (and 
    (over all(container_from ?r ?c))
    (over all(gripper_from  ?r ?g))
    (over all (robot_at ?r ?l))
    (at start(object_in_container ?it ?r ?c))
    (at start(gripper_free ?r ?g))
    
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

