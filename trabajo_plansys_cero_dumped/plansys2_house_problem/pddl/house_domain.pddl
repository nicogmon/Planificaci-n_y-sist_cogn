(define (domain house_domain)
(:requirements :strips :typing :durative-actions)

; Types
(:types
    robot
    door - object
    door_wp - waypoint
    bed - object
    trash - object
    table - object
    dishes - object
    person
)

; Predicates
(:predicates
    (robot_at ?r - robot ?w - waypoint)
    (connected ?w1 - waypoint ?w2 - waypoint)
    (door_at ?d - door ?w - waypoint)
    (door_closed ?d - door)
    (door_opened ?d - door)
    (object_at ?o - object ?w - waypoint)
    (person_at_door ?p - person ?w - waypoint ?d - door)
    (bed_tied ?b - bed)
    (table_setup ?t - table)
    (trash_pickup ?t - trash)
    (person_attended ?p - person ?dest - waypoint)
    (cleaned_dishes ?d - dishes)
)


; Actions
; move
; ask_open_door
; pass_door
; tie_bed
; dish_wash
; attend_person
; pick_up_trash
; setup_table

(:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_at ?r ?from))
        (over all (connected ?from ?to))
        )
    :effect (and
        (at end (robot_at ?r ?to))
        (at start (not(robot_at ?r ?from)))

    )
)

(:durative-action ask_open_door
    :parameters (?r - robot ?d - door ?w - door_wp)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (door_closed ?d)
        ))
        (over all (and 
            (robot_at ?r ?w)
            (door_at ?d ?w)
        ))
        
    )
    :effect (and 
        (at start (not(door_closed ?d)))
        (at end (door_opened ?d))
    )
    
)

(:durative-action pass_door
    :parameters (?r - robot ?d1 - door_wp ?d2 - door_wp ?d - door)
    :duration (= ?duration 1)
    :condition (and 
        (at start (robot_at ?r ?d1))

        (over all (and 
            (door_opened ?d)    
        ))

    )
    :effect (and 
        (at start (and 
            (not(robot_at ?r ?d1))
            
        ))
        (at end (and 
            (robot_at ?r ?d2)
        ))
    )
)

(:durative-action tie_bed
    :parameters (?r - robot ?w - waypoint ?b - bed)
    :duration (= ?duration 1)
    :condition (and 
        (over all (and 
            (object_at ?b ?w)
            (robot_at ?r ?w)
        ))
        
    )
    :effect (and 
        (at end (and 
            (bed_tied ?b)
        ))
    )
)

(:durative-action dish_wash
    :parameters (?r - robot ?w - waypoint ?d - dishes)
    :duration (= ?duration 1)
    :condition (and 

        (over all (and 
            (robot_at ?r ?w)
            (object_at ?d ?w)
        ))

    )
    :effect (and 
        (at end (and 
            (cleaned_dishes ?d)
        ))
    )
)

(:durative-action attend_person
    :parameters (?r - robot ?w - waypoint ?dest - waypoint ?p - person ?d - door)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and
            (robot_at ?r ?w)
            (person_at_door ?p ?w ?d) 
        ))
        (over all (connected ?w ?dest))
    )
    :effect (and 
        (at start (and 
            (not(person_at_door ?p ?w ?d))
            (not (robot_at ?r ?w))
        ))
        (at end (and 
            (person_attended ?p ?dest)
            (robot_at ?r ?dest)
        ))
    )
)

(:durative-action pick_up_trash
    :parameters (?r - robot ?w - waypoint ?t - trash)
    :duration (= ?duration 1)
    :condition (and 
        (over all (and 
            (robot_at ?r ?w)
            (object_at ?t ?w)
        ))
    )
    :effect (and 
        (at end (and 
            (trash_pickup ?t)
        ))
    )
)

(:durative-action setup_table
    :parameters (?r - robot ?w - waypoint ?t - table)
    :duration (= ?duration 1)
    :condition (and 
        (over all (and 
            (robot_at ?r ?w)
            (object_at ?t ?w)
        ))
    )
    :effect (and 
        (at end (and 
            (table_setup ?t)
        ))
    )
)

); End Domain
