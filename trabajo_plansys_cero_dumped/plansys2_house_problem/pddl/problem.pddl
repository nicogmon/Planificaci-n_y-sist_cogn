(define (problem house_problem)
(:domain house_domain)
    (:objects
        princ_table -table
        dishes -dishes
        bed -bed
        trash -trash
        person1 -person
        kitchen bedroom livingroom  entrance gym - waypoint
        kobuki - robot
        principal_door - door
        beedroom_door - door
        bedroom_entrance1 bedroom_entrance2 - door_wp
        bedroom_exit1 bedroom_exit2 - door_wp
    )
    
    (:init
        (robot_at kobuki entrance)
        (door_at principal_door entrance)

        (object_at princ_table livingroom)
        (object_at dishes kitchen)
        (object_at bed bedroom)
        (object_at trash gym)
        (person_at_door person1 entrance principal_door)

        (connected gym bedroom_entrance1)
        (connected bedroom_exit2 gym )
        (connected kitchen bedroom_entrance1)
        (connected bedroom_exit2 kitchen)
        (connected livingroom bedroom_entrance1)
        (connected bedroom_exit2 livingroom)
        (connected entrance bedroom_entrance1)
        (connected bedroom_exit2 entrance)

        (connected bedroom_entrance2 bedroom)
        (connected  bedroom bedroom_exit1)

        (connected gym entrance)
        (connected gym livingroom)
        (connected gym kitchen)
        (connected entrance gym)
        (connected entrance livingroom)
        (connected entrance kitchen)
        (connected livingroom entrance)
        (connected livingroom gym)
        (connected livingroom kitchen)
        (connected kitchen entrance)
        (connected kitchen gym)
        (connected kitchen livingroom)



        (door_closed principal_door)
        (door_closed beedroom_door)
        (door_at beedroom_door bedroom_entrance1)
        (door_at beedroom_door bedroom_entrance2)
        (door_at beedroom_door bedroom_exit1)
        (door_at beedroom_door bedroom_exit2)
        (door_at principal_door entrance)

    )
    
    (:goal
    (and
        (bed_tied bed)
        (table_setup princ_table)
        (cleaned_dishes dishes)
        (trash_pickup trash)
        (person_attended person1 livingroom)
        (robot_at kobuki entrance)

    )
    )
) ; End Problem
