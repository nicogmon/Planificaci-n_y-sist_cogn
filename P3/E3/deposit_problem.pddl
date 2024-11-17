(define (problem deposit_problem_extra)
(:domain deposit_durative_extra)
; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor fridge shelving  - location
  bottle newspaper rotten_apple book banana - item

  walle - robot
  Wg1 - gripper
  Wc1 - container
  
)

; Initially everything is on the floor and robot is by the table
(:init
  (robot_at walle Big_Container)
  (gripper_from walle Wg1)
  (gripper_free walle Wg1)
  (container_from walle Wc1)
  (item_at bottle floor)
  (item_at newspaper fridge)
  (item_at rotten_apple table)
  (item_at book table)
  (item_at banana fridge)
  (= (container_max_capacity walle Wc1) 5)
  (= (container_current_load walle Wc1) 0)
  (= (weight bottle) 1)
  (= (weight newspaper) 1)
  (= (weight rotten_apple) 1)
  (= (weight book) 1)
  (= (weight banana) 1)
  (container_not_full walle Wc1)
  (=(distance Big_Container table) 20)
  (=(distance Big_Container floor) 20)
  (=(distance Big_Container fridge) 20)
  (=(distance Big_Container shelving) 20)
  (=(distance table floor) 1)
  (=(distance table fridge) 1)
  (=(distance table shelving) 1)
  (=(distance table Big_Container) 20)
  (=(distance floor fridge) 1)
  (=(distance floor shelving) 1)
  (=(distance floor table) 1)
  (=(distance floor Big_Container) 20)
  (=(distance fridge shelving) 1)
  (=(distance fridge table) 1)
  (=(distance fridge floor) 1)
  (=(distance fridge Big_Container) 20)
  (=(distance shelving table) 1)
  (=(distance shelving floor) 1)
  (=(distance shelving fridge) 1)
  (=(distance shelving Big_Container) 20)
  (connected Big_Container table)
  (connected Big_Container floor)
  (connected Big_Container fridge)
  (connected Big_Container shelving)
  (connected table floor)
  (connected table fridge)
  (connected table shelving)
  (connected table Big_Container)
  (connected floor fridge)
  (connected floor shelving)
  (connected floor table)
  (connected floor Big_Container)
  (connected fridge shelving)
  (connected fridge table)
  (connected fridge floor)
  (connected fridge Big_Container)
  (connected shelving table)
  (connected shelving floor)
  (connected shelving fridge)
  (connected shelving Big_Container)
  (not_container table)
  (not_container floor)
  (not_container fridge)
  (not_container shelving)

)

; The goal is to clean the floor!
(:goal (and
    (forall (?obj -item ) 
      (item_at ?obj Big_Container)
    )

  )
)
;(:metric minimize (total_cost))
)