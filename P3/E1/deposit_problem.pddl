(define (problem deposit_problem_extra)
(:domain deposit_durative_extra)
; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor fridge shelving big_container - location
  bottle newspaper rotten_apple book banana - item

  walle - robot
  ;eva - robot
  Wg1 - gripper
  Wc1 - container
  ;Eg1 Eg2 - gripper
)

; Initially everything is on the floor and robot is by the table
(:init
  (robot_at walle big_container)
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
  (=(distance big_container table) 20)
  (=(distance big_container floor) 20)
  (=(distance big_container fridge) 20)
  (=(distance big_container shelving) 20)
  (=(distance table floor) 1)
  (=(distance table fridge) 1)
  (=(distance table shelving) 1)
  (=(distance table big_container) 20)
  (=(distance floor fridge) 1)
  (=(distance floor shelving) 1)
  (=(distance floor table) 1)
  (=(distance floor big_container) 20)
  (=(distance fridge shelving) 1)
  (=(distance fridge table) 1)
  (=(distance fridge floor) 1)
  (=(distance fridge big_container) 20)
  (=(distance shelving table) 1)
  (=(distance shelving floor) 1)
  (=(distance shelving fridge) 1)
  (=(distance shelving big_container) 20)
  (connected big_container table)
  (connected big_container floor)
  (connected big_container fridge)
  (connected big_container shelving)
  (connected table floor)
  (connected table fridge)
  (connected table shelving)
  (connected table big_container)
  (connected floor fridge)
  (connected floor shelving)
  (connected floor table)
  (connected floor big_container)
  (connected fridge shelving)
  (connected fridge table)
  (connected fridge floor)
  (connected fridge big_container)
  (connected shelving table)
  (connected shelving floor)
  (connected shelving fridge)
  (connected shelving big_container)

)

; The goal is to clean the floor!
(:goal (and
    (item_at bottle big_container)
    (item_at rotten_apple big_container)
    (item_at newspaper big_container)
    (item_at book big_container)
    (item_at banana big_container)
    ;(= (container_current_load walle) 0)

  )
)
;(:metric minimize (total_cost))
)