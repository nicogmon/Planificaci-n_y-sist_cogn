(define (problem deposit_problem)
(:domain p5_domain)
; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor fridge shelving - location
  bottle newspaper rotten_apple book banana - item
  big_container - deposit
  walle - robot
  Wg1 - gripper
  Wc1 - container
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
  (= (container_max_capacity walle Wc1) 8)
  (= (container_current_load walle Wc1) 0)
  (= (weight bottle) 1)
  (= (weight newspaper) 1)
  (= (weight rotten_apple) 1)
  (= (weight book) 1)
  (= (weight banana) 1)
)

; The goal is to clean the floor!
(:goal (and
    (item_at bottle big_container)
    (item_at rotten_apple big_container)
    (item_at newspaper big_container )
    (item_at book big_container)
    (item_at banana big_container)
  )
)
;(:metric minimize (total_cost))
)
