(define (problem recycling_gripper)
(:domain recycling_gripper)
; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor - location
  organic_trashbin - location
  paper_trashbin - location
  fridge - location
  shelving - location
  bottle newspaper rotten_apple book banana - item
  walle - robot
  eva - robot
  Wg1 Wg2 Eg1 Eg2 - gripper
 
)

; Initially everything is on the floor and robot is by the table
(:init
  (robot_at walle table)
  (robot_at eva fridge)
  (gripper_from walle Wg1)
  (gripper_from walle Wg2)
  (gripper_free walle Wg1)
  (gripper_free walle Wg2)
  (gripper_from eva Eg1)
  (gripper_from eva Eg2)
  (gripper_free eva Eg1)
  (gripper_free eva Eg2)
  (item_at bottle floor)
  (item_at newspaper fridge)
  (item_at rotten_apple paper_trashbin)
  (item_at book table)
  (item_at banana fridge)
)

; The goal is to clean the floor!
(:goal (and
    (item_at bottle table)
    (item_at rotten_apple organic_trashbin)
    (item_at newspaper table)
    (item_at book shelving)
    (item_at banana organic_trashbin)
  )
)

)
