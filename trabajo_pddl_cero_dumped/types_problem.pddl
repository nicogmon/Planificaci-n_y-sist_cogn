(define (problem recycling_example)
(:domain recycling_example)
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
)

; Initially everything is on the floor and robot is by the table
(:init
  (robot_at walle table)
  (robot_at eva floor)
  (gripper_free walle)
  (gripper_free eva)
  (item_at bottle floor)
  (item_at newspaper floor)
  (item_at rotten_apple floor)
  (item_at book table)
  (item_at banana fridge)
)

; The goal is to clean the floor!
(:goal (and
    (item_at bottle table)
    (item_at rotten_apple organic_trashbin)
    (item_at newspaper paper_trashbin)
    (item_at book shelving)
    (item_at banana organic_trashbin)
  )
)

)
