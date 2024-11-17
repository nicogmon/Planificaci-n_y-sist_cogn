(define (problem recycling_example)
(:domain recycling_example)
; We define 3 different items, 2 types of trash bins, one table and a robot
(:objects
  table floor - location
  deposit - location
  bottle newspaper rotten_apple phone empty_can - item
  walle - robot
  chappie - robot
)

; Initially everything is on the floor and robot is by the table
(:init
  (robot_at walle table)
  (robot_at chappie table)
  (gripper_free walle)
  (gripper_free chappie)
  (item_at bottle floor)
  (item_at newspaper floor)
  (item_at rotten_apple floor)
  (item_at phone table)
  (item_at empty_can table)
)

; The goal is to clean the floor!
(:goal (and
    (forall (?it - item) (item_at ?it deposit)) 
    ; (item_at bottle table)
    ; (item_at rotten_apple organic_trashbin)
    ; (item_at newspaper paper_trashbin)
    ; (item_at phone table)
    ; (item_at empty_can organic_trashbin)
  )
)

)
