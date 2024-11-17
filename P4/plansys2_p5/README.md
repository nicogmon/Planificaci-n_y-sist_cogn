# PlanSys2 Simple Example

## Description

This is a simple example that shows the basic operation of PlanSys2. A simple PDDL domain has been specified, and we will use the PlanSys2 Terminal to insert commands. Actions simulate their execution.

## How to run

In terminal 1:

```
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal

set instance walle robot
set instance table location
set instance floor location
set instance fridge location
set instance shelving location
set instance big_container location
set instance bottle item 
set instance newspaper item
set instance rotten_apple item
set instance book item
set instance banana item
set instance wg1 gripper
set instance wc1 container

get problem instances                               # Checks instances

set predicate (robot_at walle big_container)
set predicate (gripper_from walle wg1)
set predicate (gripper_free walle wg1)
set predicate (container_from walle wc1)
set predicate (item_at bottle floor)
set predicate (item_at newspaper table)
set predicate (item_at rotten_apple fridge)
set predicate (item_at book shelving)
set predicate (item_at banana fridge)
set predicate (container_not_full walle wc1)
set predicate (is_deposit  big_container)

set function (= (container_max_capacity walle wc1) 8)
set function (= (container_current_load walle wc1) 0)
set function (= (weight bottle) 1)
set function (= (weight newspaper) 1)
set function (= (weight rotten_apple) 1)
set function (= (weight book) 1)
set function (= (weight banana) 1)

get problem predicates                                # Checks predicates

set goal (and (item_at bottle big_container)(item_at rotten_apple big_container)(item_at newspaper big_container )(item_at book big_container)(item_at banana big_container))






set goal (and(robot_at leia bathroom))                # Sets the goal
get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
