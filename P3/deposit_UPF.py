from unified_planning.shortcuts import *

problem = Problem("deposit")

Location = UserType("Location")
Robot = UserType("Robot")
Item = UserType("Item")
Gripper = UserType("Gripper")
Container = UserType("Container")

container_max_capacity = problem.add_fluent("container_max_capacity", RealType(0, 100), c=Container, r=Robot)
container_current_capacity = problem.add_fluent("container_current_capacity", RealType(0, 100), c=Container, r=Robot)
weight = problem.add_fluent("weight", RealType(0, 100), i=Item)
distance = problem.add_fluent("distance", RealType(0, 100), l1=Location, l2=Location )

robot_at = problem.add_fluent("robot_at", BoolType(), r=Robot, l=Location, default_initial_value=False )
item_at = problem.add_fluent("item_at", BoolType(), it=Item,l=Location, default_initial_value=False )
gripper_free = problem.add_fluent("gripper_free", BoolType(), r=Robot, g=Gripper, default_initial_value=False )
gripper_carry = problem.add_fluent("gripper_carry", BoolType(), r=Robot, g=Gripper, it=Item, default_initial_value=False )
gripper_from = problem.add_fluent("gripper_from", BoolType(), r=Robot, g=Gripper, default_initial_value=False )
object_in_container = problem.add_fluent("object_in_container", BoolType(), it=Item, r=Robot, c=Container, default_initial_value=False )
container_from = problem.add_fluent("container_from", BoolType(), r=Robot, c=Container, default_initial_value=False )
connected = problem.add_fluent("connected", BoolType(), l1=Location, l2=Location, default_initial_value=True )




move = DurativeAction("move",  r=Robot, l_from=Location, l_to=Location, g=Gripper)
l_from = move.parameter("l_from")
l_to = move.parameter("l_to")
r = move.parameter("r")
g = move.parameter("g")

move.set_fixed_duration(distance(l_from, l_to))

move.add_condition(StartTiming(),robot_at(r, l_from))
move.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),gripper_free(r, g))
move.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),connected(l_from, l_to))
move.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),l_from != l_to)

move.add_effect(StartTiming(),robot_at(r, l_from), False)
move.add_effect(EndTiming(),robot_at(r, l_to), True)

pick = DurativeAction("pick", r=Robot, it=Item, l=Location, g=Gripper)
r= pick.parameter("r")
it = pick.parameter("it")
l = pick.parameter("l")
g = pick.parameter("g")

pick.set_fixed_duration(1)

pick.add_condition(StartTiming(),item_at(it, l))
pick.add_condition(StartTiming(),gripper_free(r, g))
pick.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),robot_at(r, l))
pick.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),gripper_from(r, g))

pick.add_effect(StartTiming(),item_at(it, l), False)
pick.add_effect(StartTiming(),gripper_free(r, g), False)
pick.add_effect(EndTiming(),gripper_carry(r, g, it), True)

load = DurativeAction("load",it=Item, r=Robot, g=Gripper, c=Container)
it = load.parameter("it")
r = load.parameter("r")
g = load.parameter("g")
c = load.parameter("c")

load.set_fixed_duration(2)

load.add_condition(StartTiming(),gripper_carry(r, g, it))
load.add_condition(StartTiming(),container_current_capacity(c, r) + weight(it) <= container_max_capacity(c, r))
load.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),container_from(r, c))
load.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),gripper_from(r, g))

load.add_effect(StartTiming(),gripper_carry(r, g, it), False)
load.add_effect(StartTiming(),gripper_free(r, g), True)
load.add_effect(EndTiming(),object_in_container(it, r, c), True)
load.add_effect(EndTiming(),container_current_capacity(c, r), (container_current_capacity(c, r) + weight(it)))

unload = DurativeAction("unload", it=Item, r=Robot, g=Gripper, c=Container, l=Location)
it = unload.parameter("it")
r = unload.parameter("r")
g = unload.parameter("g")
c = unload.parameter("c")
l = unload.parameter("l")
unload.set_fixed_duration(2)

unload.add_condition(StartTiming(),object_in_container(it, r, c))
unload.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),robot_at(r, l))
unload.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),gripper_from(r, g))
unload.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()),container_from(r, c))
unload.add_condition(StartTiming(),gripper_free(r, g))


unload.add_effect(StartTiming(),object_in_container(it, r, c), False)
unload.add_effect(StartTiming(),gripper_free(r, g), False)
unload.add_effect(StartTiming(),gripper_carry(r, g, it), True)

unload.add_effect(EndTiming(),container_current_capacity(c, r), (container_current_capacity(c, r) - weight(it)))
unload.add_effect(EndTiming(),item_at(it, l), True)
unload.add_effect(EndTiming(),gripper_carry(r,g,it), False)
unload.add_effect(EndTiming(),gripper_free(r, g), True)



table = Object("table", Location)
floor = Object("floor", Location)
fridge = Object("fridge", Location)
big_container = Object("big_container", Location)
bottle = Object("bottle", Item)
newspaper= Object("newspaper", Item)
book = Object("book", Item)


walle = Object("walle", Robot)
gripper = Object("gripper", Gripper)
container = Object("container", Container)

# problem.add_fluent(robot_at)
# problem.add_fluent(item_at)
# problem.add_fluent(gripper_free)
# problem.add_fluent(gripper_carry)
# problem.add_fluent(gripper_from)
# problem.add_fluent(object_in_container)
# problem.add_fluent(container_from)
# problem.add_fluent(container_max_capacity)
# problem.add_fluent(container_current_capacity)
# problem.add_fluent(weight)
# problem.add_fluent(distance)
# problem.add_fluent(connected)

problem.add_action(move)
problem.add_action(pick)
problem.add_action(load)
problem.add_action(unload)

problem.add_object(table)
problem.add_object(floor)
problem.add_object(fridge)
problem.add_object(big_container)
problem.add_object(bottle)
problem.add_object(newspaper)
problem.add_object(book)

problem.add_object(walle)
problem.add_object(gripper)
problem.add_object(container)

problem.set_initial_value(robot_at(walle, table), True)
problem.set_initial_value(gripper_from(walle, gripper), True)
problem.set_initial_value(gripper_free(walle, gripper), True)
problem.set_initial_value(container_from(walle, container), True)

problem.set_initial_value(item_at(bottle, floor), True)
problem.set_initial_value(item_at(newspaper, fridge), True)

problem.set_initial_value(item_at(book, table), True)

problem.set_initial_value(container_max_capacity(container, walle), 5)
problem.set_initial_value(container_current_capacity(container, walle), 0)

problem.set_initial_value(weight(bottle), 1)
problem.set_initial_value(weight(newspaper), 1)

problem.set_initial_value(weight(book), 1)


problem.set_initial_value(distance(big_container,big_container), 0)
problem.set_initial_value(distance(big_container, table), 20)
problem.set_initial_value(distance(table, big_container), 20)
problem.set_initial_value(distance(table, table), 0)
problem.set_initial_value(distance(big_container, floor), 20)
problem.set_initial_value(distance(big_container, fridge), 20)
problem.set_initial_value(distance(table, floor), 1)
problem.set_initial_value(distance(table, fridge), 1)
problem.set_initial_value(distance(floor, floor), 0)
problem.set_initial_value(distance(floor, fridge), 1)
problem.set_initial_value(distance(floor, table), 1)
problem.set_initial_value(distance(floor, big_container), 20)
problem.set_initial_value(distance(fridge, fridge), 0)
problem.set_initial_value(distance(fridge, table), 1)
problem.set_initial_value(distance(fridge, floor), 1)
problem.set_initial_value(distance(fridge, big_container), 20)

problem.set_initial_value(connected(big_container, table), True)
problem.set_initial_value(connected(table, big_container), True)
problem.set_initial_value(connected(big_container, floor), True)
problem.set_initial_value(connected(big_container, fridge), True)
problem.set_initial_value(connected(table, floor), True)
problem.set_initial_value(connected(table, fridge), True)
problem.set_initial_value(connected(floor, fridge), True)
problem.set_initial_value(connected(floor, table), True)
problem.set_initial_value(connected(floor, big_container), True)
problem.set_initial_value(connected(fridge, table), True)
problem.set_initial_value(connected(fridge, floor), True)
problem.set_initial_value(connected(fridge, big_container), True)


problem.add_goal(item_at(bottle, big_container))
problem.add_goal(item_at(newspaper, big_container))

problem.add_goal(item_at(book, big_container))


with OneshotPlanner(problem_kind=problem.kind) as planner:
 result = planner.solve(problem)
 if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
    print(f"{planner.name} found this plan: {result.plan}")
 else:
    print("No plan found.")
