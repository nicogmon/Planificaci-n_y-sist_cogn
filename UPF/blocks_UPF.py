# Import all the shortcuts, an handy way of using the unified_planning framework
from unified_planning.shortcuts import *
# Declaring types
object = UserType("Object")
Table = UserType("Table", father=object)
Block = UserType("Block", father=object)
# Creating problem ‘variables’
on = Fluent("on", BoolType(), x=object, y=object)
clear = Fluent("clear", BoolType(), x=object)

# Creating actions
move = InstantaneousAction("move", b=Block , y=Block, t=object, )
b = move.parameter("b")
y = move.parameter("y")
t = move.parameter("t")
move.add_precondition(clear(b))
move.add_precondition(clear(y))
move.add_precondition(on(b, t))
move.add_precondition(Not(Equals(b, y)))

move.add_effect(on(b, y), True)
move.add_effect(clear(t), True)
move.add_effect(on(b, t), False)
move.add_effect(clear(y), False)

move_to_table = InstantaneousAction("move_to_table", b=Block, y=Block, t=object)
b = move_to_table.parameter("b")
y = move_to_table.parameter("y")
t = move_to_table.parameter("t")

move_to_table.add_precondition(on(b, y))
move_to_table.add_precondition(clear(b))
move_to_table.add_precondition(Not(Equals(b,y)))

move_to_table.add_effect(on(b, y), False)
move_to_table.add_effect(clear(y), True)
move_to_table.add_effect(on(b, t), True)




# Declaring objects
table = Object("table", Table)
b1 = Object("b1", Block)
b2 = Object("b2", Block)
b3 = Object("b3", Block)
b4 = Object("b4", Block)


# Populating the problem with initial state and goals
problem = Problem("Blocks")

problem.add_action(move)
problem.add_object(table)
problem.add_object(b1)
problem.add_object(b2)
problem.add_object(b3)
problem.add_object(b4)
problem.set_initial_value(on(b1,table), True)
problem.set_initial_value(on(b3,table), True)
problem.set_initial_value(on(b2,b1), True)
problem.set_initial_value(on(b4,b3), True)



problem.add_goal(on(b1,table))
problem.add_goal(on(b2,b1))
problem.add_goal(on(b3,b2))
problem.add_goal(on(b4,b3))

with OneshotPlanner(problem_kind=problem.kind) as planner:
 result = planner.solve(problem)
 if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
    print(f"{planner.name} found this plan: {result.plan}")
 else:
    print("No plan found.")