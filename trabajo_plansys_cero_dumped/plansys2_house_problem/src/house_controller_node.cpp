// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <random>

class HouseController : public rclcpp::Node
{
public:
  HouseController()
  : rclcpp::Node("house_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();

    std::string goal = "(and(bed_tied princ_bed) (person_attended person1 livingroom) (table_setup princ_table) (cleaned_dishes dishes1) (trash_pickup trashbin) (robot_at kobuki entrance))";
    // "(and( (bed_tied princ_bed) (table_setup princ_table) (cleaned_dishes dishes1) (trash_pickup trashbin) (person_attended person1 livingroom) (robot_at kobuki entrance)))" (person_attended person1 livingroom)
    problem_expert_->setGoal(plansys2::Goal(goal));
    

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        return false;
    }

    // Execute the plan
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"kobuki", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"princ_table", "table"});
    problem_expert_->addInstance(plansys2::Instance{"dishes1", "dishes"});
    problem_expert_->addInstance(plansys2::Instance{"princ_bed", "bed"});
    problem_expert_->addInstance(plansys2::Instance{"trashbin", "trash"});
    problem_expert_->addInstance(plansys2::Instance{"person1", "person"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom","waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"livingroom", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"entrance", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"gym", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"principal_door", "door"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom_door", "door"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom_entrance1", "door_wp"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom_entrance2", "door_wp"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom_exit1", "door_wp"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom_exit2","door_wp"});
    problem_expert_->addInstance(plansys2::Instance{"kobuki", "robot"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at kobuki kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at principal_door entrance)"));

    problem_expert_->addPredicate(plansys2::Predicate("(object_at princ_table livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at dishes1 kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at princ_bed bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at trashbin gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at_door person1 entrance principal_door)"));
   
    problem_expert_->addPredicate(plansys2::Predicate("(connected gym bedroom_entrance1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom_exit2 gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen bedroom_entrance1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom_exit2 kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected livingroom bedroom_entrance1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom_exit2 livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance bedroom_entrance1)")); 
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom_exit2 entrance)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom_entrance2 bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom bedroom_exit1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected gym entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected gym livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected gym kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected livingroom entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected livingroom gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected livingroom kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen livingroom)"));

    problem_expert_->addPredicate(plansys2::Predicate("(door_closed principal_door)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed bedroom_door)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at bedroom_door bedroom_entrance1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at bedroom_door bedroom_entrance2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at bedroom_door bedroom_exit1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at bedroom_door bedroom_exit2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at principal_door entrance)"));



    

  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }



private:
  // typedef enum {STARTING, DESTROY } StateType;
  // StateType state_;
  // std::string planet;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HouseController>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok() ) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}