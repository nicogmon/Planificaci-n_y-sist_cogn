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

class DeathStarController : public rclcpp::Node
{
public:
  DeathStarController()
  : rclcpp::Node("deathstar_controller"), state_(STARTING)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"star", "deathstar"});
    problem_expert_->addInstance(plansys2::Instance{"endor", "planet"});
    problem_expert_->addInstance(plansys2::Instance{"coruscant", "planet"});
    problem_expert_->addInstance(plansys2::Instance{"alderaan", "planet"});
    problem_expert_->addInstance(plansys2::Instance{"dantooine", "planet"});
    problem_expert_->addInstance(plansys2::Instance{"hoth", "planet"});
    problem_expert_->addInstance(plansys2::Instance{"yavin4", "planet"});

    problem_expert_->addPredicate(plansys2::Predicate("(connected coruscant endor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected endor coruscant)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected coruscant alderaan)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected alderaan coruscant)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected coruscant dantooine)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected dantooine coruscant)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected endor hoth)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected hoth endor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected endor yavin4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected yavin4 endor)"));

    problem_expert_->addPredicate(plansys2::Predicate("(star_at star coruscant)"));
  }

  bool step()
  {
    switch (state_) {
      case STARTING:
        {
          planet = get_next_planet();
          if (planet == "") {
            RCLCPP_INFO(get_logger(), "No more planets to destroy");
            return false;
            break;
          }
         
          
          std::string goal = "(and(destroyed " + planet + "))";
          
          problem_expert_->setGoal(plansys2::Goal(goal));
          

          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

              break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = DESTROY;
          }
          return true;
        }
        break;
      case DESTROY:
        {
          auto feedback = executor_client_->getFeedBack();
          for (const auto & action_feedback : feedback.action_execution_status) {
              std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;
          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              RCLCPP_INFO(get_logger(), "Planet %s destroyed", planet.c_str());
              
              state_ = STARTING;
            } 
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }
          }
          return true;
        }
        break;
      default:
        break;
    }
    return false;
  }

  std::vector<std::string> planets_ = std::vector<std::string> {"alderaan", "hoth", "yavin4", "dantooine", "endor", "coruscant"};
  std::string get_next_planet() {
    if (planets_.size() == 0) {
      return "";
    }
    // Shuffle the whole list
    std::shuffle(planets_.begin(), planets_.end(), std::mt19937{std::random_device{}()});
    // Select the last one and remove it
    std::string planet_selected = planets_.back();
    planets_.pop_back();
    return planet_selected;
}

private:
  typedef enum {STARTING, DESTROY } StateType;
  StateType state_;
  std::string planet;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeathStarController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok() && node->step()) {
    

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}