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

#include <string>
#include <iostream>

#include "plansys2_house_problem/behavior_tree_nodes/PlaceGlasses.hpp"

#include "behaviortree_cpp/behavior_tree.h"

namespace plansys2_house_problem
{

PlaceGlasses::PlaceGlasses(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
PlaceGlasses::halt()
{
  std::cout << "PlaceGlasses halt" << std::endl;
}

BT::NodeStatus
PlaceGlasses::tick()
{
  std::cout << "PlaceGlasses tick " << counter_ << std::endl;

  if (counter_++ < 5) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_house_problem

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_house_problem::PlaceGlasses>("PlaceGlasses");
}
