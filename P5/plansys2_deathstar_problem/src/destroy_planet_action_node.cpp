#include <iostream>
#include <memory>
#include <string>
#include<cstdlib>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DestroyPlanetAction : public plansys2::ActionExecutorClient
{
public:
  DestroyPlanetAction()
  : plansys2::ActionExecutorClient("destroy_planet", 2000ms)
  {
    progress_ = 0.0;
    first = true;
  }

private:
  void do_work()
  {
    std::srand(std::time(nullptr));
    if (first) {
      progress_ = 0.5;
      first = false;
      send_feedback(progress_, "Charging Laser");
      return;
    }
    shots++;
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Laser fired, number of shots " << shots <<
                std::flush;
    std::cout << std::endl;
    if (rand() % 2  == 0) {
      progress_ = 0.5;
      send_feedback(progress_, "Failed to destroy planet");
    } else {
      
      progress_ = 1.0;
      send_feedback(progress_, "Destroying Planet");
      std::cout << "\r\e[K" << std::flush;
      std::cout << "Destroying Planet ... [" <<
                 std::min(100.0, progress_ * 100 ) << "%]  " <<
                 std::flush;
      std::cout << std::endl;
      finish(true, 1.0, "Planet destroyed");
      progress_ = 0.0;
      shots = 0;
      first = true;
      return;
    }
    
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Destroying Planet ... [" <<
                 std::min(100.0, progress_ * 100 ) << "%]  " <<
                 std::flush;
  }

  double progress_;
  bool first;
  int shots;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DestroyPlanetAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "destroy_planet"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
