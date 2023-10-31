#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DetachCarrierFromRobot : public plansys2::ActionExecutorClient
{
public:
  DetachCarrierFromRobot()
  : plansys2::ActionExecutorClient("detach_carrier_from_robot", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Detach carrier to robot is running");
    } else {
      finish(true, 1.0, "Detach carrier to robot completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Detaching carrier to robot ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetachCarrierFromRobot>();

  node->set_parameter(rclcpp::Parameter("action_name", "detach_carrier_from_robot"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
