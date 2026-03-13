#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "status_interfaces/action/manipulator_task.hpp"

class ArmActionClient : public rclcpp::Node {
public:
  using ArmTask = status_interfaces::action::ManipulatorTask;

  explicit ArmActionClient() : Node("arm_task_client") {
    this->client_ptr_ = rclcpp_action::create_client<ArmTask>(this, "arm_task_action");
  }

  void send_goal(std::string task_name) {
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = ArmTask::Goal();
    goal_msg.arm_task = task_name;

    auto send_goal_options = rclcpp_action::Client<ArmTask>::SendGoalOptions();
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ArmTask>::SharedPtr client_ptr_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmActionClient>();
  node->send_goal("case 2"); // Change to "case 2" as needed
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}