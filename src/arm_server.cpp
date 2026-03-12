#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>


#include "status_interfaces/action/manipulator_task.hpp"

class ArmActionServer : public rclcpp::Node {
public:
  using ArmTask = status_interfaces::action::ManipulatorTask;
  using GoalHandleArmTask = rclcpp_action::ServerGoalHandle<ArmTask>;

  explicit ArmActionServer() : Node("arm_task_server") {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ArmTask>(
      this, "arm_task_action",
      std::bind(&ArmActionServer::handle_goal, this, _1, _2),
      std::bind(&ArmActionServer::handle_cancel, this, _1),
      std::bind(&ArmActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ArmTask>::SharedPtr action_server_;

  // Boilerplate for goal handling
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmTask::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received request: %s", goal->arm_task.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleArmTask>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleArmTask> goal_handle) {
    std::thread{std::bind(&ArmActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleArmTask> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ArmTask::Result>();
    
    // Initialize MoveIt MoveGroupInterface for the Gen3 Lite
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");

    if (goal->arm_task == "case 1") {
      // Move to a specific Pose
      geometry_msgs::msg::Pose target_pose;
      target_pose.orientation.w = 1.0;
      target_pose.position.x = 0.3; // Values depend on your base_link
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.4;
      move_group.setPoseTarget(target_pose);
    } 
    else if (goal->arm_task == "case 2") {
      // Move to predefined "stow" state (defined in SRDF)
      move_group.setNamedTarget("stow");
    }

    auto move_result = move_group.move();
    result->success = (move_result == moveit::core::MoveItErrorCode::SUCCESS);
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmActionServer>());
  rclcpp::shutdown();
  return 0;
}