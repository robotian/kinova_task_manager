#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "status_interfaces/action/manipulator_task.hpp"

class ManipulatorTestClient : public rclcpp::Node {
public:
    using Manipulator = status_interfaces::action::ManipulatorTask;
    using GoalHandleManipulator = rclcpp_action::ClientGoalHandle<Manipulator>;

    explicit ManipulatorTestClient(const rclcpp::NodeOptions & options)
    : Node("manipulator_test_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<Manipulator>(
            this, "manipulator_action_server");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManipulatorTestClient::send_goal, this));
    }

    void send_goal() {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = Manipulator::Goal();
        goal_msg.arm_task = "START HARVEST"; // Test the harvest sequence

        RCLCPP_INFO(this->get_logger(), "Sending goal: %s", goal_msg.arm_task.c_str());

        auto send_goal_options = rclcpp_action::Client<Manipulator>::SendGoalOptions();
        
        // 1. Setup Feedback Callback
        send_goal_options.feedback_callback =
            std::bind(&ManipulatorTestClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        // 2. Setup Result Callback
        send_goal_options.result_callback =
            std::bind(&ManipulatorTestClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Manipulator>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Handle Feedback
    void feedback_callback(
        GoalHandleManipulator::SharedPtr,
        const std::shared_ptr<const Manipulator::Feedback> feedback) 
    {
        std::string status_name;
        switch (feedback->status) {
            case Manipulator::Feedback::IDLE:         status_name = "IDLE"; break;
            case Manipulator::Feedback::PLANNING:     status_name = "PLANNING"; break;
            case Manipulator::Feedback::MOVING:       status_name = "MOVING"; break;
            case Manipulator::Feedback::MOVING_COMPLETE: status_name = "MOVING_COMPLETE"; break;
            case Manipulator::Feedback::FAILED:       status_name = "FAILED"; break;
            default: status_name = "UNKNOWN"; break;
        }
        RCLCPP_INFO(this->get_logger(), "Feedback received: [Status: %s]", status_name.c_str());
    }

    // Handle Result
    void result_callback(const GoalHandleManipulator::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorTestClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    return 0;
}