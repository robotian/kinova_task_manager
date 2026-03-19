#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kinova_task_manager/pick_place_task.h>
#include <kinova_task_manager/move_to_config_task.h>
#include "status_interfaces/action/manipulator_task.hpp"
#include "kinova_task_manager/manipulator_commands.hpp"


using Manipulator = status_interfaces::action::ManipulatorTask;
using GoalHandleManipulator = rclcpp_action::ServerGoalHandle<Manipulator>;
using namespace std::placeholders;


class ManipulatorActionServer : public rclcpp::Node {
public:
    explicit ManipulatorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("manipulator_action_server", options) {
        
        RCLCPP_INFO(this->get_logger(), "Starting Manipulator Action Server...");
        

        this->action_server_ = rclcpp_action::create_server<Manipulator>(
            this,
            "manipulator_action_server",
            std::bind(&ManipulatorActionServer::handle_goal, this, _1, _2),
            std::bind(&ManipulatorActionServer::handle_cancel, this, _1),
            std::bind(&ManipulatorActionServer::handle_accepted, this, _1));
        
        // Initialize parameters
        // param_listener_ = std::make_shared<pick_place_task_demo::ParamListener>(this->shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Manipulator Action Server is ready.");
    }

    void init_parameters() {
        param_listener_ = std::make_shared<pick_place_task_demo::ParamListener>(this->shared_from_this());
    }

private:
    rclcpp_action::Server<Manipulator>::SharedPtr action_server_;
    std::shared_ptr<pick_place_task_demo::ParamListener> param_listener_;

    // 1. Handle incoming goal requests
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Manipulator::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 2. Handle cancellation
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 3. Handle execution
    void handle_accepted(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        // Execute in a separate thread to avoid blocking the executor
        std::thread{std::bind(&ManipulatorActionServer::doTask, this, _1), goal_handle}.detach();
    }


    /*
    void execute(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        auto result = std::make_shared<Manipulator::Result>();
        const auto params = param_listener_->get_params();

        // Setup Scene
        moveit_task_constructor_demo::setupDemoScene(params);

        // Construct Task
        moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
        
        if (!pick_place_task.init(this->shared_from_this(), params)) {
            RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        if (pick_place_task.plan(params.max_solutions)) {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded");
            
            if (pick_place_task.execute()) {
                RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                result->success = true;
                goal_handle->succeed(result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed");
                result->success = false;
                goal_handle->abort(result);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            result->success = false;
            goal_handle->abort(result);
        }
    }*/

    void doTask(const std::shared_ptr<GoalHandleManipulator> goal_handle){
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Manipulator::Result>();
        // auto feedback = std::make_shared<Manipulator::Feedback>(); // Create feedback object
        
        // Safety check: is ROS still running?
        if (!rclcpp::ok()) return;

        // Helper lambda to send status quickly
        // auto send_feedback = [&](std::string msg) {
        //     feedback->status = msg;
        //     goal_handle->publish_feedback(feedback);
        //     RCLCPP_INFO(LOGGER, "Feedback: %s", msg.c_str());
        // };

        // Convert string to enum
        kinova_task_manager::ManipulatorCommand cmd = kinova_task_manager::stringToCommand(goal->arm_task);
        RCLCPP_INFO(this->get_logger(), "Initializing Task: %s", goal->arm_task.c_str());

        
        std::string target_config = "empty";

        switch (cmd) {
            case kinova_task_manager::ManipulatorCommand::GO_STOW:
                target_config = "stow";
            case kinova_task_manager::ManipulatorCommand::GO_READY:
                target_config = "pre_cut_1";
            case kinova_task_manager::ManipulatorCommand::GO_DROP:
                target_config = "drop";
            {
                auto action_task = std::make_unique<moveit_task_constructor_demo::MoveToConfigTask>("move_to_config_task");

                if(!action_task->setTargetConfig(target_config)){
                    RCLCPP_ERROR(this->get_logger(), "The target config does not exist");
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                if (!action_task->init(this->shared_from_this(), param_listener_->get_params())) {
                    RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }
                if (action_task->plan(5)) {
                    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
                    
                    if (action_task->execute()) {
                        RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                        result->success = true;
                        goal_handle->succeed(result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Execution failed");
                        result->success = false;
                        goal_handle->abort(result);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed");
                    result->success = false;
                    goal_handle->abort(result);
                }
                break;
            }
            case kinova_task_manager::ManipulatorCommand::MOVE_EEF:
                break;

            case kinova_task_manager::ManipulatorCommand::START_HARVEST:
            {
                const auto params = param_listener_->get_params();
                // Setup Scene
                moveit_task_constructor_demo::setupDemoScene(params);

                // Construct Task
                moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
                
                if (!pick_place_task.init(this->shared_from_this(), params)) {
                    RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                if (pick_place_task.plan(params.max_solutions)) {
                    RCLCPP_INFO(this->get_logger(), "Planning succeeded");

                    // pick_place_task.execute();
                    // RCLCPP_INFO(this->get_logger(), "1st Run");

                    // pick_place_task.execute();
                    // RCLCPP_INFO(this->get_logger(), "2nd Run");
                    
                    if (pick_place_task.execute()) {
                        RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                        result->success = true;
                        goal_handle->succeed(result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Execution failed");
                        result->success = false;
                        goal_handle->abort(result);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed");
                    result->success = false;
                    goal_handle->abort(result);
                }
                break;
            }   

            case kinova_task_manager::ManipulatorCommand::UNKNOWN:
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown task received: %s", goal->arm_task.c_str());
                result->success = false;
                goal_handle->abort(result);
                return;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<ManipulatorActionServer>(node_options);

    node->init_parameters();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}