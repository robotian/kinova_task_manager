#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kinova_task_manager/pick_place_task.h>
#include <kinova_task_manager/move_to_config_task.h>
#include <kinova_task_manager/move_eef_task.h>
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
        // param_listener_ = std::make_shared<manipulator_action_server::ParamListener>(this->shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Manipulator Action Server is ready.");
    }

    void init_parameters() {
        param_listener_ = std::make_shared<manipulator_action_server::ParamListener>(this->shared_from_this());
    }

private:
    rclcpp_action::Server<Manipulator>::SharedPtr action_server_;
    std::shared_ptr<manipulator_action_server::ParamListener> param_listener_;

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
        
        // Safety check: is ROS still running?
        if (!rclcpp::ok()) return;

        // Convert string to enum
        kinova_task_manager::ManipulatorCommand cmd = kinova_task_manager::stringToCommand(goal->arm_task);
        RCLCPP_INFO(this->get_logger(), "Initializing Task: %s", goal->arm_task.c_str());
        
        
        std::string target_config = "";

        switch (cmd) {
            case kinova_task_manager::ManipulatorCommand::GO_STOW:
                target_config = "stow";
                break;
            case kinova_task_manager::ManipulatorCommand::GO_READY:
                target_config = "pre_cut_1";
                break;
            case kinova_task_manager::ManipulatorCommand::GO_DROP:
                target_config = "drop";
                break;
            default:
                break;
        }
        
        if (!target_config.empty()) {
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
            return;
        }



        switch (cmd) {            
            case kinova_task_manager::ManipulatorCommand::MOVE_EEF:
            {   
                moveit_task_constructor_demo::clearPlanningScene();
                
                auto action_task = std::make_unique<moveit_task_constructor_demo::MoveEefTask>("move_eef_task");

                if(!action_task->setTargetPose(goal->target_eef_pose)){
                    RCLCPP_ERROR(this->get_logger(), "The target pose cannot be reached");
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
            case kinova_task_manager::ManipulatorCommand::START_HARVEST:
            {
                const auto params = param_listener_->get_params();

                int num_of_actions = 3;
                bool isLastAction = false;

                for(int i=0; i<num_of_actions; i++){
                    // Setup Scene
                    moveit_task_constructor_demo::setupDemoScene(params);
    
                    // Construct Task
                    moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
                    if(i==num_of_actions-1){
                        isLastAction = true;
                    }else
                        isLastAction = false;
                    
                    if (!pick_place_task.init(this->shared_from_this(), params, isLastAction)) {
                        RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                        result->success = false;
                        goal_handle->abort(result);
                        return;
                    }
    
                    if (pick_place_task.plan(params.max_solutions)) {
                        RCLCPP_INFO(this->get_logger(), "Planning succeeded");
    
                        if (pick_place_task.execute()) {
                            RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                            // result->success = true;
                            // goal_handle->succeed(result);
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Execution failed");
                            result->success = false;
                            goal_handle->abort(result);
                            return;
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Planning failed");
                        result->success = false;
                        goal_handle->abort(result);
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "Harvesting actions number %d succeeded",i+1);
                }

                RCLCPP_INFO(this->get_logger(), "All Harvesting actions succeeded");
                result->success = true;
                goal_handle->succeed(result);
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