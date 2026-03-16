#include <memory>
#include <thread>
#include <atomic> // Added for shutdown flag
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
#include "status_interfaces/action/manipulator_task.hpp"
#include "kinova_task_manager/manipulator_commands.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif


static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_task_server");
namespace mtc = moveit::task_constructor;
using namespace std::placeholders;


using ArmTask = status_interfaces::action::ManipulatorTask;
using GoalHandleArmTask = rclcpp_action::ServerGoalHandle<ArmTask>;

// CRITICAL: Inherit from enable_shared_from_this
class ArmActionServer : public rclcpp::Node 
{
public:
  ArmActionServer(const rclcpp::NodeOptions& options) : 
    rclcpp::Node("arm_task_server", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) 
  {
    RCLCPP_INFO(LOGGER, "Starting Manipulator Action Server...");

    this->patchSrdf();
    
    this->action_server_ = rclcpp_action::create_server<ArmTask>(
      this, 
      "arm_task_action", 
      std::bind(&ArmActionServer::handle_goal, this, _1, _2),
      std::bind(&ArmActionServer::handle_cancel, this, _1),
      std::bind(&ArmActionServer::handle_accepted, this, _1)
    );
  }

  // Destructor to ensure thread is joined on shutdown
  ~ArmActionServer() {
    if (task_thread_.joinable()) {
      task_thread_.join();
    }
  }


  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return rclcpp::Node::get_node_base_interface();
  }
  

private: 
  rclcpp_action::Server<ArmTask>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Clean Termination Members
  std::thread task_thread_;
  mtc::Task task_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, 
    std::shared_ptr<const ArmTask::Goal> goal) 
  {
    RCLCPP_INFO(this->get_logger(), "Received request: %s", goal->arm_task.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleArmTask>) {
    RCLCPP_INFO(this->get_logger(), "Cancel requested");
    // If MTC task is running, you could call task_.stages()->clear() or similar here
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleArmTask> goal_handle) {
    // JOIN previous thread before starting a new one to prevent resource leaks
    if (task_thread_.joinable()) {
      task_thread_.join();
    }
    // No longer detached
    task_thread_ = std::thread(std::bind(&ArmActionServer::doTask, this, goal_handle));
  }

  mtc::Task createTask(){
    mtc::Task task;
    task.stages()->setName("New Task");
    task.loadRobotModel(shared_from_this());
    const auto& arm_group_name = "arm_0";
    const auto& hand_group_name = "arm_0_gripper";
    const auto& hand_frame = "arm_0_end_effector_link";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // --- Solvers ---
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);
     // --- 0. Current State ---
    mtc::Stage* current_state_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage.get();
        task.add(std::move(stage));
    }
    // --- 2. Move to Stow ---
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move to stow", sampling_planner);
        stage->setGroup(arm_group_name);
        stage->setGoal("stow"); // Must match <group_state name="ready"> in SRDF
        task.add(std::move(stage));
    }

    return task;
  }  

  mtc::Task createFakeHarvestingTask(){
    mtc::Task task;
    task.stages()->setName("Single Harvesting Motion");



  }

  void patchSrdf(){
    // ------------------------------------------------------------------------
    // SRDF PATCHING (Inject <end_effector> AND <group> if missing)
    // ------------------------------------------------------------------------
    std::string srdf_param_name = "robot_description_semantic";
    std::string srdf_string;
    bool srdf_needs_update = false;
    
    
    // 1. Get current SRDF
    if (this->has_parameter(srdf_param_name)) {
        srdf_string = this->get_parameter(srdf_param_name).as_string();
    } else {
        try {
            srdf_string = this->declare_parameter<std::string>(srdf_param_name, "");
        } catch(...) {}
    }

    // 2. Patch 1: Add End Effector if missing
    std::string eef_name = "manual_eef"; 
    if (!srdf_string.empty() && srdf_string.find("<end_effector") == std::string::npos) {
        std::string eef_patch = R"(<end_effector name="manual_eef" parent_link="arm_0_end_effector_link" group="arm_0_gripper"/>)";
        size_t pos = srdf_string.rfind("</robot>");
        if (pos != std::string::npos) {
            srdf_string.insert(pos, eef_patch);
            srdf_needs_update = true;
            RCLCPP_INFO(LOGGER, "Patched SRDF: Added manual_eef.");
        }
    }

    // 4. Apply Parameter Update if needed
    if (srdf_needs_update) {
        this->set_parameter(rclcpp::Parameter(srdf_param_name, srdf_string));
    }
  }

  mtc::Task createMoveEEFTask(std::string config) {
    mtc::Task task;
    task.stages()->setName("Move to Pose");

    std::string eef_name = "manual_eef"; 

    task.loadRobotModel(shared_from_this());

    const auto& arm_group_name = "arm_0";
    const auto& hand_group_name = "arm_0_gripper";
    const auto& hand_frame = "arm_0_end_effector_link";

    // Note: We keep "arm_0" as the default group for the task properties,
    // but because "manipulator_combined" now exists in the SRDF, 
    // execution will succeed when it needs to move both.
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("hand", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // --- NEW: Define Planners ---
    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    
    // --- NEW: 1. Current State Stage (The "Monitor") ---
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    mtc::Stage* current_state_ptr = current_state.get(); // Save pointer for monitoring
    task.add(std::move(current_state));

    // --- NEW: 2. Connect Stage (The "Bridge") ---
    // This allows MoveIt to plan a trajectory from Current State to the Target Pose
    auto connect = std::make_unique<mtc::stages::Connect>(
        "move to target",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, pipeline_planner}}
    );
    task.add(std::move(connect));

    // 3. Create the Pose Generator
    auto target_pose_stage = std::make_unique<mtc::stages::GeneratePose>("target pose");
    
    // --- NEW: Tell this stage to monitor the Current State ---
    target_pose_stage->setMonitoredStage(current_state_ptr); 

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "arm_0_base_link";
    target_pose.header.stamp = this->now();
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.3;
    target_pose.pose.position.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(-1.57, 1.57, 0.0);


    // geometry_msgs::msg::PoseStamped target_pose;
    // target_pose.header.frame_id = "base_link";
    // target_pose.header.stamp = this->now();
    // target_pose.pose.position.x = 0.0;
    // target_pose.pose.position.y = 0.0;
    // target_pose.pose.position.z = 0.8;

    // tf2::Quaternion q;
    // q.setRPY(0.0, 1.57, 0.0);


    target_pose.pose.orientation = tf2::toMsg(q);

    target_pose_stage->setPose(target_pose);

    // 4. Wrap it in a ComputeIK stage
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(target_pose_stage));
    wrapper->setMaxIKSolutions(30);
      wrapper->setMinSolutionDistance(0.1);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    wrapper->setEndEffector(eef_name);
    wrapper->setGroup(arm_group_name);
    wrapper->setIKFrame(hand_frame);
    
    task.add(std::move(wrapper));

    return task;
}

  mtc::Task createGoToConfigTask(std::string config){
    // To Do: need to check the config first 

    mtc::Task task;
    task.stages()->setName("Move to Config Task");
    task.loadRobotModel(shared_from_this());
    const auto& arm_group_name = "arm_0";
    const auto& hand_group_name = "arm_0_gripper";
    const auto& hand_frame = "arm_0_end_effector_link";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // --- Solvers ---
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    // INCREASE SPEED HERE (Values between 0.0 and 1.0)
    // 1.0 is the maximum speed defined in your joint_limits.yaml
    sampling_planner->setMaxVelocityScalingFactor(1.0);     // Set to max speed
    sampling_planner->setMaxAccelerationScalingFactor(1.0); // Set to max acceleration

    

    // --- 0. Current State ---
    mtc::Stage* current_state_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage.get();
        task.add(std::move(stage));
    }
    // --- 2. Move to Stow ---
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
        stage->setGroup(arm_group_name);
        stage->setGoal(config); // Must match <group_state name="ready"> in SRDF
        task.add(std::move(stage));
    }

    return task;
  }

  void doTask(const std::shared_ptr<GoalHandleArmTask> goal_handle){
    const auto goal = goal_handle->get_goal();
    auto act_result = std::make_shared<ArmTask::Result>();
    auto feedback = std::make_shared<ArmTask::Feedback>(); // Create feedback object
    
    // Safety check: is ROS still running?
    if (!rclcpp::ok()) return;

    // Helper lambda to send status quickly
    auto send_feedback = [&](std::string msg) {
        feedback->status = msg;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(LOGGER, "Feedback: %s", msg.c_str());
    };

    // Convert string to enum
    kinova_task_manager::ManipulatorCommand cmd = kinova_task_manager::stringToCommand(goal->arm_task);

    // Switch statement for cleaner logic
    switch (cmd) {
        case kinova_task_manager::ManipulatorCommand::GO_STOW:
            // RCLCPP_INFO(LOGGER, "Executing: GO STOW");
            send_feedback("Initializing Task: GO STOW");
            task_ = createGoToConfigTask("stow");
            break;

        case kinova_task_manager::ManipulatorCommand::GO_READY:
            send_feedback("Initializing Task: GO READY");
            // RCLCPP_INFO(LOGGER, "Executing: GO READY");
            task_ = createGoToConfigTask("pre_cut_1");
            break;

        case kinova_task_manager::ManipulatorCommand::MOVE_EEF:
            // RCLCPP_INFO(LOGGER, "Executing: MOVE EEF");
            send_feedback("Initializing Task: MOVE EEF");
            task_ = createMoveEEFTask("default_config");
            break;

        case kinova_task_manager::ManipulatorCommand::UNKNOWN:
        default:
            RCLCPP_ERROR(LOGGER, "Unknown task received: %s", goal->arm_task.c_str());
            act_result->success = false;
            goal_handle->abort(act_result);
            return;
    }

    try {
        send_feedback("Planning trajectory...");
        task_.init();
        if (!task_.plan(5)) {
            send_feedback("Planning failed!");
            act_result->success = false;
            goal_handle->abort(act_result);
            return;
        }

        send_feedback("Plan found. Executing movement...");
        task_.introspection().publishSolution(*task_.solutions().front());

        // Execution starts here
        auto result = task_.execute(*task_.solutions().front());

        if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            send_feedback("Movement completed successfully.");
            act_result->success = true;
            goal_handle->succeed(act_result);
        } else {
            send_feedback("Movement failed during execution.");
            act_result->success = false;
            goal_handle->abort(act_result);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception: %s", e.what());
        act_result->success = false;
        goal_handle->abort(act_result);
    }
  }

};





int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto arm_task_server = std::make_shared<ArmActionServer>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_task_server->getNodeBaseInterface());
  executor.spin();

  // Explicitly shutdown to unregister from the ROS graph
  RCLCPP_INFO(LOGGER, "Shutting down...");
  rclcpp::shutdown();
  return 0;
}