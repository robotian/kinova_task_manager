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


// --- Helpers
namespace {
    Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
        return Eigen::Translation3d(values[0], values[1], values[2]) *
               Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
    }
    
    geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
        return tf2::toMsg(vectorToEigen(values));
    };
} 

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

    this->eef_name = "manual_eef"; 
    this->arm_group_name = "arm_0";
    this->hand_group_name = "arm_0_gripper";
    this->hand_frame = "arm_0_end_effector_link";

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


  std::string eef_name = "manual_eef"; 
  std::string arm_group_name = "arm_0";
  std::string hand_group_name = "arm_0_gripper";
  std::string hand_frame = "arm_0_end_effector_link";


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

  void setupPlanningScene()
    {
      moveit_msgs::msg::CollisionObject object;
      object.id = "object";
      object.header.frame_id = "arm_0_base_link";
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      object.primitives[0].dimensions = { 0.05, 0.01 };
    
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.0;
      pose.position.y = 0.3;
      pose.position.z = 0.1;
      pose.orientation.w = 1.0;
      object.pose = pose;
    
      moveit::planning_interface::PlanningSceneInterface psi;
      psi.applyCollisionObject(object);
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
    // std::string eef_name = "manual_eef"; 
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

  mtc::Task createTask(){
    mtc::Task task;
    task.stages()->setName("New Task");
    task.loadRobotModel(shared_from_this());
    // const auto& arm_group_name = "arm_0";
    // const auto& hand_group_name = "arm_0_gripper";
    // const auto& hand_frame = "arm_0_end_effector_link";

    // Set task properties
    task.setProperty("group", this->arm_group_name);
    task.setProperty("eef", this->hand_group_name);
    task.setProperty("ik_frame", this->hand_frame);

    // --- Solvers ---
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
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
        stage->setGroup(this->arm_group_name);
        stage->setGoal("stow"); // Must match <group_state name="ready"> in SRDF
        task.add(std::move(stage));
    }

    return task;
  } 


  mtc::Task createFakeHarvestingTask() {
       const auto& object_id = "object";
      this->setupPlanningScene();
  // moveit::planning_interface::PlanningSceneInterface psi;

  // // 1. Define Object
  // shape_msgs::msg::SolidPrimitive cylinder_primitive;
  // cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // cylinder_primitive.dimensions = { 0.05, 0.01}; 

  // geometry_msgs::msg::Pose pick_pose;
  // pick_pose.position.x = 0.2; // Moved closer for Gen3 Lite reachability
  // pick_pose.position.y = 0.3; 
  // pick_pose.position.z = 0.1;
  // pick_pose.orientation.w = 1.0;

  // const auto& object_id = "object";
  // moveit_msgs::msg::CollisionObject object;
  // object.id = object_id;
  // object.header.frame_id = "arm_0_base_link";
  // object.primitives.push_back(cylinder_primitive);
  // object.primitive_poses.push_back(pick_pose);
  // object.operation = moveit_msgs::msg::CollisionObject::ADD;
  // psi.applyCollisionObject(object);

  mtc::Task task;
  task.stages()->setName("Single Harvesting Motion");
  task.loadRobotModel(shared_from_this());

  task.setProperty("group", this->arm_group_name);
  task.setProperty("eef", this->eef_name);
  task.setProperty("hand", this->hand_group_name);
  task.setProperty("ik_frame", this->hand_frame);

  // Planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  // --- STAGE 1: Current State ---
  mtc::Stage* current_state_ptr = nullptr;
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  // --- STAGE 2: Move to Ready ---
  auto move_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
  move_ready->setGroup(this->arm_group_name);
  move_ready->setGoal("pre_cut_1");
  task.add(std::move(move_ready));

  // --- STAGE 3: Open Hand ---
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  open_hand->setGroup(this->hand_group_name);
  open_hand->setGoal("open_default");
  auto* open_hand_ptr = open_hand.get(); // Capture for monitoring
  task.add(std::move(open_hand));

  // {
  //   auto allow_base_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("ignore_self_collisions");
  //   const auto& all_links = task.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

  //   for (size_t i = 0; i < all_links.size(); ++i) {
  //       for (size_t j = i + 1; j < all_links.size(); ++j) {
  //           const std::string& link_a = all_links[i];
  //           const std::string& link_b = all_links[j];

  //           // Define what we consider "Arm Links"
  //           bool is_arm_a = (link_a.find("arm_0") != std::string::npos);
  //           bool is_arm_b = (link_b.find("arm_0") != std::string::npos);

  //           // LOGIC: 
  //           // If both are arm links, skip (keep collision checking ON for safety).
  //           // If at least one is NOT an arm link (base, wheels, etc.), allow collision.
  //           if ((!is_arm_a || !is_arm_b)) {
  //               allow_base_collision->allowCollisions(link_a, link_b, true);
  //               RCLCPP_INFO(LOGGER, "Checking collision pair: %s <-> %s", link_a.c_str(), link_b.c_str());
  //           }
  //       }
  //   }

  //   allow_base_collision->allowCollisions(object_id, 
  //       task.getRobotModel()->getJointModelGroup(this->hand_group_name)->getLinkModelNames(), 
  //       true);
  //   task.add(std::move(allow_base_collision));
  //   // grasp->insert(std::move(allow_base_collision));
  // }

  // --- STAGE 4: Allow Collisions (CRITICAL: Must be BEFORE Connect) ---
  // {
  //   auto allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_grasp_collision");
  //   // Allow gripper to touch the object
  //   allow_collision->allowCollisions(object_id, 
  //       task.getRobotModel()->getJointModelGroup(this->hand_group_name)->getLinkModelNames(), 
  //       true);
  //   task.add(std::move(allow_collision));
  // }

  // --- STAGE 5: Connect (The Bridge) ---
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
        "bridge to approach",
        mtc::stages::Connect::GroupPlannerVector{{this->arm_group_name, sampling_planner}}
    );
    connect->setTimeout(15.0);
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(connect));      
  }

  // --- STAGE 6: Pick Container ---
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group","ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group","ik_frame" });

    {
      auto allow_base_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("ignore_self_collisions");
      const auto& all_links = task.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

      for (size_t i = 0; i < all_links.size(); ++i) {
          for (size_t j = i + 1; j < all_links.size(); ++j) {
              const std::string& link_a = all_links[i];
              const std::string& link_b = all_links[j];

              // Define what we consider "Arm Links"
              bool is_arm_a = (link_a.find("arm_0") != std::string::npos);
              bool is_arm_b = (link_b.find("arm_0") != std::string::npos);

              // LOGIC: 
              // If both are arm links, skip (keep collision checking ON for safety).
              // If at least one is NOT an arm link (base, wheels, etc.), allow collision.
              if (!(is_arm_a && is_arm_b)) {
                  allow_base_collision->allowCollisions(link_a, link_b, true);
                  RCLCPP_INFO(LOGGER, "Checking collision pair: %s <-> %s", link_a.c_str(), link_b.c_str());
              }
          }
      }

      allow_base_collision->allowCollisions(object_id, 
          task.getRobotModel()->getJointModelGroup(this->hand_group_name)->getLinkModelNames(), 
          true);
      // task.add(std::move(allow_base_collision));
      grasp->insert(std::move(allow_base_collision));
    }
    
    // 6a. Approach
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.01, 0.10); // Reduced slightly for precision
        
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = this->hand_frame;
        vec.vector.z = 0.05; // Approach along end-effector Z
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
    }

    // 6b. Generate Grasp Pose
    {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setPreGraspPose("open_default");
        stage->setObject(object_id);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(open_hand_ptr); // Monitor the hand-open state
        
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(20);
        wrapper->setMinSolutionDistance(0.1);
        
        // Transform: Adjust Z (0.14) so fingers wrap around object center
        std::vector<double> grasp_frame_transform = {0.0, 0.0, 0.14, 1.571, -1.571, 1.571};
        wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), this->hand_frame);
        
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
    }

    // 6c. Close Hand
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(this->hand_group_name);
        stage->setGoal("close");
        grasp->insert(std::move(stage));
    }

    // 6d. Attach Object
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        stage->attachObject(object_id, this->hand_frame);
        grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // --- STAGE 7: Lift Object ---
  {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->setGroup(this->arm_group_name);
      stage->setMinMaxDistance(0.05, 0.1);
      stage->setIKFrame(this->hand_frame);
      
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "arm_0_base_link";
      vec.vector.z = 0.1; // Lift up
      stage->setDirection(vec);
      task.add(std::move(stage));
  }

  return task;
}

/*
  mtc::Task createFakeHarvestingTask() {

    moveit::planning_interface::PlanningSceneInterface psi;

    // Define cylinder properties
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder_primitive.dimensions = { 0.05, 0.01}; 

    geometry_msgs::msg::Pose pick_pose;
    pick_pose.position.x = 0.0;
    pick_pose.position.y = 0.5;
    pick_pose.position.z = 0.1;
    pick_pose.orientation.w = 1.0;

    const auto& object_id = "object";
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = "arm_0_base_link";
    object.primitives.push_back(cylinder_primitive);
    object.primitive_poses.push_back(pick_pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObject(object);


    mtc::Task task;
    task.stages()->setName("Single Harvesting Motion");
    task.loadRobotModel(shared_from_this());

    // Set global task properties
    task.setProperty("group", this->arm_group_name);
    task.setProperty("eef", this->eef_name);
    task.setProperty("hand", this->hand_group_name);
    task.setProperty("ik_frame", this->hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    sampling_planner->setMaxVelocityScalingFactor(1.0);
    sampling_planner->setMaxAccelerationScalingFactor(1.0);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // 1. Current State
    mtc::Stage* current_state_ptr = nullptr;
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = current_state.get();
    task.add(std::move(current_state));

    

    // 2. Move to Ready (Joint Space)
    auto move_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
    move_ready->setGroup(this->arm_group_name);
    move_ready->setGoal("pre_cut_1");
    task.add(std::move(move_ready));

    // 3. Open Hand (Gripper Group)
    auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
    open_hand->setGroup(this->hand_group_name);
    open_hand->setGoal("open_default");
    task.add(std::move(open_hand));

    {
      auto allow_base_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("ignore_self_collisions");
      const auto& all_links = task.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

      for (size_t i = 0; i < all_links.size(); ++i) {
          for (size_t j = i + 1; j < all_links.size(); ++j) {
              const std::string& link_a = all_links[i];
              const std::string& link_b = all_links[j];

              // Define what we consider "Arm Links"
              bool is_arm_a = (link_a.find("arm_0") != std::string::npos);
              bool is_arm_b = (link_b.find("arm_0") != std::string::npos);

              // LOGIC: 
              // If both are arm links, skip (keep collision checking ON for safety).
              // If at least one is NOT an arm link (base, wheels, etc.), allow collision.
              if (!(is_arm_a || is_arm_b)) {
                  allow_base_collision->allowCollisions(link_a, link_b, true);
                  RCLCPP_INFO(LOGGER, "Checking collision pair: %s <-> %s", link_a.c_str(), link_b.c_str());
              }
          }
      }
      task.add(std::move(allow_base_collision));
      // grasp->insert(std::move(allow_base_collision));
    }

    // 3c. Allow Collisions
    {       
      RCLCPP_INFO(LOGGER,"allowing collision 2");
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions for the object");
      stage->allowCollisions(object_id, task.getRobotModel()->getLinkModelNamesWithCollisionGeometry(), true);
      task.add(std::move(stage));
    }

    


    // 4. Connect (The Bridge to the IK solution)
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
          "bridge to approach",
          mtc::stages::Connect::GroupPlannerVector{{this->arm_group_name, sampling_planner}}
      );
      connect->setTimeout(15.0);
      connect->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(connect));      
    }

    mtc::Stage* pick_stage_ptr = nullptr;
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "group","ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group","ik_frame" });
      
      // 3a. Approach
      {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
          stage->properties().set("marker_ns", "approach_object");

          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.01, 0.15);
          
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = this->hand_frame;
          vec.vector.z = 0.08; 
          stage->setDirection(vec);
          grasp->insert(std::move(stage));
      }

      // 3b. Generate Grasp Pose
      {
          auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          stage->setPreGraspPose("open_default");
          stage->setObject(object_id);
          stage->setAngleDelta(M_PI / 12);
          stage->setMonitoredStage(current_state_ptr);
          
          auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(20);
          wrapper->setMinSolutionDistance(0.1);
          
          // Offset the IK frame so the fingers don't bury themselves inside the cylinder center
          std::vector<double> grasp_frame_transform = {0.0, 0.0, 0.14, 1.571, -1.571, 1.571};
          wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), this->hand_frame);
          
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
          
          grasp->insert(std::move(wrapper));
      }

      // 3b. Generate Grasp Pose
      // {
      //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
      //   stage->properties().set("marker_ns", "grasp_pose");
      //   stage->setPreGraspPose("open_default");
      //   stage->setObject(object_id);
      //   stage->setAngleDelta(M_PI / 12);
      //   stage->setMonitoredStage(current_state_ptr);
        
      //   auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      //   wrapper->setMaxIKSolutions(40);
      //   wrapper->setMinSolutionDistance(0.1);
      //   std::vector<double> grasp_frame_transform = {0.0, 0.0, 0.14, 1.571, -1.571, 1.571};
      //   wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), this->hand_frame);

      //   // Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      //   // grasp_frame_transform.translation() << 0.0, 0.0, 0.14; // Adjust this based on your gripper length
      //   // wrapper->setIKFrame(grasp_frame_transform, this->hand_frame);
        
      //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
      //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      //   grasp->insert(std::move(wrapper));
      // }

      // {
      //   auto allow_base_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("ignore_self_collisions");
      //   const auto& all_links = task.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

      //   for (size_t i = 0; i < all_links.size(); ++i) {
      //       for (size_t j = i + 1; j < all_links.size(); ++j) {
      //           const std::string& link_a = all_links[i];
      //           const std::string& link_b = all_links[j];

      //           // Define what we consider "Arm Links"
      //           bool is_arm_a = (link_a.find("arm_0") != std::string::npos);
      //           bool is_arm_b = (link_b.find("arm_0") != std::string::npos);

      //           // LOGIC: 
      //           // If both are arm links, skip (keep collision checking ON for safety).
      //           // If at least one is NOT an arm link (base, wheels, etc.), allow collision.
      //           if (!(is_arm_a || is_arm_b)) {
      //               allow_base_collision->allowCollisions(link_a, link_b, true);
      //               RCLCPP_INFO(LOGGER, "Checking collision pair: %s <-> %s", link_a.c_str(), link_b.c_str());
      //           }
      //       }
      //   }
      //   grasp->insert(std::move(allow_base_collision));
      // }

      // // 3c. Allow Collisions
      // {       
      //   RCLCPP_INFO(LOGGER,"allowing collision 2");
      //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions for the object");
      //   stage->allowCollisions(object_id, task.getRobotModel()->getLinkModelNamesWithCollisionGeometry(), true);
      //   grasp->insert(std::move(stage));
      // }

      // 3d. Close Hand
      {
          auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
          stage->setGroup(this->hand_group_name);
          stage->setGoal("close");
          grasp->insert(std::move(stage));
      }

      // 3e. Attach Object
      {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
          stage->attachObject(object_id, this->hand_frame);
          grasp->insert(std::move(stage));
      }


      pick_stage_ptr = grasp.get();
      task.add(std::move(grasp));
    }


    return task;
  } 

*/
  /*
  mtc::Task createFakeHarvestingTask(){
    mtc::Task task;
    task.stages()->setName("Single Harvesting Motion");
    task.loadRobotModel(shared_from_this());

    task.setProperty("group", this->arm_group_name);
    task.setProperty("eef", this->eef_name);
    task.setProperty("hand", this->hand_group_name);
    task.setProperty("ik_frame", this->hand_frame);

    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

    // --- Solvers ---
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    sampling_planner->setMaxVelocityScalingFactor(1.0);     // Set to max speed
    sampling_planner->setMaxAccelerationScalingFactor(1.0); // Set to max acceleration

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);

    // --- NEW: 1. Current State Stage (The "Monitor") ---
    // auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    // mtc::Stage* current_state_ptr = current_state.get(); // Save pointer for monitoring
    // task.add(std::move(current_state));
    // --- 0. Current State ---
    mtc::Stage* current_state_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage.get();
        task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
      stage->setGroup(this->arm_group_name);
      stage->setGoal("ready"); // Must match <group_state name="ready"> in SRDF
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to precut1", sampling_planner);
      stage->setGroup(this->arm_group_name);
      stage->setGoal("pre_cut_1"); 
      task.add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(this->hand_group_name);
      stage->setGoal("open");
      task.add(std::move(stage));
    }
    {
      // This allows MoveIt to plan a trajectory from Current State to the Target Pose
      auto connect = std::make_unique<mtc::stages::Connect>(
          "move to target",
          mtc::stages::Connect::GroupPlannerVector{{this->arm_group_name, sampling_planner}}
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
      target_pose.pose.orientation = tf2::toMsg(q);

      target_pose_stage->setPose(target_pose);

      // 4. Wrap it in a ComputeIK stage
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(target_pose_stage));
      wrapper->setMaxIKSolutions(30);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->setEndEffector(this->eef_name);
      wrapper->setGroup(this->arm_group_name);
      wrapper->setIKFrame(this->hand_frame);
      
      task.add(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(this->hand_group_name);
      stage->setGoal("close");
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
      stage->setGroup(this->arm_group_name);
      stage->setGoal("ready"); // Must match <group_state name="ready"> in SRDF
      task.add(std::move(stage));
    }




    return task;
  }*/

  

  mtc::Task createMoveEEFTask(std::string config) {
    mtc::Task task;
    task.stages()->setName("Move to Pose");
    
    task.loadRobotModel(shared_from_this());
    
    // Note: We keep "arm_0" as the default group for the task properties,
    // but because "manipulator_combined" now exists in the SRDF, 
    // execution will succeed when it needs to move both.
    task.setProperty("group", this->arm_group_name);
    task.setProperty("eef", this->eef_name);
    task.setProperty("hand", this->hand_group_name);
    task.setProperty("ik_frame", this->hand_frame);

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
        mtc::stages::Connect::GroupPlannerVector{{this->arm_group_name, pipeline_planner}}
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
    target_pose.pose.orientation = tf2::toMsg(q);

    target_pose_stage->setPose(target_pose);

    // 4. Wrap it in a ComputeIK stage
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(target_pose_stage));
    wrapper->setMaxIKSolutions(30);
    wrapper->setMinSolutionDistance(0.1);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    wrapper->setEndEffector(this->eef_name);
    wrapper->setGroup(this->arm_group_name);
    wrapper->setIKFrame(this->hand_frame);
    
    task.add(std::move(wrapper));



    return task;
}

  mtc::Task createGoToConfigTask(std::string config){
    // To Do: need to check the config first 

    mtc::Task task;
    task.stages()->setName("Move to Config Task");
    task.loadRobotModel(shared_from_this());
    // const auto& arm_group_name = "arm_0";
    // const auto& hand_group_name = "arm_0_gripper";
    // const auto& hand_frame = "arm_0_end_effector_link";

    // Set task properties
    task.setProperty("group", this->arm_group_name);
    task.setProperty("eef", this->hand_group_name);
    task.setProperty("ik_frame", this->hand_frame);

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
        stage->setGroup(this->arm_group_name);
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

        case kinova_task_manager::ManipulatorCommand::START_HARVEST:
            send_feedback("Initializing Task: START HARVEST");
            task_ = createFakeHarvestingTask();
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
