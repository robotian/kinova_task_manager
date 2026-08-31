
#include <Eigen/Geometry>
#include <kinova_task_manager/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <cmath>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

// Time allowed for a planning-scene change to propagate to move_group before
// we plan or execute against it.
static constexpr std::chrono::milliseconds SCENE_SYNC_DELAY{ 200 };

// Fallback when a caller passes 0.0: matches the single-candidate ("near
// row") default used by manipulator_action_server's grasp_angle_delta_*
// parameters, since GenerateGraspPose::init() throws on an exact-zero delta.
static constexpr double kDefaultGraspAngleDelta = 2.0 * M_PI;

namespace {
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
    return Eigen::Translation3d(values[0], values[1], values[2]) *
           Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
    return tf2::toMsg(vectorToEigen(values));
}
}  // namespace

namespace moveit_task_constructor_demo {

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object) {
    if (!psi.applyCollisionObject(object))
        throw std::runtime_error("Failed to spawn object: " + object.id);
}

void removeAllCollisionObjects(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<std::string> existing_ids = psi.getKnownObjectNames();
    if (existing_ids.empty())
        return;

    std::vector<moveit_msgs::msg::CollisionObject> remove_objects;
    remove_objects.reserve(existing_ids.size());
    for (const auto& id : existing_ids) {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = id;
        obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        remove_objects.push_back(obj);
    }
    if (!psi.applyCollisionObjects(remove_objects))
        throw std::runtime_error("Failed to clear planning scene");
}

// Detach EVERY object still attached to a robot link and drop it from the world.
void detachAllLeftoverObjects(moveit::planning_interface::PlanningSceneInterface& psi) {
    auto attached = psi.getAttachedObjects();
    if (attached.empty())
        return;

    for (const auto& entry : attached) {
        const std::string& id = entry.first;
        const std::string& link = entry.second.link_name;

        RCLCPP_WARN(LOGGER, "Detaching leftover object '%s' from link '%s'",
                    id.c_str(), link.c_str());

        moveit_msgs::msg::AttachedCollisionObject detach;
        detach.object.id = id;
        detach.link_name = link;
        detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!psi.applyAttachedCollisionObject(detach))
            RCLCPP_ERROR(LOGGER, "Failed to detach leftover object '%s'", id.c_str());
    }
    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}


moveit_msgs::msg::CollisionObject createTable(const manipulator_action_server::Params& params) {
    geometry_msgs::msg::Pose pose = vectorToPose(params.table_pose);
    moveit_msgs::msg::CollisionObject object;
    object.id = params.table_name;
    object.header.frame_id = params.table_reference_frame;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { params.table_dimensions.at(0), params.table_dimensions.at(1),
                                         params.table_dimensions.at(2) };
    pose.position.z = -0.12;
    object.primitive_poses.push_back(pose);
    return object;
}

moveit_msgs::msg::CollisionObject createObject(const manipulator_action_server::Params& params) {
    geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
    moveit_msgs::msg::CollisionObject object;
    object.id = params.object_name;
    object.header.frame_id = params.object_reference_frame;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };
    pose.position.z += 0.5 * params.object_dimensions[0];
    object.primitive_poses.push_back(pose);
    return object;
}

void setupDemoScene(const manipulator_action_server::Params& params) {
    RCLCPP_INFO(LOGGER, "Setting the planning scene");
    moveit::planning_interface::PlanningSceneInterface psi;

    detachAllLeftoverObjects(psi);
    removeAllCollisionObjects(psi);
    rclcpp::sleep_for(SCENE_SYNC_DELAY);

    if (params.spawn_table)
        spawnObject(psi, createTable(params));
    spawnObject(psi, createObject(params));
    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

moveit_msgs::msg::CollisionObject createGridObject(const manipulator_action_server::Params& params,
                                                   const geometry_msgs::msg::Pose& pose_in,
                                                   const std::string& id,
                                                   double radius) {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header.frame_id = params.object_reference_frame;

    // Object frame = the BASE of the cylinder (grasps generate around this)
    object.pose = pose_in;

    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { params.object_dimensions.at(0), radius };

    geometry_msgs::msg::Pose rel;
    rel.orientation.w = 1.0;
    rel.position.z = 0.5 * params.object_dimensions.at(0);
    object.primitive_poses.push_back(rel);

    return object;
}

void setupHarvestGridScene(const manipulator_action_server::Params& params,
                           const std::vector<geometry_msgs::msg::Pose>& object_poses,
                           const std::vector<std::string>& object_ids,
                           const std::vector<double>& object_radii) {
    RCLCPP_INFO(LOGGER, "Setting the planning scene with %zu harvest grid object(s)",
                object_poses.size());
    moveit::planning_interface::PlanningSceneInterface psi;

    detachAllLeftoverObjects(psi);
    removeAllCollisionObjects(psi);
    rclcpp::sleep_for(SCENE_SYNC_DELAY);

    if (params.spawn_table)
        spawnObject(psi, createTable(params));

    for (std::size_t i = 0; i < object_poses.size(); ++i) {
        spawnObject(psi, createGridObject(params, object_poses.at(i), object_ids.at(i),
                                          object_radii.at(i)));
    }
    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

void setupSingleTargetScene(const manipulator_action_server::Params& params,
                            const geometry_msgs::msg::Pose& object_pose,
                            const std::string& object_id,
                            double radius) {
    RCLCPP_INFO(LOGGER, "Setting the planning scene with single target '%s'", object_id.c_str());
    moveit::planning_interface::PlanningSceneInterface psi;

    detachAllLeftoverObjects(psi);
    removeAllCollisionObjects(psi);
    rclcpp::sleep_for(SCENE_SYNC_DELAY);

    if (params.spawn_table)
        spawnObject(psi, createTable(params));
    spawnObject(psi, createGridObject(params, object_pose, object_id, radius));
    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

void removeHarvestedObject(const std::string& object_id) {
    RCLCPP_INFO(LOGGER, "Removing harvested object '%s' from planning scene", object_id.c_str());

    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = object_id;
    remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit::planning_interface::PlanningSceneInterface psi;
    if (!psi.applyCollisionObject(remove_object))
        throw std::runtime_error("Failed to remove harvested object: " + object_id);
}

void clearPlanningScene() {
    RCLCPP_INFO(LOGGER, "Clearing the planning scene");
    moveit::planning_interface::PlanningSceneInterface psi;

    detachAllLeftoverObjects(psi);
    removeAllCollisionObjects(psi);
    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

bool isObjectAttached(const std::string& object_id) {
    moveit::planning_interface::PlanningSceneInterface psi;
    auto attached = psi.getAttachedObjects({ object_id });
    return attached.count(object_id) > 0;
}

void detachAndRemoveObject(const std::string& object_id) {
    moveit::planning_interface::PlanningSceneInterface psi;

    auto attached = psi.getAttachedObjects({ object_id });
    if (attached.count(object_id) > 0) {
        RCLCPP_WARN(LOGGER, "Detaching object '%s' from link '%s'",
                    object_id.c_str(), attached[object_id].link_name.c_str());

        moveit_msgs::msg::AttachedCollisionObject detach;
        detach.object.id = object_id;
        detach.link_name = attached[object_id].link_name;
        detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!psi.applyAttachedCollisionObject(detach))
            RCLCPP_ERROR(LOGGER, "Failed to detach object '%s'", object_id.c_str());
    }

    moveit_msgs::msg::CollisionObject remove;
    remove.id = object_id;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!psi.applyCollisionObject(remove))
        RCLCPP_ERROR(LOGGER, "Failed to remove object '%s' from the world", object_id.c_str());
    else
        RCLCPP_INFO(LOGGER, "Object '%s' cleared from the planning scene", object_id.c_str());

    rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

bool openGripper(const rclcpp::Node::SharedPtr& node,
                 const manipulator_action_server::Params& params) {
    RCLCPP_INFO(LOGGER, "Opening gripper (failure recovery)");

    moveit::task_constructor::Task task;
    task.stages()->setName("open_gripper_recovery");
    task.loadRobotModel(node);

    auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

    task.add(std::make_unique<stages::CurrentState>("current state"));

    auto open = std::make_unique<stages::MoveTo>("open hand", interpolation_planner);
    open->setGroup(params.hand_group_name);
    open->setGoal(params.hand_open_pose);
    task.add(std::move(open));

    try {
        task.init();
    } catch (InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Open-gripper task init failed: " << e);
        return false;
    }

    if (!task.plan(1)) {
        RCLCPP_ERROR(LOGGER, "Open-gripper planning failed");
        return false;
    }

    auto res = task.execute(*task.solutions().front());
    if (res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Open-gripper execution failed: %d", res.val);
        return false;
    }
    return true;
}
PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node,
                         const manipulator_action_server::Params& params,
                         bool last_action_flag,
                         const std::vector<std::string>& neighbor_ids,
                         double grasp_angle_delta)
{
    RCLCPP_INFO(LOGGER, "Initializing task pipeline");

    // MTC's GenerateGraspPose::init() throws when angle_delta is exactly zero.
    if (grasp_angle_delta == 0.0) {
        RCLCPP_WARN(LOGGER, "grasp_angle_delta was 0 - falling back to %.4f rad",
                    kDefaultGraspAngleDelta);
        grasp_angle_delta = kDefaultGraspAngleDelta;
    }

    task_.reset();
    task_.reset(new moveit::task_constructor::Task());

    Task& t = *task_;
    t.stages()->setName(task_name_);
    t.loadRobotModel(node);

    // Whole robot, and gripper fingers only.
    const std::vector<std::string> all_links =
        t.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    const std::vector<std::string> gripper_links =
        t.getRobotModel()
            ->getJointModelGroup(params.hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry();

    // ---------------------------------------------------------------------
    //  LANE A allowance policy - "never sweep the arm through a neighbour".
    //  TARGET    : allowed vs whole robot (you're grasping it).
    //  NEIGHBOURS: allowed vs GRIPPER FINGERS ONLY. Arm/forearm/bracelet/
    //              antennas keep treating them as hard obstacles, so the
    //              planner routes around instead of sweeping through.
    //
    //  This WILL reintroduce 'anteena_link_left'/'arm_bracelet_link' vs
    //  'object_N' planning failures / aborts for interior targets in a tight
    //  grid. That is intended - a skipped plant beats a speared one, and the
    //  action server's replan/skip path absorbs it. To never fail a pick,
    //  don't use this policy; set harvest_single_target_mode:=true instead.
    // ---------------------------------------------------------------------
    auto allowPlantCollisions = [&](stages::ModifyPlanningScene* stage) {
        stage->allowCollisions(params.object_name, all_links, true);
        for (const auto& id : neighbor_ids) {
            if (id == params.object_name)
                continue;
            stage->allowCollisions(id, gripper_links, true);
            stage->allowCollisions({ id }, { params.object_name }, true);
        }
    };

    // Planners
    // auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTstar");
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node, "ompl","AnytimePathShortening");
    // sampling_planner->setPlannerId("AnytimePathShortening");
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    sampling_planner->setMaxVelocityScalingFactor(0.7);
    sampling_planner->setMaxAccelerationScalingFactor(0.7);

    auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(.005);

    // Slow joint-interpolation planner, used only for the joint-6 twist.
    auto slow_interp_planner = std::make_shared<solvers::JointInterpolationPlanner>();
    slow_interp_planner->setMaxVelocityScalingFactor(0.08);     
    slow_interp_planner->setMaxAccelerationScalingFactor(0.08);

    //slow cartesian planner, .
    auto slow_cartesian_planner = std::make_shared<solvers::CartesianPath>();
    slow_cartesian_planner->setMaxVelocityScalingFactor(0.07);
    slow_cartesian_planner->setMaxAccelerationScalingFactor(0.07);
    slow_cartesian_planner->setStepSize(.005);

    // Task properties
    t.setProperty("group", params.arm_group_name);
    t.setProperty("eef", params.eef_name);
    t.setProperty("hand", params.hand_group_name);
    t.setProperty("hand_grasping_frame", params.hand_frame);
    t.setProperty("ik_frame", params.hand_frame);

    /****************************************************
     *               Current State
     ***************************************************/
    {
        auto current_state = std::make_unique<stages::CurrentState>("current state");
        auto applicability_filter =
            std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
        applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
            if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                comment = "object with id '" + object + "' is already attached and cannot be picked";
                return false;
            }
            return true;
        });
        t.add(std::move(applicability_filter));
    }
    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object+table)");

        stage->allowCollisions(params.object_name, gripper_links, true);
        // stage->allowCollisions(params.table_name, gripper_links, true);

        t.add(std::move(stage));
    }

    /****************************************************
     *               Open Hand
     ***************************************************/
    Stage* initial_state_ptr = nullptr;
    {
        auto stage = std::make_unique<stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(params.hand_group_name);
        stage->setGoal(params.hand_open_pose);
        initial_state_ptr = stage.get();
        t.add(std::move(stage));
    }

    /****************************************************
     *               Move to Pick
     ***************************************************/
    {
        stages::Connect::GroupPlannerVector planners = { { params.arm_group_name, sampling_planner } };
        auto stage = std::make_unique<stages::Connect>("move to pick", planners);
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(stage));
    }


    /****************************************************
     *               Pick Object
     *
     *  "approach object" is inserted FIRST so neighbouring plants remain hard
     *  obstacles while the gripper threads down between them.
     ***************************************************/
    {
        auto grasp = std::make_unique<SerialContainer>("pick object");
        t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        {
            auto stage = std::make_unique<stages::MoveRelative>("approach object", slow_cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // 1. Generate Grasp Pose + IK
        {
            auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose(params.hand_open_pose);
            stage->setObject(params.object_name);

            // Yaw sampling step (candidates = ceil(2*pi/delta)):
            //  .
            stage->setAngleDelta(grasp_angle_delta);
            RCLCPP_INFO(LOGGER,
                "[%s] GenerateGraspPose on '%s': angle_delta=%.4f rad -> %d yaw candidate(s)",
                task_name_.c_str(), params.object_name.c_str(), grasp_angle_delta,
                static_cast<int>(std::ceil(2.0 * M_PI / std::fabs(grasp_angle_delta))));

            stage->setMonitoredStage(initial_state_ptr);

            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            // Collision-aware IK: reject grasp orientations whose arm config
            // would clip a neighbour (neighbours are hard for arm links here).
            // Interior targets may yield zero IK solutions and fail to plan -
            // intended Lane A trade.
            wrapper->setIgnoreCollisions(false);
            grasp->insert(std::move(wrapper));
        }

        // 2. Allowances: target vs whole robot, neighbours vs GRIPPER ONLY.
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>(
                "allow collision (world object, robot; neighbours vs gripper)");
            allowPlantCollisions(stage.get());
            grasp->insert(std::move(stage));
        }

        // 4. Close Hand
        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_close_pose);
            grasp->insert(std::move(stage));
        }
        //open
        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_open_pose);
            grasp->insert(std::move(stage));
        }
        //moving joint 6 by 10 degresss
        // {
        //     auto stage = std::make_unique<stages::MoveRelative>("twist joint 6", slow_interp_planner);
        //     stage->setGroup(params.arm_group_name);
        //     stage->setIKFrame(params.hand_frame);

        //     // Named joint-space delta: only joint_6 moves, by 10 deg.
        //     std::map<std::string, double> offset{ { "arm_joint_6", 2.0 * M_PI / 180.0 } };
        //     stage->setDirection(offset);

        //     grasp->insert(std::move(stage));
        // }
        // {
        //     auto stage = std::make_unique<stages::MoveRelative>("raise eef inclined", slow_cartesian_planner);
        //     stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        //     stage->setIKFrame(params.hand_frame);
        //     stage->setMinMaxDistance(0.01, 0.01);   // travel 3–5 cm along the tilted line

        //     const double tilt = 5.0 * M_PI / 180.0;

        //     geometry_msgs::msg::Vector3Stamped vec;
        //     vec.header.frame_id = params.world_frame;   // interpret the vector in world frame
        //     vec.vector.z = std::cos(tilt);              // ~0.996  (mostly up)
        //     vec.vector.x = std::cos(tilt);              // ~0.087  (small lean in +X)
        //     vec.vector.y = 0;              // ~0.996  (small lean in +Y)
        //     stage->setDirection(vec);

        //     grasp->insert(std::move(stage));
        // }


        // 5. Allow collision: object <-> support surface (table)
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
            stage->allowCollisions({ params.object_name }, { params.surface_link }, true);
            grasp->insert(std::move(stage));
        }

        // 6. Attach Object to hand frame
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
            stage->attachObject(params.object_name, params.hand_frame);
            grasp->insert(std::move(stage));
        }

        // 7. Re-assert allowances now the object is ATTACHED. Neighbours stay
        //    gripper-only so the arm keeps avoiding them on the lift.
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>(
                "allow collision (attached object, robot; neighbours vs gripper)");
            allowPlantCollisions(stage.get());
            grasp->insert(std::move(stage));
        }

        // 8. Lift Object
        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        t.add(std::move(grasp));
    }

    /****************************************************
     *   Allow collision (neighbours) - TRANSPORT PHASE
     *
     *  Lane A keeps neighbours HARD for the arm during transport too, so the
     *  forearm/bracelet route around the remaining plants to pre_cut_1/drop/
     *  home instead of sweeping through. Only the gripper may brush a plant.
     *  Must be an MTC stage - ExecuteTaskSolution replays scene diffs and each
     *  ModifyPlanningScene diff carries a FULL ACM that REPLACES the live one.
     ***************************************************/
    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>(
            "allow collision (neighbours vs gripper, transport)");
        allowPlantCollisions(stage.get());

        if (!neighbor_ids.empty())
            RCLCPP_INFO(LOGGER,
                "Transport phase: %zu neighbour(s) kept hard for arm links (gripper-only allowance)",
                neighbor_ids.size() - 1);

        t.add(std::move(stage));
    }
    //  Move to safe pose before drop
    // {
    //     auto stage = std::make_unique<stages::MoveTo>("safe transit (pre_cut_1)", sampling_planner);
    //     stage->setGroup(params.arm_group_name);
    //     stage->setGoal("pre_cut_1");
    //     t.add(std::move(stage));
    // }

    /****************************************************
     *               Move to Drop
     ***************************************************/
    {
        auto stage = std::make_unique<stages::MoveTo>("move to drop", sampling_planner);
        stage->setGoal("drop");
        stage->setGroup(params.arm_group_name);
        t.add(std::move(stage));
    }

    /****************************************************
     *               Open Hand (at drop)
     ***************************************************/
    {
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(params.hand_group_name);
        stage->setGoal(params.hand_open_pose);
        t.add(std::move(stage));
    }

    /****************************************************
     *               Detach Object
     ***************************************************/
    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
        stage->detachObject(params.object_name, params.hand_frame);
        t.add(std::move(stage));
    }
    {
        auto stage = std::make_unique<stages::MoveTo>("safe transit (home_0)", sampling_planner);
        stage->setGroup(params.arm_group_name);
        stage->setGoal("home_0");
        t.add(std::move(stage));
    }

    /****************************************************
     *               Move to Home (final, optional)
     ***************************************************/
    if (last_action_flag) {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setGoal(params.arm_home_pose);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        t.add(std::move(stage));
    }

    try {
        t.init();
    } catch (InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
        return false;
    }

    return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions) {
    RCLCPP_INFO(LOGGER, "Start searching for task solutions");

    int attempts = 0;
    int max_attempts = 3;
    bool success = false;

    while (attempts < max_attempts && !success) {
        if (task_->plan(max_solutions)) {
            success = true;
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed, retrying attempt %d...", attempts + 1);
            task_->reset();
            attempts++;
        }
    }

    return success;
}

bool PickPlaceTask::execute() {
    RCLCPP_INFO(LOGGER, "Executing solution trajectory");
    moveit_msgs::msg::MoveItErrorCodes execute_result;

    execute_result = task_->execute(*task_->solutions().front());

    if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
        return false;
    }

    return true;
}

}  