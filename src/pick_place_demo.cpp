/*********************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2019 PickNik LLC. All rights reserved.
 *********************************************************************/

#include <Eigen/Geometry>
#include <kinova_task_manager/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>




static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

// Time allowed for a planning-scene change to propagate to move_group before
// we plan or execute against it. NOTE: the previous code used
// std::chrono::microseconds(100) here, i.e. 0.1 ms, which is far too short for
// the scene diff to reach move_group.
static constexpr std::chrono::milliseconds SCENE_SYNC_DELAY{ 200 };

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

// Detach EVERY object still attached to a robot link and drop it from the
// world.
//
// Attached objects are not returned by getKnownObjectNames(), so
// removeAllCollisionObjects() cannot clear them. If a previous pick was
// preempted after MTC's "attach object" stage, the cylinder stays glued to the
// hand: the live scene then contains the same id BOTH attached and in the
// world, and the execution monitor aborts the next trajectory with
//   Found a contact between 'object_2' (Object) and 'object_2' (Robot attached)
// Always call this before spawning a new scene.
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
	pose.position.z -= 0.5 * params.table_dimensions[2];
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

	// A preempted pick can leave the cylinder attached to the hand; clear it
	// before respawning or the old and new bodies collide with each other.
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
                                                   const std::string& id) {
	moveit_msgs::msg::CollisionObject object;
	object.id = id;
	object.header.frame_id = params.object_reference_frame;

	// Object frame = the BASE of the cylinder (grasps generate around this)
	object.pose = pose_in;

	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };

	// Cylinder body sits ABOVE the base frame: center at +half height, relative
	geometry_msgs::msg::Pose rel;
	rel.orientation.w = 1.0;
	rel.position.z = 0.5 * params.object_dimensions.at(0);
	object.primitive_poses.push_back(rel);

	return object;
}

// ONE target at a time.
//
// Wipes the scene (including anything still attached to the hand) and spawns
// the table plus exactly ONE cylinder — the target about to be picked.
//
// Why: move_group's execution monitor re-validates the remaining trajectory
// against the LIVE planning scene on every scene update. The MTC
// ModifyPlanningScene stages below only whitelist params.object_name, so every
// OTHER cylinder in the scene is still forbidden against every robot link. As
// soon as a finger passes near a neighbour the monitor fires
//   "Stopping execution because the path to execute became invalid"
// and cancels the controller goal mid-motion, which is what drives the arm
// into the Kortex fault state. With a single object in the scene there is no
// neighbour left to invalidate the path.
//
// Trade-off: the planner no longer routes around the not-yet-picked plants,
// so the arm may brush through where they physically are. Acceptable for soft
// plant matter; if that becomes a problem, spawn the full grid for planning
// and whitelist every object_* in the live ACM just before execute() instead.
void setupSingleTargetScene(const manipulator_action_server::Params& params,
                            const geometry_msgs::msg::Pose& object_pose,
                            const std::string& object_id) {
	RCLCPP_INFO(LOGGER, "Setting the planning scene with single target '%s'", object_id.c_str());
	moveit::planning_interface::PlanningSceneInterface psi;

	detachAllLeftoverObjects(psi);
	removeAllCollisionObjects(psi);
	rclcpp::sleep_for(SCENE_SYNC_DELAY);

	if (params.spawn_table)
		spawnObject(psi, createTable(params));
	spawnObject(psi, createGridObject(params, object_pose, object_id));

	// Give move_group time to apply the diff before planning against it.
	rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

// Spawn the whole harvest grid at once. Kept for reference / experiments:
// planning against it makes the arm avoid neighbouring plants, but the
// execution monitor will then abort on those neighbours unless they are also
// whitelisted in the LIVE scene ACM before execute(). The action server uses
// setupSingleTargetScene() instead.
void setupHarvestGridScene(const manipulator_action_server::Params& params,
                           const std::vector<geometry_msgs::msg::Pose>& object_poses,
                           const std::vector<std::string>& object_ids) {
	RCLCPP_INFO(LOGGER, "Setting the planning scene with %zu harvest grid object(s)",
	            object_poses.size());
	moveit::planning_interface::PlanningSceneInterface psi;

	detachAllLeftoverObjects(psi);
	removeAllCollisionObjects(psi);
	rclcpp::sleep_for(SCENE_SYNC_DELAY);

	if (params.spawn_table)
		spawnObject(psi, createTable(params));

	for (std::size_t i = 0; i < object_poses.size(); ++i) {
		spawnObject(psi, createGridObject(params, object_poses.at(i), object_ids.at(i)));
	}
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

// True when 'object_id' is currently attached to a robot link in the LIVE
// planning scene. A pick that fails after MTC's "attach object" stage leaves
// the object attached — MoveIt then believes the hand is carrying it, every
// later plan collides with the phantom object, and the applicability filter
// blocks re-picking it. Note: attached objects are NOT returned by
// getKnownObjectNames(), so the scene-setup helpers call
// detachAllLeftoverObjects() to clear them.
bool isObjectAttached(const std::string& object_id) {
	moveit::planning_interface::PlanningSceneInterface psi;
	auto attached = psi.getAttachedObjects({ object_id });
	return attached.count(object_id) > 0;
}

// Detach 'object_id' from whatever link holds it (no-op when not attached),
// then remove it from the world. Safe to call regardless of attachment
// state; never throws.
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

	// Detaching re-inserts the object into the world at its current pose —
	// remove it from the world as well.
	moveit_msgs::msg::CollisionObject remove;
	remove.id = object_id;
	remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
	if (!psi.applyCollisionObject(remove))
		RCLCPP_ERROR(LOGGER, "Failed to remove object '%s' from the world", object_id.c_str());
	else
		RCLCPP_INFO(LOGGER, "Object '%s' cleared from the planning scene", object_id.c_str());

	rclcpp::sleep_for(SCENE_SYNC_DELAY);
}

// Recovery move: open the gripper at the current arm pose (used after moving
// to the drop pose so a physically-held plant is released). Built as a
// minimal MTC task so it uses the same execution path as everything else.
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

// REMOVED: applyCollisionAllowancesToLiveScene()
//
// It was never called from anywhere, and as written it was unsafe: it pushed
// AllowedCollisionEntry values with a single-element `enabled` vector into a
// diff whose allowed_collision_matrix was otherwise empty. A non-empty ACM in
// a PlanningScene diff can REPLACE the live matrix rather than merge into it,
// which would drop the SRDF's disabled self-collision pairs. If a live-scene
// ACM edit is ever needed again, do a read-modify-write through the
// /get_planning_scene service and send back the FULL matrix.

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node,
                         const manipulator_action_server::Params& params,
                         bool last_action_flag)
{
	RCLCPP_INFO(LOGGER, "Initializing task pipeline");

	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	// Planners
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTstar");
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
	sampling_planner->setMaxVelocityScalingFactor(0.5);
	sampling_planner->setMaxAccelerationScalingFactor(0.5);

	auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(0.5);
	cartesian_planner->setMaxAccelerationScalingFactor(0.5);
	cartesian_planner->setStepSize(.005);

	// Task properties
	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("hand", params.hand_group_name);
	t.setProperty("hand_grasping_frame", params.hand_frame);
	t.setProperty("ik_frame", params.hand_frame);

	/****************************************************
	 *               Current State
	 ***************************************************/
	Stage* current_state_ptr = nullptr;
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
		current_state_ptr = current_state.get();
		t.add(std::move(applicability_filter));
	}
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object+table)");

		auto gripper_links = t.getRobotModel()
			->getJointModelGroup(params.hand_group_name)
			->getLinkModelNamesWithCollisionGeometry();

		stage->allowCollisions(params.object_name, gripper_links, true);
		stage->allowCollisions(params.table_name, gripper_links, true);

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
	 ***************************************************/
	Stage* pick_stage_ptr = nullptr;
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
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
			stage->setAngleDelta(M_PI/2);
			stage->setMonitoredStage(initial_state_ptr);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			 // Collision allowances are handled explicitly by ModifyPlanningScene stages
   			wrapper->setIgnoreCollisions(true);
			grasp->insert(std::move(wrapper));
		}

		// 2. Allow collision: hand <-> object AND hand <-> table
		//    Must come BEFORE approach so the execution monitor doesn't abort
		//    when a finger touches the object/table during approach.
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");

			// Allow hand group links to touch the object
			stage->allowCollisions(
				params.object_name,
				t.getRobotModel()->getJointModelGroup(params.hand_group_name)
					->getLinkModelNamesWithCollisionGeometry(),
				true);

			// Allow hand group links to touch the table surface
			// (finger clips table edge during approach on real hardware)
			stage->allowCollisions(
				params.table_name,
				t.getRobotModel()->getJointModelGroup(params.hand_group_name)
					->getLinkModelNamesWithCollisionGeometry(),
				true);

			grasp->insert(std::move(stage));
		}

		// 4. Close Hand
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", interpolation_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_close_pose);
			grasp->insert(std::move(stage));
		}
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", interpolation_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_open_pose);
			grasp->insert(std::move(stage));
		}

		// 5. Allow collision: object <-> support surface (table)
		//    Must come BEFORE attach so attach doesn't fail due to table collision
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

		pick_stage_ptr = grasp.get();
		t.add(std::move(grasp));
	}

	/****************************************************
	 *               Move to Drop
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
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

}  // namespace moveit_task_constructor_demo