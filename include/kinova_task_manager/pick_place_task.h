/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
  Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <rclcpp/node.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
// #include <kinova_task_manager/manipulator_action_server_parameters.hpp>
// #include <kinova_task_manager/pick_place_demo_parameters.hpp>
#include <kinova_task_manager/manipulator_action_server_parameters.hpp>

#pragma once

namespace moveit_task_constructor_demo {
using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under node
static constexpr double kDefaultGraspAngleDelta = M_PI / 2.0;
static constexpr double kSingleYawAngleDelta = 2.0 * M_PI;
void setupDemoScene(const manipulator_action_server::Params& params);
void clearPlanningScene();
// Spawn the table (if enabled) plus one cylinder collision object per
// (pose, id) pair — used to populate the whole harvest grid at once so all
// not-yet-picked plants stay present as obstacles while each one is picked.
void setupHarvestGridScene(const manipulator_action_server::Params& params,
                           const std::vector<geometry_msgs::msg::Pose>& object_poses,
                           const std::vector<std::string>& object_ids,
                           const std::vector<double>& object_radii);
void setupSingleTargetScene(const manipulator_action_server::Params& params,
                            const geometry_msgs::msg::Pose& object_pose,
                            const std::string& object_id,
                            double radius);


// Remove a single harvested cylinder's collision object from the live
// planning scene. Call once a target has been successfully picked and
// dropped — otherwise it stays parked at the drop pose and piles up with
// every other harvested object from the same detection.
void removeHarvestedObject(const std::string& object_id);
// True when 'object_id' is attached to a robot link in the live planning scene.
bool isObjectAttached(const std::string& object_id);

// Detach 'object_id' if attached, then remove it from the world. Never throws.
void detachAndRemoveObject(const std::string& object_id);

// Open the gripper at the current pose (failure recovery).
bool openGripper(const rclcpp::Node::SharedPtr& node,
                 const manipulator_action_server::Params& params);



class PickPlaceTask
{
public:
	PickPlaceTask(const std::string& task_name);
	~PickPlaceTask() = default;
	// bool init(const rclcpp::Node::SharedPtr& node, const manipulator_action_server::Params& params);
	// bool init(const rclcpp::Node::SharedPtr& node, const manipulator_action_server::Params& params, bool last_action_flag);
	bool init(const rclcpp::Node::SharedPtr& node, const manipulator_action_server::Params& params,
	          bool last_action_flag, const std::vector<std::string>& neighbor_ids = {},
	          double grasp_angle_delta = kDefaultGraspAngleDelta);

	bool plan(const std::size_t max_solutions);

	bool execute();

	void preempt() { if (task_) task_->preempt(); }

	std::string task_name_;
	moveit::task_constructor::TaskPtr task_;
};
}  // namespace moveit_task_constructor_demo
