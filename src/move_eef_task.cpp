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

#include <Eigen/Geometry>
#include <kinova_task_manager/move_eef_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

namespace moveit_task_constructor_demo {

MoveEefTask::MoveEefTask(const std::string& task_name) : task_name_(task_name) {}


bool MoveEefTask::setTargetPose(const geometry_msgs::msg::PoseStamped& target_pose){
    RCLCPP_INFO(LOGGER, "Target pose received");

    target_pose_=target_pose;
    return true;
}


bool MoveEefTask::init(const rclcpp::Node::SharedPtr& node, const manipulator_action_server::Params& params) {
	RCLCPP_INFO(LOGGER, "Initializing task pipeline");

	// Reset ROS introspection before constructing the new object
	// TODO(v4hn): global storage for Introspection services to enable one-liner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	// Individual movement stages are collected within the Task object
	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	/* Create planners used in various stages. Various options are available,
	   namely Cartesian, MoveIt pipeline, and joint interpolation. */
	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
	sampling_planner->setMaxVelocityScalingFactor(1.0);
    sampling_planner->setMaxAccelerationScalingFactor(1.0);

	auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// Cartesian planner
	// auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	// cartesian_planner->setMaxVelocityScalingFactor(1.0);
	// cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	// cartesian_planner->setStepSize(.01);

	// Set task properties
	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("hand", params.hand_group_name);
	t.setProperty("hand_grasping_frame", params.hand_frame);
	t.setProperty("ik_frame", params.hand_frame);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
    Stage* current_state_ptr = nullptr;
	{
        auto stage = std::make_unique<stages::CurrentState>("current");
        current_state_ptr = stage.get();
        t.add(std::move(stage));
    }

    {
        auto connect = std::make_unique<stages::Connect>(
            "move to target",
            stages::Connect::GroupPlannerVector{{params.arm_group_name, sampling_planner}}
        );
        t.add(std::move(connect));
    }

    {
        auto target_pose_stage = std::make_unique<stages::GeneratePose>("target pose");
        target_pose_stage->setMonitoredStage(current_state_ptr); 

		RCLCPP_DEBUG(LOGGER, "The target psoe frame id = %s", this->target_pose_.header.frame_id.c_str());
		
        target_pose_stage->setPose(this->target_pose_);
		

        auto wrapper = std::make_unique<stages::ComputeIK>("compute ik", std::move(target_pose_stage));
        wrapper->setMaxIKSolutions(30);
        wrapper->setMinSolutionDistance(0.1);
        wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" }); 
        wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
        wrapper->setEndEffector(params.eef_name);
        wrapper->setGroup(params.arm_group_name);
        wrapper->setIKFrame(params.hand_frame);
        
        t.add(std::move(wrapper));

    }
    
	
	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool MoveEefTask::plan(const std::size_t max_solutions) {
	RCLCPP_INFO(LOGGER, "Start searching for task solutions");

	return static_cast<bool>(task_->plan(max_solutions));
}

bool MoveEefTask::execute() {
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
