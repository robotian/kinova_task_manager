#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kinova_task_manager/pick_place_task.h>
#include <kinova_task_manager/move_to_config_task.h>
#include <kinova_task_manager/move_eef_task.h>
#include "status_interfaces/action/manipulator_task.hpp"
#include "status_interfaces/msg/image_detection.hpp"
#include "kinova_task_manager/manipulator_commands.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/srv/trigger.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>


using Manipulator = status_interfaces::action::ManipulatorTask;
using GoalHandleManipulator = rclcpp_action::ServerGoalHandle<Manipulator>;
using namespace std::placeholders;


class ManipulatorActionServer : public rclcpp::Node {
public:
    explicit ManipulatorActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("manipulator_action_server", options) {

        RCLCPP_INFO(this->get_logger(), "Starting Manipulator Action Server...");

        this->manipulator_status_ = Manipulator::Feedback::PLANNING;

        this->action_server_ = rclcpp_action::create_server<Manipulator>(
            this,
            "manipulator_action_server",
            std::bind(&ManipulatorActionServer::handle_goal, this, _1, _2),
            std::bind(&ManipulatorActionServer::handle_cancel, this, _1),
            std::bind(&ManipulatorActionServer::handle_accepted, this, _1));

        // Stop the physical arm on shutdown (Ctrl+C, kill, launch teardown).
        // Without this, closing the node just abandons whatever trajectory
        // goal is in flight - the controller keeps executing it because
        // nothing ever cancels it. Task::preempt() (MTC) makes the currently
        // blocked execute() call cancel its 'execute_task_solution' goal,
        // which MoveIt propagates down to the controller.
        rclcpp::on_shutdown([this]() {
            shutting_down_.store(true);
            std::lock_guard<std::mutex> lock(active_task_mutex_);
            if (active_preempt_) {
                RCLCPP_WARN(this->get_logger(),
                    "Shutdown requested while the arm is moving - cancelling trajectory execution...");
                active_preempt_();
            }
        });

        // Fault controller service client
        fault_reset_client_ = this->create_client<example_interfaces::srv::Trigger>(
            "/a200_0284/manipulators/fault_controller/reset_fault");

        // Fault state subscription (latched last value via QoS depth 1).
        fault_state_sub_ = this->create_subscription<example_interfaces::msg::Bool>(
            "/a200_0284/manipulators/fault_controller/internal_fault",
            rclcpp::QoS(1),
            [this](const example_interfaces::msg::Bool::SharedPtr msg) {
                bool was_in_fault = arm_in_fault_.exchange(msg->data);
                if (msg->data && !was_in_fault) {
                    RCLCPP_ERROR(this->get_logger(), "Arm entered FAULT state!");
                } else if (!msg->data && was_in_fault) {
                    RCLCPP_INFO(this->get_logger(), "Arm fault cleared.");
                }
            });

        // TF2 for frame transforms (used by IMAGE_DETECTION_HARVEST)
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Detection / workspace parameters --------------------------------
        workspace_reach_radius_m_ = this->declare_parameter("workspace_reach_radius_m", 0.75);
        detection_timeout_sec_    = this->declare_parameter("detection_timeout_sec", 15.0);
        camera_frame_             = this->declare_parameter("camera_frame",
                                        std::string("arm_camera_color_frame"));
        harvest_replan_attempts_  = this->declare_parameter("harvest_replan_attempts", 5);
        execution_retry_attempts_ = this->declare_parameter("execution_retry_attempts", 2);

        // --- Square-grid target generation -----------------------------------
        // A rectangle centred on the detection center; pick points sit on a
        // fixed grid_point_spacing_m lattice inside it. Points per axis =
        // floor(side/spacing) + 1.
        //   0.40 m @ 0.15 m -> 3x3 (9)
        //   0.40 m @ 0.10 m -> 5x5 (25)
        // Rectangle side is taken from the detection's extents (x, y) when
        // extents_valid is true; center_box_size_m is the fallback used
        // otherwise, and also the clamp bound for implausible extents.
        center_box_size_m_    = this->declare_parameter("center_box_size_m", 0.38);
        grid_point_spacing_m_ = this->declare_parameter("grid_point_spacing_m", 0.14);
        max_harvest_targets_  = this->declare_parameter("max_harvest_targets", 12);
        extents_min_m_        = this->declare_parameter("extents_min_m", 0.20);
        extents_max_m_        = this->declare_parameter("extents_max_m", 0.50);

        // --- Grid shape ---------------------------------------------------------
        // "square": the rectangular lattice above (default, unchanged behaviour).
        // "circular": a center point plus 8 points on one ring at 45-degree
        //   increments (0/45/90/.../315), elliptically scaled by the same
        //   extents-derived half-widths as the square grid. Falls back to
        //   "square" for the goal if fewer than circular_min_reachable of the
        //   9 points survive the reachability check.
        grid_shape_ = this->declare_parameter("grid_shape", std::string("square"));
        if (grid_shape_ != "square" && grid_shape_ != "circular") {
            RCLCPP_WARN(this->get_logger(),
                "Unknown grid_shape '%s' - falling back to 'square'", grid_shape_.c_str());
            grid_shape_ = "square";
        }
        circular_min_reachable_ = this->declare_parameter("circular_min_reachable", 5);

        // --- Cylinder overlap -----------------------------------------------
        // Points closer than 2*radius (object_dimensions[1]) would spawn
        // overlapping collision cylinders; cylinder_overlap_margin_m adds
        // clearance on top of that touching distance. Applies to both grid
        // shapes - see minCylinderSeparation()/dedupPickTargets().
        cylinder_overlap_margin_m_ = this->declare_parameter("cylinder_overlap_margin_m", 0.01);

        // --- Per-row cylinder size taper -----------------------------------
        // Cylinder radius scales with how far a target's row is from the
        // middle row: cylinder_size_edge_scale at row 0 and the last row,
        // cylinder_size_center_scale at the middle row, smoothly interpolated
        // in between (object_dimensions[1] is the un-scaled base radius).
        // Applies to both grid shapes - see cylinderRadiusForRow(). Set both
        // to 1.0 to disable (every cylinder gets the base radius, as before).
        cylinder_size_edge_scale_   = this->declare_parameter("cylinder_size_edge_scale", 1.15);
        cylinder_size_center_scale_ = this->declare_parameter("cylinder_size_center_scale", 0.85);

        // --- Grid-point reachability check -------------------------------------
        // "circle": cheap hypot(x,y) <= workspace_reach_radius_m_ check (default,
        //   unchanged behaviour).
        // "ik": real kinematic feasibility via RobotState::setFromIK against the
        //   arm group's configured KDL solver (~5 ms timeout, 3 attempts/point) -
        //   catches points inside the workspace circle that are still
        //   unreachable because of orientation/joint-limit constraints.
        reachability_mode_ = this->declare_parameter("reachability_mode", std::string("circle"));
        if (reachability_mode_ != "circle" && reachability_mode_ != "ik") {
            RCLCPP_WARN(this->get_logger(),
                "Unknown reachability_mode '%s' - falling back to 'circle'", reachability_mode_.c_str());
            reachability_mode_ = "circle";
        }

        // --- Center gating ----------------------------------------------------
        center_x_min_            = this->declare_parameter("center_x_min", -0.05);
        center_x_max_            = this->declare_parameter("center_x_max", 0.15);
        center_acquire_attempts_ = this->declare_parameter("center_acquire_attempts", 3);

        // --- Center stability gating --------------------------------------------
        // A single-frame center can be a transient outlier (partial occlusion,
        // wind-blown plant, detector flicker). Require center_stability_count
        // consecutive, distinct detections whose transformed centers all fall
        // within center_stability_tolerance_m of each other before accepting -
        // any reading that drifts outside tolerance restarts the streak.
        // center_stability_count=1 reduces to "accept the first valid reading",
        // i.e. today's behaviour.
        center_stability_count_       = this->declare_parameter("center_stability_count", 1);
        center_stability_tolerance_m_ = this->declare_parameter("center_stability_tolerance_m", 0.015);

        // --- Scene strategy ---------------------------------------------------
        // false (default): spawn the whole grid up front so the planner routes
        //   around neighbours; every neighbour id is handed to
        //   PickPlaceTask::init() for the in-task allowances.
        // true: spawn ONE object at a time (no neighbour can invalidate the
        //   trajectory). Flip with:
        //     ros2 param set /manipulator_action_server harvest_single_target_mode true
        harvest_single_target_mode_ =
            this->declare_parameter("harvest_single_target_mode", false);

        // --- Grasp yaw sampling, per grid row ---------------------------------
        // MTC's GenerateGraspPose sweeps yaw about the object axis in steps of
        // angle_delta over the full circle, so the candidate count is
        // ceil(2*pi / delta):
        //   2*M_PI -> 1 candidate  (yaw 0 only; deterministic and fast, but no
        //                           alternative orientation to fall back on)
        //   M_PI   -> 2 candidates (0, 180)
        //   M_PI/2 -> 4 candidates (0, 90, 180, 270)
        //
        // Rows [0, single_yaw_row_count) are the ones NEAREST the arm (row 0 is
        // nearest) and use the "near" delta; every farther row uses the "far"
        // delta. With the 3x3 default that means grid indices 0-5 get one yaw
        // and the far row (6-8) gets four.
        single_yaw_row_count_   = this->declare_parameter("single_yaw_row_count", 2);
        grasp_angle_delta_near_ = this->declare_parameter("grasp_angle_delta_near", 2.0 * M_PI);
        grasp_angle_delta_far_  = this->declare_parameter("grasp_angle_delta_far", 2.0 * M_PI);
        grasp_angle_delta_widen_ = this->declare_parameter("grasp_angle_delta_widen", M_PI/2.0);

        // Re-running init() with the same delta re-samples the identical yaw
        // set, so a single-yaw target would burn every replan attempt on the
        // exact same infeasible pose. When true, attempts 2..N widen to the
        // "far" delta.
        widen_yaw_on_replan_ = this->declare_parameter("widen_yaw_on_replan", true);

        RCLCPP_INFO(this->get_logger(),
            "Manipulator Action Server ready. Square grid, scene strategy: %s, reachability check: %s",
            harvest_single_target_mode_ ? "ONE object at a time" : "full grid up front",
            reachability_mode_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Grasp yaw: rows < %d -> %.4f rad (%d cand.), farther rows -> %.4f rad (%d cand.), "
            "widen -> %.4f rad (%d cand.), widen on replan: %s",
            single_yaw_row_count_,
            grasp_angle_delta_near_, yawCandidateCount(grasp_angle_delta_near_),
            grasp_angle_delta_far_,  yawCandidateCount(grasp_angle_delta_far_),
            grasp_angle_delta_widen_, yawCandidateCount(grasp_angle_delta_widen_),
            widen_yaw_on_replan_ ? "yes" : "no");
    }

    ~ManipulatorActionServer() override {
        // By the time we get here, main() has already called rclcpp::spin()
        // (returned) and rclcpp::shutdown() - the on_shutdown handler above
        // has already fired and requested a preempt of whatever was
        // executing. Join here so the process doesn't exit (destroying tf_buffer_,
        // param_listener_, etc.) while that thread is still unwinding.
        std::lock_guard<std::mutex> lock(threads_mutex_);
        for (auto& entry : worker_threads_) {
            if (entry.thread.joinable()) entry.thread.join();
        }
    }

    void init_parameters() {
        param_listener_ = std::make_shared<manipulator_action_server::ParamListener>(this->shared_from_this());

        // Only pay the model-load cost when the "ik" reachability mode is
        // actually selected. Loaded once here and reused for every goal -
        // never reload per-goal, that's the slow part.
        if (reachability_mode_ == "ik") {
            robot_model_loader::RobotModelLoader loader(this->shared_from_this());
            robot_model_ = loader.getModel();
            if (!robot_model_) {
                RCLCPP_ERROR(this->get_logger(),
                    "reachability_mode=ik requested but the robot model failed to load "
                    "(check robot_description/robot_description_semantic) - "
                    "falling back to 'circle'");
                reachability_mode_ = "circle";
            }
        }
    }

private:
    rclcpp_action::Server<Manipulator>::SharedPtr action_server_;
    std::shared_ptr<manipulator_action_server::ParamListener> param_listener_;
    int8_t manipulator_status_;

    // Fault handling
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr fault_reset_client_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr fault_state_sub_;
    std::atomic<bool> arm_in_fault_{false};

    // Shutdown / cancellation. active_preempt_ is whatever cancels the task
    // currently blocked in execute() (set up by ActiveTaskGuard below); the
    // on_shutdown handler calls it so closing the node actually stops the arm
    // instead of abandoning the in-flight trajectory goal.
    std::mutex active_task_mutex_;
    std::function<void()> active_preempt_;
    std::atomic<bool> shutting_down_{false};

    // Per-goal worker threads. Tracked (not detached) so the destructor can
    // join them - see ~ManipulatorActionServer().
    struct WorkerEntry {
        std::thread thread;
        std::shared_ptr<std::atomic<bool>> done;
    };
    std::mutex threads_mutex_;
    std::vector<WorkerEntry> worker_threads_;

    // RAII: while alive, active_preempt_ cancels `task`'s trajectory goal.
    // Wrap every task.execute() call with this so a shutdown mid-motion has
    // something to call.
    template <typename TaskT>
    class ActiveTaskGuard {
    public:
        ActiveTaskGuard(ManipulatorActionServer* self, TaskT& task) : self_(self) {
            std::lock_guard<std::mutex> lock(self_->active_task_mutex_);
            self_->active_preempt_ = [&task]() { task.preempt(); };
        }
        ~ActiveTaskGuard() {
            std::lock_guard<std::mutex> lock(self_->active_task_mutex_);
            self_->active_preempt_ = nullptr;
        }
        ActiveTaskGuard(const ActiveTaskGuard&) = delete;
        ActiveTaskGuard& operator=(const ActiveTaskGuard&) = delete;
    private:
        ManipulatorActionServer* self_;
    };

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Detection subscription (per-goal lifetime)
    rclcpp::Subscription<status_interfaces::msg::ImageDetection>::SharedPtr detection_sub_;
    std::shared_ptr<status_interfaces::msg::ImageDetection> latest_detection_;
    std::mutex detection_mutex_;

    int harvest_replan_attempts_;
    int execution_retry_attempts_;

    // Square-grid parameters
    double center_box_size_m_;     // fallback side length when extents are invalid (m)
    double grid_point_spacing_m_;  // spacing between grid points (m)
    int    max_harvest_targets_;   // hard cap on targets per goal
    double extents_min_m_;         // clamp: minimum grid side derived from extents (m)
    double extents_max_m_;         // clamp: maximum grid side derived from extents (m)

    // Grid shape ("square" or "circular")
    std::string grid_shape_;
    int         circular_min_reachable_;     // min reachable ring points before falling back to square
    double      cylinder_overlap_margin_m_;  // extra clearance added to 2*radius when deduping targets
    double      cylinder_size_edge_scale_;   // radius multiplier at row 0 / last row
    double      cylinder_size_center_scale_; // radius multiplier at the middle row

    // Grid-point reachability check ("circle" or "ik")
    std::string reachability_mode_;
    moveit::core::RobotModelConstPtr robot_model_;  // only loaded when reachability_mode_ == "ik"

    // Detection parameters
    double workspace_reach_radius_m_;
    double detection_timeout_sec_;
    double center_x_min_;
    double center_x_max_;
    int    center_acquire_attempts_;
    std::string camera_frame_;
    int    center_stability_count_;
    double center_stability_tolerance_m_;

    // Scene strategy
    bool harvest_single_target_mode_;

    // Grasp yaw sampling
    int    single_yaw_row_count_;
    double grasp_angle_delta_near_;
    double grasp_angle_delta_far_;
    double grasp_angle_delta_widen_;
    bool   widen_yaw_on_replan_;

    // Fixed pick height above arm_base_link (metres)
    static constexpr double PICK_Z_HEIGHT = 0.01;//-0.12 for offical field testing, 

    // ----------------------------------------------------------------
    //  Grasp yaw helpers
    // ----------------------------------------------------------------

    static int yawCandidateCount(double angle_delta) {
        if (angle_delta == 0.0) return 0;
        return static_cast<int>(std::ceil(2.0 * M_PI / std::fabs(angle_delta)));
    }

    // Row 0 is nearest the arm. Rows below the threshold get the "near" delta.
    double graspAngleDeltaForRow(int row) const {
        return (row < single_yaw_row_count_) ? grasp_angle_delta_near_
                                             : grasp_angle_delta_far_;
    }

    // ----------------------------------------------------------------
    //  Detection subscription helpers
    // ----------------------------------------------------------------

    void subscribeToDetection(const std::string& topic) {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_detection_ = nullptr;
        detection_sub_ = this->create_subscription<status_interfaces::msg::ImageDetection>(
            topic, rclcpp::SensorDataQoS(),
            [this](const status_interfaces::msg::ImageDetection::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(detection_mutex_);
                latest_detection_ = std::make_shared<status_interfaces::msg::ImageDetection>(*msg);
            });
        RCLCPP_INFO(this->get_logger(), "Subscribed to detection topic: %s", topic.c_str());
    }

    void unsubscribeFromDetection() {
        detection_sub_.reset();
    }

    bool validateCenter(const status_interfaces::msg::ImageDetection& det,
                        double& cx, double& cy) {
        if (!det.center_valid) {
            RCLCPP_WARN(this->get_logger(), "Detection center is not valid");
            return false;
        }
        if (!transformPointXYToArmBase(det.center, cx, cy)) {
            RCLCPP_WARN(this->get_logger(), "TF transform of detection center failed");
            return false;
        }
        if (!isPointReachable(cx, cy)) {
            RCLCPP_WARN(this->get_logger(),
                "Center (%.3f, %.3f) exceeds workspace radius %.3f m",
                cx, cy, workspace_reach_radius_m_);
            return false;
        }
        if (cx < center_x_min_ || cx > center_x_max_) {
            RCLCPP_WARN(this->get_logger(),
                "Center x=%.3f outside allowed band [%.3f, %.3f] m", cx, center_x_min_, center_x_max_);
            return false;
        }
        return true;
    }

    // Polls latest_detection_ until center_stability_count_ consecutive,
    // distinct (by header.stamp) detections pass validateCenter and all fall
    // within center_stability_tolerance_m_ of each other - a single reading
    // that drifts outside tolerance restarts the streak rather than failing
    // outright, since the plant may just be settling after being disturbed.
    // center_stability_count_==1 returns on the very first valid reading,
    // matching the old waitForValidDetection+validateCenter behaviour.
    std::shared_ptr<status_interfaces::msg::ImageDetection>
    waitForStableDetection(const std::shared_ptr<GoalHandleManipulator>& gh,
                           double& cx, double& cy) {
        auto deadline = this->now() + rclcpp::Duration::from_seconds(detection_timeout_sec_);
        const int required = std::max(1, center_stability_count_);

        std::vector<std::pair<double, double>> streak;
        std::shared_ptr<status_interfaces::msg::ImageDetection> streak_last;
        builtin_interfaces::msg::Time last_stamp;
        bool have_last_stamp = false;

        while (rclcpp::ok() && this->now() < deadline && !gh->is_canceling()) {
            std::shared_ptr<status_interfaces::msg::ImageDetection> det;
            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                if (latest_detection_ && latest_detection_->detection_valid) {
                    det = latest_detection_;
                }
            }

            if (det) {
                const auto& stamp = det->header.stamp;
                bool is_new = !have_last_stamp ||
                    stamp.sec != last_stamp.sec || stamp.nanosec != last_stamp.nanosec;

                if (is_new) {
                    have_last_stamp = true;
                    last_stamp = stamp;

                    double px, py;
                    if (validateCenter(*det, px, py)) {
                        bool agrees = std::all_of(streak.begin(), streak.end(),
                            [&](const auto& p) {
                                return std::hypot(px - p.first, py - p.second)
                                    <= center_stability_tolerance_m_;
                            });
                        if (!agrees) {
                            RCLCPP_INFO(this->get_logger(),
                                "Center (%.3f, %.3f) drifted beyond %.3f m tolerance - "
                                "restarting %d-reading stability streak",
                                px, py, center_stability_tolerance_m_, required);
                            streak.clear();
                        }
                        streak.push_back({px, py});
                        streak_last = det;

                        if (static_cast<int>(streak.size()) >= required) {
                            cx = px; cy = py;
                            return streak_last;
                        }
                    } else {
                        streak.clear();
                        streak_last = nullptr;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return nullptr;
    }

    std::shared_ptr<status_interfaces::msg::ImageDetection>
    acquireValidDetection(const std::string& topic,
                          const std::shared_ptr<GoalHandleManipulator>& gh,
                          double& cx, double& cy)
    {
        int max_attempts = std::max(1, center_acquire_attempts_);
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            if (gh->is_canceling() || !rclcpp::ok()) return nullptr;

            subscribeToDetection(topic);
            auto det = waitForStableDetection(gh, cx, cy);
            unsubscribeFromDetection();

            if (!det) {
                RCLCPP_WARN(this->get_logger(),
                    "No stable detection (%d consecutive within %.3f m) within %.1f s (attempt %d/%d)",
                    std::max(1, center_stability_count_), center_stability_tolerance_m_,
                    detection_timeout_sec_, attempt, max_attempts);
                continue;
            }

            RCLCPP_INFO(this->get_logger(),
                "Stable detection acquired (id: %s, score: %.2f) at (%.3f, %.3f) - attempt %d/%d",
                det->detector_id.c_str(), det->score, cx, cy, attempt, max_attempts);
            return det;
        }
        return nullptr;
    }

    // ----------------------------------------------------------------
    //  Workspace / geometry helpers
    // ----------------------------------------------------------------

    bool isPointReachable(double x, double y) {
        return std::hypot(x, y) <= workspace_reach_radius_m_;
    }

    bool transformPointXYToArmBase(const geometry_msgs::msg::Point& in_pt,
                                   double& x_out, double& y_out) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                "arm_base_link", camera_frame_, tf2::TimePointZero,
                std::chrono::milliseconds(100));

            geometry_msgs::msg::PointStamped ps_in, ps_out;
            ps_in.header.frame_id = camera_frame_;
            ps_in.point = in_pt;
            tf2::doTransform(ps_in, ps_out, tf);

            x_out = ps_out.point.x;
            y_out = ps_out.point.y;
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return false;
        }
    }

    // Extents are a size, not a position, so only the transform's rotation
    // applies (Vector3Stamped ignores translation) - this re-expresses the
    // camera-frame box width/depth along arm_base_link's x/y axes.
    bool transformExtentsToArmBase(const geometry_msgs::msg::Vector3& in_vec,
                                   double& x_out, double& y_out) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                "arm_base_link", camera_frame_, tf2::TimePointZero,
                std::chrono::milliseconds(100));

            geometry_msgs::msg::Vector3Stamped vs_in, vs_out;
            vs_in.header.frame_id = camera_frame_;
            vs_in.vector = in_vec;
            tf2::doTransform(vs_in, vs_out, tf);

            x_out = std::fabs(vs_out.vector.x);
            y_out = std::fabs(vs_out.vector.y);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform of extents failed: %s", ex.what());
            return false;
        }
    }

    // Real kinematic feasibility check for reachability_mode_ == "ik": builds a
    // probe pose at (x, y, PICK_Z_HEIGHT) with identity orientation (matching
    // how createGridObject spawns the cylinder), offset by grasp_frame_transform
    // (the same offset ComputeIK applies via setIKFrame in pick_place_task.cpp),
    // and solves IK against the arm group's configured KDL solver. Single
    // yaw=0 probe, pure-kinematic (no collision checking against the table or
    // neighbouring cylinders) - a fast upfront filter, not a replacement for
    // the collision-aware ComputeIK stage that runs during the actual pick.
    bool checkIKReachable(double x, double y) {
        if (!robot_model_) return isPointReachable(x, y);  // shouldn't happen; guard anyway

        const auto params = param_listener_->get_params();
        const auto* jmg = robot_model_->getJointModelGroup(params.arm_group_name);
        if (!jmg) {
            RCLCPP_ERROR(this->get_logger(),
                "Joint model group '%s' not found - falling back to circle check",
                params.arm_group_name.c_str());
            return isPointReachable(x, y);
        }

        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();

        Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
        object_pose.translation() = Eigen::Vector3d(x, y, PICK_Z_HEIGHT);

        const auto& g = params.grasp_frame_transform;  // [x,y,z,r,p,y]
        Eigen::Isometry3d grasp_offset =
            Eigen::Translation3d(g[0], g[1], g[2]) *
            Eigen::AngleAxisd(g[3], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(g[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(g[5], Eigen::Vector3d::UnitZ());

        // object_pose is expressed in arm_base_link (matches object_reference_frame
        // used to spawn the actual collision cylinder); setFromIK needs the target
        // in the model's own root frame, so compose through arm_base_link's FK.
        Eigen::Isometry3d target_in_model_frame =
            state.getGlobalLinkTransform("arm_base_link") * object_pose * grasp_offset;

        return state.setFromIK(jmg, target_in_model_frame, params.hand_frame, 0.0);
    }

    bool isTargetReachable(double x, double y) {
        return (reachability_mode_ == "ik") ? checkIKReachable(x, y) : isPointReachable(x, y);
    }

    // Walk from (px,py) toward the center in 1 cm steps until reachable.
    std::pair<double, double> projectPointInward(
        double px, double py, double center_x, double center_y)
    {
        double dx = center_x - px, dy = center_y - py;
        double dist = std::hypot(dx, dy);
        if (dist < 1e-6) return {px, py};

        double ux = dx / dist, uy = dy / dist;
        constexpr double step = 0.01;
        int steps = static_cast<int>(dist / step);
        for (int i = 0; i <= steps; ++i) {
            double cx = px + ux * (i * step);
            double cy = py + uy * (i * step);
            if (isTargetReachable(cx, cy)) return {cx, cy};
        }
        return {center_x, center_y};
    }

    // 'row' travels with the point so the grasp-yaw policy survives the dedup
    // and cap passes below, which shift/drop indices. Never derive the row
    // from the target index.
    struct PickTarget { double x, y; int row; };

    // Discard points whose cylinders would overlap (or collapsed onto each
    // other after inward projection). min_separation_m should be the actual
    // cylinder diameter plus clearance - see minCylinderSeparation() - not an
    // arbitrary constant, so it tracks object_dimensions if that ever changes.
    // Shared by every grid shape. Keeps the earlier point on a collision,
    // consistent with the near-to-far ordering callers already produce.
    static void dedupPickTargets(std::vector<PickTarget>& targets, double min_separation_m) {
        std::vector<PickTarget> deduped;
        deduped.reserve(targets.size());
        for (const auto& t : targets) {
            bool overlaps = std::any_of(deduped.begin(), deduped.end(),
                [&](const PickTarget& k) { return std::hypot(t.x - k.x, t.y - k.y) < min_separation_m; });
            if (!overlaps) deduped.push_back(t);
        }
        targets = std::move(deduped);
    }

    // Two equal-radius cylinders touch when their centers are exactly
    // 2*radius apart; cylinder_overlap_margin_m_ adds clearance on top so
    // spawned collision objects aren't razor-thin adjacent. Dedup runs
    // before per-row radius scaling is assigned (cylinderRadiusForRow), so
    // this uses the largest radius any row could actually get - otherwise an
    // edge row (scaled up) could still end up closer than its true diameter.
    double minCylinderSeparation() {
        const auto params = param_listener_->get_params();
        const double base_radius = params.object_dimensions.at(1);
        const double max_scale = std::max({ cylinder_size_edge_scale_, cylinder_size_center_scale_, 1.0 });
        return 2.0 * base_radius * max_scale + cylinder_overlap_margin_m_;
    }

    // Radius taper by row: cylinder_size_edge_scale_ at row 0 and max_row,
    // cylinder_size_center_scale_ at the middle row, linearly interpolated
    // in between by how far (normalized) this row is from the middle.
    // max_row == 0 (a single-row grid) always returns the base radius, since
    // "edge" and "middle" are the same row there.
    double cylinderRadiusForRow(int row, int max_row) {
        const auto params = param_listener_->get_params();
        const double base_radius = params.object_dimensions.at(1);
        if (max_row <= 0) return base_radius;

        const double t = static_cast<double>(row) / static_cast<double>(max_row);  // 0..1
        const double dist_from_middle = std::fabs(t - 0.5) * 2.0;                  // 0 at middle, 1 at edges
        const double scale = cylinder_size_center_scale_ +
            (cylinder_size_edge_scale_ - cylinder_size_center_scale_) * dist_from_middle;
        return base_radius * scale;
    }

    // Cap total targets. Callers must already have targets in the desired
    // near-to-far sweep order, since this keeps the front and drops the rest.
    static void capPickTargets(std::vector<PickTarget>& targets, int max_targets) {
        if (static_cast<int>(targets.size()) > max_targets)
            targets.resize(static_cast<size_t>(std::max(0, max_targets)));
    }

    // ----------------------------------------------------------------
    //  Circular (radial) target generation
    //
    //  A center point plus 8 points on one ring at 45-degree increments
    //  (0/45/90/.../315), elliptically scaled by (rx, ry) so an anisotropic
    //  detection footprint (extents.x != extents.y) still gets covered -
    //  same idea as the rectangular grid, just radial. Out-of-reach ring
    //  points are projected inward toward the center like the square grid.
    //  Returned sorted near-to-far (ascending Y, smaller Y is nearer the arm
    //  in arm_base_link) so row assignment and pick order both preserve the
    //  "never reach back over an unpicked target" sweep property.
    // ----------------------------------------------------------------
    std::vector<PickTarget> generateCircularTargets(double center_x, double center_y,
                                                        double rx, double ry,
                                                        double min_separation_m)
        {
            rx = std::max(0.0, rx);
            ry = std::max(0.0, ry);

            RCLCPP_INFO(this->get_logger(),
                "Generating circular grid: rx %.2f m, ry %.2f m (9 raw targets)", rx, ry);

            // Nearest point (bottom of the ring, theta=270 deg) is lifted toward
            // the center by this fraction of ry, so it isn't as far out in front.
            constexpr double kNearestRaiseFrac = 0.15;   // 15% of ry, toward center
            // The near pair (lower-left / lower-right, theta=225/315) is pulled
            // together horizontally to this fraction of their nominal spread.
            constexpr double kNearPairXScale   = 0.85;   // 10% closer together
            constexpr double kNearPairRaiseFrac = 0.10;   // lift k5/k7 by 10% of ry, toward center
            constexpr double kFarPairInScale    = 0.85;   // pull k0/k1 15% toward center


            std::vector<PickTarget> targets;
            targets.reserve(9);
            targets.push_back({center_x, center_y, 0});  // row assigned below

            for (int k = 0; k < 8; ++k) {
                const double theta = k * (M_PI / 4.0);  // 0, 45, 90, ... 315 degrees
                double px = center_x + rx * std::cos(theta);
                double py = center_y + ry * std::sin(theta);

                // k == 6 is theta=270 (bottom, nearest the arm): raise it toward
                // center along +Y so it sits a little higher in the pattern.
                if (k == 6) {
                    py += kNearestRaiseFrac * ry;
                }
                // k == 5 (theta=225, lower-left) and k == 7 (theta=315, lower-right)
                // are the near pair: shrink their horizontal offset from center.
                if (k == 5 || k == 7) {
                    // px = center_x + (px - center_x) * kNearPairXScale;
                    py += kNearPairRaiseFrac * ry;
                }
                // k == 0 (theta=0, right) and k == 1 (theta=45, upper-right): pull inward
                // toward center along both axes. For k0 (Δy=0) this only moves X.
                if (k == 0 || k == 1) {
                    px = center_x + (px - center_x) * kFarPairInScale;
                    py = center_y + (py - center_y) * kFarPairInScale;
                }

                if (!isTargetReachable(px, py)) {
                    auto [proj_x, proj_y] = projectPointInward(px, py, center_x, center_y);
                    RCLCPP_INFO(this->get_logger(),
                        "Circular point at %.0f deg (%.3f,%.3f) out of reach - projected to (%.3f,%.3f)",
                        theta * 180.0 / M_PI, px, py, proj_x, proj_y);
                    px = proj_x; py = proj_y;
                }
                targets.push_back({px, py, 0});
            }

            dedupPickTargets(targets, min_separation_m);
            capPickTargets(targets, max_harvest_targets_);

            // Near-to-far by row (ascending Y), then RIGHT-TO-LEFT within a row
            // (descending X) so the sweep never reaches back over an unpicked
            // neighbour - same discipline as the square grid's column order.
            // kRowEps groups points that are the "same row" despite floating-point
            // noise from cos/sin and inward projection; without it the right/left
            // points of a row can differ by microns and skip the X tie-break.
            constexpr double kRowEps = 1e-3;  // 1 mm: treat |dy| < this as same row
            std::sort(targets.begin(), targets.end(),
                [](const PickTarget& a, const PickTarget& b) {
                    if (std::fabs(a.y - b.y) > kRowEps) return a.y < b.y;  // nearer first
                    return a.x > b.x;                                      // then right-to-left
                });
            for (size_t i = 0; i < targets.size(); ++i) targets[i].row = static_cast<int>(i);

            RCLCPP_INFO(this->get_logger(), "Generated %zu circular pick target(s)", targets.size());
            return targets;
        }

    // ----------------------------------------------------------------
    //  Square-grid target generation
    //
    //  A side_x (columns) by side_y (rows) rectangle, centred on
    //  (center_x, center_y), with pick points on a fixed
    //  grid_point_spacing_m lattice. Points per axis = floor(side/spacing)+1,
    //  symmetric about the center. Swept near-to-far (rows) and
    //  right-to-left (columns) so the arm never reaches back over an
    //  unpicked target. Out-of-reach points are projected inward toward
    //  the center.
    // ----------------------------------------------------------------
    std::vector<PickTarget> generateSquareGridTargets(double center_x, double center_y,
                                                       double side_x, double side_y,
                                                       double min_separation_m)
    {
        const double sx      = std::max(0.0, side_x);
        const double sy      = std::max(0.0, side_y);
        const double spacing = std::max(0.02, grid_point_spacing_m_);

        int nx = static_cast<int>(std::floor(sx / spacing)) + 1;  // columns
        int ny = static_cast<int>(std::floor(sy / spacing)) + 1;  // rows
        if (nx < 1) nx = 1;
        if (ny < 1) ny = 1;

        const double first_x = -0.45 * (nx - 1) * spacing;  // centre the lattice
        const double first_y = -0.45 * (ny - 1) * spacing;

        RCLCPP_INFO(this->get_logger(),
            "Generating %dx%d grid: %.2f x %.2f m, spacing %.2f m (%d raw targets)",
            nx, ny, sx, sy, spacing, nx * ny);

        std::vector<PickTarget> targets;
        targets.reserve(static_cast<size_t>(nx) * ny);

        // Row r = 0 nearest the arm (smaller Y is nearer in arm_base_link).
        for (int r = 0; r < ny; ++r) {
            double py = center_y + first_y + r * spacing;
            for (int c = nx - 1; c >= 0; --c) {      // columns right-to-left
                double px = center_x + first_x + c * spacing;

                if (!isTargetReachable(px, py)) {
                    auto [proj_x, proj_y] = projectPointInward(px, py, center_x, center_y);
                    RCLCPP_INFO(this->get_logger(),
                        "Grid point (r%d,c%d) at (%.3f,%.3f) out of reach - projected to (%.3f,%.3f)",
                        r, c, px, py, proj_x, proj_y);
                    px = proj_x; py = proj_y;
                }
                targets.push_back({px, py, r});
            }
        }

        // Discard overlapping cylinders (incl. coincident points created by
        // inward projection), then cap (keeps near rows, drops farthest -
        // already ordered near-to-far).
        dedupPickTargets(targets, min_separation_m);
        capPickTargets(targets, max_harvest_targets_);

        RCLCPP_INFO(this->get_logger(), "Generated %zu square-grid pick target(s)", targets.size());
        return targets;
    }

    // Grid rectangle/ellipse is sized from the detection's physical extents
    // when available (clamped to [extents_min_m, extents_max_m] to guard
    // against a noisy/implausible measurement); otherwise falls back to the
    // fixed center_box_size_m square. Corner fields remain unused.
    //
    // grid_shape_ == "circular" only commits to the circular result if at
    // least circular_min_reachable of its 9 points survived the reachability
    // check; otherwise it falls back to the square grid so a goal never ends
    // up with fewer targets than the square-grid strategy alone would give.
    std::vector<PickTarget> generatePickTargets(
        const status_interfaces::msg::ImageDetection& det,
        double center_x, double center_y)
    {
        double side_x = center_box_size_m_;
        double side_y = center_box_size_m_;

        if (det.extents_valid) {
            double ex, ey;
            if (transformExtentsToArmBase(det.extents, ex, ey)) {
                side_x = std::clamp(ex, extents_min_m_, extents_max_m_);
                side_y = std::clamp(ey, extents_min_m_, extents_max_m_);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Extents valid but TF transform failed - falling back to fixed %.2f m grid",
                    center_box_size_m_);
            }
        }

        const double min_separation_m = minCylinderSeparation();

        if (grid_shape_ == "circular") {
            auto circular = generateCircularTargets(center_x, center_y, side_x / 2.0, side_y / 2.0,
                                                     min_separation_m);
            const int required = std::max(1, circular_min_reachable_);
            if (static_cast<int>(circular.size()) >= required) {
                return circular;
            }
            RCLCPP_WARN(this->get_logger(),
                "Circular grid only produced %zu reachable target(s) (need %d) - "
                "falling back to square grid",
                circular.size(), required);
        }

        return generateSquareGridTargets(center_x, center_y, side_x, side_y, min_separation_m);
    }

    // ----------------------------------------------------------------
    //  Recovery helpers
    // ----------------------------------------------------------------

    void moveToHome() {
        RCLCPP_INFO(this->get_logger(), "Moving arm to home after failure...");
        auto home_task = std::make_unique<moveit_task_constructor_demo::MoveToConfigTask>("move_home_recovery");
        if (!home_task->setTargetConfig("home_0")) {
            RCLCPP_ERROR(this->get_logger(), "home_0 config not found - skipping recovery move.");
            return;
        }
        if (!home_task->init(this->shared_from_this(), param_listener_->get_params())) {
            RCLCPP_ERROR(this->get_logger(), "Home recovery task init failed.");
            return;
        }
        if (home_task->plan(3)) {
            ActiveTaskGuard<moveit_task_constructor_demo::MoveToConfigTask> guard(this, *home_task);
            home_task->execute();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not plan path to home - manual recovery may be needed.");
        }
    }

    // A fresh MoveToConfigTask (setTargetConfig+init+plan) per attempt, not
    // just a re-execute - once execution fails (especially due to a fault)
    // the arm's real state no longer matches the stale plan's assumed start
    // state, so re-running the same trajectory isn't safe. On a fault, the
    // fault is cleared before the next attempt; if it can't be cleared,
    // there's no point retrying and we stop immediately.
    bool moveToDrop() {
        int max_attempts = 1 + std::max(0, execution_retry_attempts_);
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(this->get_logger(),
                "Moving arm to drop/bin pose to release object (attempt %d/%d)...",
                attempt, max_attempts);

            auto drop_task = std::make_unique<moveit_task_constructor_demo::MoveToConfigTask>("move_drop_recovery");
            if (!drop_task->setTargetConfig("drop")) {
                RCLCPP_ERROR(this->get_logger(), "drop config not found - cannot reach bin.");
                return false;
            }
            if (!drop_task->init(this->shared_from_this(), param_listener_->get_params())) {
                RCLCPP_ERROR(this->get_logger(), "Drop recovery task init failed.");
                return false;
            }
            if (!drop_task->plan(3)) {
                RCLCPP_ERROR(this->get_logger(),
                    "Could not plan path to drop pose (attempt %d/%d).", attempt, max_attempts);
            } else if (executeAndVerify(*drop_task)) {   // also catches a fault during the move
                RCLCPP_INFO(this->get_logger(), "Reached bin.");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "Drop-pose execution failed / arm faulted (attempt %d/%d).", attempt, max_attempts);
            }

            if (attempt == max_attempts) break;

            if (arm_in_fault_.load() && !tryResetFault()) {
                RCLCPP_ERROR(this->get_logger(), "Fault could not be cleared - not retrying drop.");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        return false;
    }

    // Returns true if the target ended up safely resolved - either it was never
    // attached, or it was attached and got successfully dropped in the bin with
    // the gripper confirmed open. Returns false if it was attached but release
    // couldn't be confirmed (never reached the bin, or the gripper wouldn't
    // open) - the plant may still be physically held. Either way, its collision
    // object is always removed from the scene.
    bool cleanupFailedPickTarget(const std::string& object_id) {
        bool released = true;
        if (moveit_task_constructor_demo::isObjectAttached(object_id)) {
            RCLCPP_WARN(this->get_logger(),
                "Object '%s' is attached after failure - moving to bin before releasing", object_id.c_str());

            if (moveToDrop()) {
                // Open the gripper ONLY after we've actually arrived at the bin.
                if (!moveit_task_constructor_demo::openGripper(
                        this->shared_from_this(), param_listener_->get_params())) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Reached bin but could not open gripper - check the hardware.");
                    released = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Released '%s' into the bin.", object_id.c_str());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "Did not reach bin - NOT opening gripper here to avoid dropping '%s' in a bad spot.",
                    object_id.c_str());
                // Object stays physically held; we still clear it from the scene below.
                released = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Object '%s' failed before attach - removing it from the scene", object_id.c_str());
        }
        moveit_task_constructor_demo::detachAndRemoveObject(object_id);
        return released;
    }

    bool tryResetFault(int timeout_sec = 5) {
        if (!arm_in_fault_.load()) return true;
        RCLCPP_WARN(this->get_logger(), "Arm is in FAULT - attempting programmatic reset...");

        if (!fault_reset_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "fault_controller/reset_fault service not available.");
            return false;
        }
        auto req    = std::make_shared<example_interfaces::srv::Trigger::Request>();
        auto future = fault_reset_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Fault reset service call timed out.");
            return false;
        }
        auto response = future.get();
        bool ok = response->success;
        if (ok) {
            RCLCPP_INFO(this->get_logger(), "Fault reset accepted.");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            if (arm_in_fault_.load()) {
                RCLCPP_ERROR(this->get_logger(), "Reset service succeeded but arm still reports FAULT.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fault reset returned failure: %s", response->message.c_str());
        }
        return ok;
    }

    template <typename TaskT>
    bool executeAndVerify(TaskT& task) {
        ActiveTaskGuard<TaskT> guard(this, task);
        if (!task.execute()) return false;
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        if (arm_in_fault_.load()) {
            RCLCPP_ERROR(this->get_logger(),
                "Trajectory reported SUCCESS but arm is in FAULT - treating as execution failure.");
            return false;
        }
        return true;
    }

    // Called when arm_in_fault_ is observed during target i's execution.
    // Tries to reset the fault and safely release the current target (move
    // to bin, open gripper, clear its collision object). If that fully
    // succeeds, the harvest can resume with the next target instead of
    // aborting the whole goal - returns true in that case, having already
    // sent FAULT_RECOVERED feedback and moved the arm home. If the fault
    // can't be cleared, or release couldn't be confirmed, it's not safe to
    // just carry on (arm may still be faulted or still holding the plant) -
    // returns false, having already aborted the goal.
    bool handleFaultAndMaybeContinue(const std::shared_ptr<GoalHandleManipulator>& goal_handle,
                                     const std::shared_ptr<Manipulator::Result>& result,
                                     int harvested, int total, const std::string& object_id)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Arm FAULTED during execution - attempting fault reset and cleanup...");

        if (tryResetFault() && cleanupFailedPickTarget(object_id)) {
            RCLCPP_WARN(this->get_logger(),
                "Fault cleared and '%s' safely released - resuming harvest with the next target.",
                object_id.c_str());
            moveToHome();
            send_feedback(goal_handle, Manipulator::Feedback::FAULT_RECOVERED);
            return true;
        }

        RCLCPP_ERROR(this->get_logger(),
            "Could not fully recover from the fault (reset and/or release failed) - aborting goal.");
        send_feedback(goal_handle, Manipulator::Feedback::FAILED);
        result->success = false;
        result->harvested_count = harvested;
        result->total_targets = total;
        goal_handle->abort(result);
        return false;
    }

    void moveToReady() {
        RCLCPP_INFO(this->get_logger(), "Moving arm to ready pose before replanning...");
        auto ready_task = std::make_unique<moveit_task_constructor_demo::MoveToConfigTask>("move_ready_recovery");
        if (!ready_task->setTargetConfig("pre_cut_1")) {
            RCLCPP_ERROR(this->get_logger(), "pre_cut_1 config not found - skipping ready-pose recovery.");
            return;
        }
        if (!ready_task->init(this->shared_from_this(), param_listener_->get_params())) {
            RCLCPP_ERROR(this->get_logger(), "Ready-pose recovery task init failed.");
            return;
        }
        if (ready_task->plan(3)) {
            ActiveTaskGuard<moveit_task_constructor_demo::MoveToConfigTask> guard(this, *ready_task);
            ready_task->execute();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not plan path to ready pose - retrying from current pose.");
        }
    }

    std::unique_ptr<moveit_task_constructor_demo::PickPlaceTask> replanPickTarget(
        const manipulator_action_server::Params& params, int target_index, bool is_last,
        const std::vector<std::string>& neighbor_ids, double grasp_angle_delta)
    {
        auto task = std::make_unique<moveit_task_constructor_demo::PickPlaceTask>(
            "det_harvest_" + std::to_string(target_index) + "_recovery");
        if (!task->init(this->shared_from_this(), params, is_last, neighbor_ids, grasp_angle_delta)) {
            RCLCPP_ERROR(this->get_logger(), "Recovery task init failed for pick target %d", target_index + 1);
            return nullptr;
        }
        if (!task->plan(params.max_solutions)) {
            RCLCPP_ERROR(this->get_logger(), "Recovery planning failed for pick target %d", target_index + 1);
            return nullptr;
        }
        return task;
    }

    // ----------------------------------------------------------------
    //  Feedback helpers
    // ----------------------------------------------------------------

    void send_feedback(const std::shared_ptr<GoalHandleManipulator> goal_handle, int8_t status) {
        if (!rclcpp::ok() || !goal_handle->is_active()) return;
        try {
            auto feedback = std::make_shared<Manipulator::Feedback>();
            feedback->status = status;
            goal_handle->publish_feedback(feedback);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not publish feedback (%s) - continuing", e.what());
        }
    }

    void send_feedback(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        send_feedback(goal_handle, manipulator_status_);
    }

    // ----------------------------------------------------------------
    //  Action server callbacks
    // ----------------------------------------------------------------

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Manipulator::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        this->manipulator_status_ = Manipulator::Feedback::IDLE;
        send_feedback(goal_handle);

        // Tracked (not detached) so ~ManipulatorActionServer() can join
        // before the node - and everything a task's execute() depends on -
        // gets destroyed out from under it.
        auto done = std::make_shared<std::atomic<bool>>(false);
        std::thread worker([this, goal_handle, done]() {
            doTask(goal_handle);
            done->store(true);
        });

        std::lock_guard<std::mutex> lock(threads_mutex_);
        worker_threads_.erase(
            std::remove_if(worker_threads_.begin(), worker_threads_.end(),
                [](WorkerEntry& entry) {
                    if (!entry.done->load()) return false;
                    if (entry.thread.joinable()) entry.thread.join();
                    return true;
                }),
            worker_threads_.end());
        worker_threads_.push_back({std::move(worker), done});
    }

    // ----------------------------------------------------------------
    //  IMAGE_DETECTION_HARVEST handler
    // ----------------------------------------------------------------

    void executeImageDetectionHarvest(const std::shared_ptr<GoalHandleManipulator>& goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Manipulator::Result>();

        // 1+2. Acquire a detection whose center passes every gate.
        send_feedback(goal_handle, Manipulator::Feedback::PLANNING);
        RCLCPP_INFO(this->get_logger(), "Awaiting detection on: %s", goal->extra_data.c_str());

        double center_x, center_y;
        auto detection = acquireValidDetection(goal->extra_data, goal_handle, center_x, center_y);

        if (!detection) {
            RCLCPP_ERROR(this->get_logger(),
                "No detection with an acceptable center after %d attempt(s) - aborting",
                std::max(1, center_acquire_attempts_));
            send_feedback(goal_handle, Manipulator::Feedback::FAILED);
            result->success = false; result->harvested_count = 0; result->total_targets = 0;
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Center in arm_base_link: (%.3f, %.3f, %.3f)",
            center_x, center_y, PICK_Z_HEIGHT);

        // 3. Generate the square grid around the center. (Only the center is
        //    needed, so there is no corner-retry / fallback path here.)
        auto pick_targets = generatePickTargets(*detection, center_x, center_y);

        int total_targets = static_cast<int>(pick_targets.size());
        if (total_targets == 0) {
            RCLCPP_ERROR(this->get_logger(), "No pick targets could be generated - aborting");
            send_feedback(goal_handle, Manipulator::Feedback::FAILED);
            result->success = false; result->harvested_count = 0; result->total_targets = 0;
            goal_handle->abort(result);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Generated %d pick target(s)", total_targets);

        // 4. Build id/pose/radius lists.
        auto params = param_listener_->get_params();
        int success_count = 0;
        std::vector<std::string> grid_object_ids;
        std::vector<geometry_msgs::msg::Pose> grid_object_poses;
        std::vector<double> grid_object_radii;
        grid_object_ids.reserve(total_targets);
        grid_object_poses.reserve(total_targets);
        grid_object_radii.reserve(total_targets);

        int max_row = 0;
        for (const auto& t : pick_targets) max_row = std::max(max_row, t.row);

        for (int i = 0; i < total_targets; ++i) {
            grid_object_ids.push_back(params.object_name + "_" + std::to_string(i));
            geometry_msgs::msg::Pose pose;
            pose.position.x = pick_targets[i].x;
            pose.position.y = pick_targets[i].y;
            pose.position.z = PICK_Z_HEIGHT;
            pose.orientation.w = 1.0;
            grid_object_poses.push_back(pose);
            grid_object_radii.push_back(cylinderRadiusForRow(pick_targets[i].row, max_row));
        }

        if (!harvest_single_target_mode_) {
            moveit_task_constructor_demo::setupHarvestGridScene(
                params, grid_object_poses, grid_object_ids, grid_object_radii);
        }

        const std::vector<std::string> neighbor_ids =
            harvest_single_target_mode_ ? std::vector<std::string>{} : grid_object_ids;

        RCLCPP_INFO(this->get_logger(), "Scene strategy: %s (%zu neighbour id(s))",
            harvest_single_target_mode_ ? "ONE object at a time" : "full grid",
            neighbor_ids.size());

        // 5. Execute pick/place for each target.
        for (int i = 0; i < total_targets; ++i) {
            if (shutting_down_.load()) {
                // Context is going away - don't touch goal_handle, just stop.
                RCLCPP_WARN(this->get_logger(), "Node is shutting down - stopping harvest.");
                return;
            }
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal cancelled during detection harvest");
                result->success = false; result->harvested_count = success_count;
                result->total_targets = total_targets;
                goal_handle->canceled(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Pick target %d/%d: (%.3f, %.3f, %.3f)",
                i + 1, total_targets, pick_targets[i].x, pick_targets[i].y, PICK_Z_HEIGHT);

            params.object_name = grid_object_ids[i];

            // Grasp yaw policy for this target, taken from the row tag that was
            // attached at generation time (index-independent).
            const int    target_row = pick_targets[i].row;
            const double base_delta = graspAngleDeltaForRow(target_row);
            RCLCPP_INFO(this->get_logger(),
                "Target %d is in row %d -> angle_delta %.4f rad (%d yaw candidate(s))",
                i + 1, target_row, base_delta, yawCandidateCount(base_delta));

            if (harvest_single_target_mode_) {
                moveit_task_constructor_demo::setupSingleTargetScene(
                    params, grid_object_poses[i], grid_object_ids[i], grid_object_radii[i]);
            }

            bool is_last = (i == total_targets - 1);
            bool planned = false;
            std::unique_ptr<moveit_task_constructor_demo::PickPlaceTask> pick_place_task;

            int max_attempts = std::max(1, harvest_replan_attempts_);
            for (int attempt = 1; attempt <= max_attempts; ++attempt) {
                send_feedback(goal_handle, Manipulator::Feedback::PLANNING);

                // Attempt 1 uses the row's delta. Later attempts widen to the
                // far-row delta: re-running init() with the SAME delta re-samples
                // the identical yaw set, so a single-yaw row would otherwise
                // burn every attempt on the exact same infeasible pose.
                const double attempt_delta =
                    (attempt == 1 || !widen_yaw_on_replan_) ? base_delta
                                                            : grasp_angle_delta_widen_;
                if (attempt > 1 && attempt_delta != base_delta) {
                    RCLCPP_INFO(this->get_logger(),
                        "Widening yaw sampling for target %d: %.4f -> %.4f rad (%d candidate(s))",
                        i + 1, base_delta, attempt_delta, yawCandidateCount(attempt_delta));
                }

                pick_place_task = std::make_unique<moveit_task_constructor_demo::PickPlaceTask>(
                    "det_harvest_" + std::to_string(i) + "_a" + std::to_string(attempt));

                if (!pick_place_task->init(this->shared_from_this(), params, is_last,
                                           neighbor_ids, attempt_delta)) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Task init failed for pick target %d (attempt %d/%d)", i + 1, attempt, max_attempts);
                } else if (pick_place_task->plan(params.max_solutions)) {
                    planned = true;
                    break;
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Planning failed for pick target %d (attempt %d/%d)", i + 1, attempt, max_attempts);
                }

                if (attempt < max_attempts) {
                    RCLCPP_INFO(this->get_logger(), "Retrying pick target %d from ready pose", i + 1);
                    moveToReady();
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                }
            }

            if (!planned) {
                RCLCPP_ERROR(this->get_logger(),
                    "Planning failed for pick target %d after %d attempt(s) - removing its object and skipping",
                    i + 1, max_attempts);
                cleanupFailedPickTarget(params.object_name);
                continue;
            }

            send_feedback(goal_handle, Manipulator::Feedback::MOVING);

            bool executed = false;
            bool fault_recovered = false;
            int max_execute_attempts = 1 + std::max(0, execution_retry_attempts_);
            for (int exec_attempt = 1; exec_attempt <= max_execute_attempts; ++exec_attempt) {
                if (executeAndVerify(*pick_place_task)) { executed = true; break; }

                RCLCPP_ERROR(this->get_logger(),
                    "Execution failed for pick target %d (attempt %d/%d)", i + 1, exec_attempt, max_execute_attempts);

                if (arm_in_fault_.load()) {
                    if (handleFaultAndMaybeContinue(goal_handle, result, success_count, total_targets,
                                                    params.object_name)) {
                        fault_recovered = true;
                        break;
                    }
                    return;
                }
                if(moveit_task_constructor_demo::isObjectAttached(params.object_name)) {
                        RCLCPP_WARN(this->get_logger(),
                        "'%s' was attached mid-execution - stale solution can't be safely "
                        "re-sent, stopping retries for this attempt.", params.object_name.c_str());
                        break;

                }
                if (exec_attempt < max_execute_attempts)
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }

            if (fault_recovered) {
                // Target already cleaned up and safely released inside
                // handleFaultAndMaybeContinue() - move on to the next one.
                continue;
            }

            if (shutting_down_.load()) {
                // Execution was cancelled by the shutdown handler, not a real
                // failure - the arm is stopping. Skip home/replan recovery
                // (it would just try to move against a dying context) and return.
                RCLCPP_WARN(this->get_logger(), "Node is shutting down - stopping harvest without recovery.");
                return;
            }

            if (!executed) {
                RCLCPP_WARN(this->get_logger(),
                    "Pick target %d still failing after %d attempt(s) - returning home and replanning",
                    i + 1, max_execute_attempts);
                moveToHome();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                if (harvest_single_target_mode_) {
                    moveit_task_constructor_demo::setupSingleTargetScene(
                        params, grid_object_poses[i], grid_object_ids[i], grid_object_radii[i]);
                }

                // Last-ditch replan always uses the wider sampling — at this
                // point extra planning time is cheaper than losing the plant.
                auto recovery_task = replanPickTarget(params, i, is_last, neighbor_ids,
                                                      grasp_angle_delta_widen_);
                if (recovery_task) executed = executeAndVerify(*recovery_task);

                if (!executed) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Recovery failed for pick target %d - cleaning up and skipping", i + 1);
                    if (arm_in_fault_.load()) {
                        if (handleFaultAndMaybeContinue(goal_handle, result, success_count, total_targets,
                                                        params.object_name)) {
                            continue;
                        }
                        return;
                    }
                    cleanupFailedPickTarget(params.object_name);
                    moveToHome();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }

            if (executed) {
                RCLCPP_INFO(this->get_logger(), "Pick target %d succeeded", i + 1);
                moveit_task_constructor_demo::removeHarvestedObject(params.object_name);
                send_feedback(goal_handle, Manipulator::Feedback::MOVING_COMPLETE);
                success_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }

        // 6. Report.
        float pct = total_targets > 0 ? (100.0f * success_count / total_targets) : 0.0f;
        RCLCPP_INFO(this->get_logger(), "IMAGE DETECTION HARVEST complete: %d/%d (%.0f%%)",
            success_count, total_targets, pct);
        result->harvested_count = success_count;
        result->total_targets = total_targets;
        result->success = (success_count > 0);

        if (success_count > 0) {
            send_feedback(goal_handle, Manipulator::Feedback::IDLE);
            goal_handle->succeed(result);
        } else {
            send_feedback(goal_handle, Manipulator::Feedback::FAILED);
            goal_handle->abort(result);
        }
    }

    // ----------------------------------------------------------------
    //  Main task dispatcher
    // ----------------------------------------------------------------

    void doTask(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Manipulator::Result>();
        if (!rclcpp::ok()) return;

        kinova_task_manager::ManipulatorCommand cmd =
            kinova_task_manager::stringToCommand(goal->arm_task);
        RCLCPP_INFO(this->get_logger(), "Initializing Task: %s", goal->arm_task.c_str());

        std::string target_config = "";
        switch (cmd) {
            case kinova_task_manager::ManipulatorCommand::GO_STOW:  target_config = "stow"; break;
            case kinova_task_manager::ManipulatorCommand::GO_READY: target_config = "image_detection"; break;
            case kinova_task_manager::ManipulatorCommand::GO_DROP:  target_config = "drop"; break;
            default: break;
        }

        if (!target_config.empty()) {
            manipulator_status_ = Manipulator::Feedback::PLANNING;
            send_feedback(goal_handle);

            auto action_task = std::make_unique<moveit_task_constructor_demo::MoveToConfigTask>("move_to_config_task");
            if (!action_task->setTargetConfig(target_config)) {
                RCLCPP_ERROR(this->get_logger(), "The target config does not exist");
                manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                result->success = false; goal_handle->abort(result); return;
            }
            if (!action_task->init(this->shared_from_this(), param_listener_->get_params())) {
                RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                result->success = false; goal_handle->abort(result); return;
            }
            if (action_task->plan(5)) {
                RCLCPP_INFO(this->get_logger(), "Planning succeeded");
                manipulator_status_ = Manipulator::Feedback::MOVING; send_feedback(goal_handle);
                if (executeAndVerify(*action_task)) {
                    RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                    manipulator_status_ = Manipulator::Feedback::MOVING_COMPLETE; send_feedback(goal_handle);
                    manipulator_status_ = Manipulator::Feedback::IDLE; send_feedback(goal_handle);
                    result->success = true; goal_handle->succeed(result);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Execution failed");
                    manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                    result->success = false; goal_handle->abort(result);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Planning failed");
                manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                result->success = false; goal_handle->abort(result);
            }
            return;
        }

        switch (cmd) {
            case kinova_task_manager::ManipulatorCommand::MOVE_EEF:
            {
                moveit_task_constructor_demo::clearPlanningScene();
                auto action_task = std::make_unique<moveit_task_constructor_demo::MoveEefTask>("move_eef_task");
                if (!action_task->setTargetPose(goal->target_eef_pose)) {
                    RCLCPP_ERROR(this->get_logger(), "The target pose cannot be reached");
                    manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                    result->success = false; goal_handle->abort(result); return;
                }
                if (!action_task->init(this->shared_from_this(), param_listener_->get_params())) {
                    RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                    manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                    result->success = false; goal_handle->abort(result); return;
                }
                if (action_task->plan(5)) {
                    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
                    manipulator_status_ = Manipulator::Feedback::MOVING; send_feedback(goal_handle);
                    if (executeAndVerify(*action_task)) {
                        RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                        manipulator_status_ = Manipulator::Feedback::MOVING_COMPLETE; send_feedback(goal_handle);
                        result->success = true; goal_handle->succeed(result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Execution failed");
                        manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                        result->success = false; goal_handle->abort(result);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed");
                    manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                    result->success = false; goal_handle->abort(result);
                }
                break;
            }

            case kinova_task_manager::ManipulatorCommand::START_HARVEST:
            {
                const auto params = param_listener_->get_params();
                int num_of_actions = 3;
                bool isLastAction = false;

                for (int i = 0; i < num_of_actions; i++) {
                    if (shutting_down_.load()) {
                        RCLCPP_WARN(this->get_logger(), "Node is shutting down - stopping harvest.");
                        return;
                    }
                    manipulator_status_ = Manipulator::Feedback::PLANNING; send_feedback(goal_handle);
                    moveit_task_constructor_demo::setupDemoScene(params);

                    moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
                    isLastAction = (i == num_of_actions - 1);

                    // No grid here (single object from params), so use the
                    // far-row delta: the wider, more forgiving sampling.
                    if (!pick_place_task.init(this->shared_from_this(), params, isLastAction,
                                              {}, grasp_angle_delta_far_)) {
                        RCLCPP_ERROR(this->get_logger(), "Task initialization failed");
                        manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                        result->success = false; goal_handle->abort(result); return;
                    }
                    if (pick_place_task.plan(params.max_solutions)) {
                        RCLCPP_INFO(this->get_logger(), "Planning succeeded");
                        manipulator_status_ = Manipulator::Feedback::MOVING; send_feedback(goal_handle);
                        if (executeAndVerify(pick_place_task)) {
                            RCLCPP_INFO(this->get_logger(), "Execution succeeded");
                            manipulator_status_ = Manipulator::Feedback::MOVING_COMPLETE; send_feedback(goal_handle);
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Execution failed");
                            if (arm_in_fault_.load()) {
                                if (!tryResetFault()) {
                                    RCLCPP_ERROR(this->get_logger(), "Arm fault could not be cleared.");
                                    manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                                    result->success = false; goal_handle->abort(result); return;
                                }
                            }
                            cleanupFailedPickTarget(params.object_name);
                            moveToHome();
                            manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                            result->success = false; goal_handle->abort(result); return;
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Planning failed");
                        manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                        result->success = false; goal_handle->abort(result); return;
                    }
                    RCLCPP_INFO(this->get_logger(), "Harvesting action %d succeeded", i + 1);
                }

                RCLCPP_INFO(this->get_logger(), "All harvesting actions succeeded");
                manipulator_status_ = Manipulator::Feedback::IDLE; send_feedback(goal_handle);
                result->success = true; goal_handle->succeed(result);
                break;
            }

            case kinova_task_manager::ManipulatorCommand::IMAGE_DETECTION_HARVEST:
                executeImageDetectionHarvest(goal_handle);
                return;

            case kinova_task_manager::ManipulatorCommand::UNKNOWN:
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown task received: %s", goal->arm_task.c_str());
                manipulator_status_ = Manipulator::Feedback::FAILED; send_feedback(goal_handle);
                result->success = false; goal_handle->abort(result);
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