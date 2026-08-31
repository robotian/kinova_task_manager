#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "status_interfaces/action/manipulator_task.hpp"

/**
 * Two-step harvest client:
 *   Step 1 — GO DETECT              : move arm to tag_detection pose
 *   Step 2 — IMAGE DETECTION HARVEST: read detection, transform, pick all reachable targets
 *
 * Usage:
 *   ros2 run kinova_task_manager test_harvest_image_client \
 *     --ros-args -r __ns:=/a200_0284 \
 *                -p detection_topic:=/a200_0284/manipulators/arm_detection/image_annotated/detection_data
 */
class HarvestImageClient : public rclcpp::Node {
public:
    using Manipulator = status_interfaces::action::ManipulatorTask;
    using GoalHandleManipulator = rclcpp_action::ClientGoalHandle<Manipulator>;

    enum class Step { GO_READY, HARVEST, DONE };

    explicit HarvestImageClient(const rclcpp::NodeOptions& options)
    : Node("test_harvest_image_client", options),
      step_(Step::GO_READY)
    {
        detection_topic_ = this->declare_parameter<std::string>(
            "detection_topic",
            "/a200_0284/manipulators/arm_detection/image_annotated/detection_data");

        RCLCPP_INFO(this->get_logger(), "Detection topic: %s", detection_topic_.c_str());

        client_ptr_ = rclcpp_action::create_client<Manipulator>(
            this, "manipulator_action_server");

        // Brief delay then kick off step 1
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&HarvestImageClient::send_current_step, this));
    }

private:
    rclcpp_action::Client<Manipulator>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string detection_topic_;
    Step step_;

    // ----------------------------------------------------------------
    //  Send goal for whichever step we're on
    // ----------------------------------------------------------------
    void send_current_step() {
        timer_->cancel();

        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after 10 s");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = Manipulator::Goal();

        if (step_ == Step::GO_READY) {
            goal_msg.arm_task = "GO READY";
            RCLCPP_INFO(this->get_logger(),
                "[Step 1/2] Sending GO READY — moving arm to detection pose");
        } else {
            goal_msg.arm_task   = "IMAGE DETECTION HARVEST";
            goal_msg.extra_data = detection_topic_;
            RCLCPP_INFO(this->get_logger(),
                "[Step 2/2] Sending IMAGE DETECTION HARVEST — topic: %s",
                detection_topic_.c_str());
        }

        auto opts = rclcpp_action::Client<Manipulator>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&HarvestImageClient::on_goal_response, this, std::placeholders::_1);
        opts.feedback_callback =
            std::bind(&HarvestImageClient::on_feedback, this,
                      std::placeholders::_1, std::placeholders::_2);
        opts.result_callback =
            std::bind(&HarvestImageClient::on_result, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, opts);
    }

    // ----------------------------------------------------------------
    //  Callbacks
    // ----------------------------------------------------------------
    void on_goal_response(const GoalHandleManipulator::SharedPtr& gh) {
        if (!gh) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted — waiting for result...");
        }
    }

    void on_feedback(GoalHandleManipulator::SharedPtr,
                     const std::shared_ptr<const Manipulator::Feedback> fb) {
        RCLCPP_INFO(this->get_logger(), "[Feedback] %s", status_name(fb->status).c_str());
    }

    void on_result(const GoalHandleManipulator::WrappedResult& res) {
        if (step_ == Step::GO_READY) {
            // ── Step 1 result ──────────────────────────────────────────
            if (res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result->success) {
                RCLCPP_INFO(this->get_logger(),
                    "[Step 1/2] GO_READY succeeded — arm at detection pose");
                step_ = Step::HARVEST;
                // Small pause, then send harvest goal
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(300),
                    std::bind(&HarvestImageClient::send_current_step, this));
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "[Step 1/2] GO_READY FAILED (code=%d) — aborting harvest",
                    static_cast<int>(res.code));
                rclcpp::shutdown();
            }
        } else {
            // ── Step 2 result ──────────────────────────────────────────
            switch (res.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                        "[Step 2/2] HARVEST SUCCEEDED — picked %d / %d targets (%.0f%%)",
                        res.result->harvested_count,
                        res.result->total_targets,
                        res.result->total_targets > 0
                            ? (100.0 * res.result->harvested_count / res.result->total_targets)
                            : 0.0);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(),
                        "[Step 2/2] HARVEST ABORTED — picked %d / %d targets",
                        res.result->harvested_count, res.result->total_targets);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(),
                        "[Step 2/2] HARVEST CANCELLED — picked %d / %d targets",
                        res.result->harvested_count, res.result->total_targets);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
            }
            rclcpp::shutdown();
        }
    }

    // ----------------------------------------------------------------
    //  Helpers
    // ----------------------------------------------------------------
    static std::string status_name(int8_t s) {
        switch (s) {
            case Manipulator::Feedback::IDLE:            return "IDLE";
            case Manipulator::Feedback::PLANNING:        return "PLANNING";
            case Manipulator::Feedback::MOVING:          return "MOVING";
            case Manipulator::Feedback::MOVING_COMPLETE: return "MOVING_COMPLETE";
            case Manipulator::Feedback::FAILED:          return "FAILED";
            default:                                     return "UNKNOWN";
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HarvestImageClient>(rclcpp::NodeOptions()));
    return 0;
}