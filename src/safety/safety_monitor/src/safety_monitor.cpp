#include <format>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_path_limits.hpp"
#include "common_msgs/msg/huat_stop.hpp"
#include "safety_monitor/stop_state_machine.h"

class SafetyMonitor {
   public:
    SafetyMonitor() {
        node_ = rclcpp::Node::make_shared("safety_monitor");

        double planner_timeout = 0.5;
        std::string pathlimits_topic, vehicle_state_topic, stop_topic, stop_request_topic, reset_stop_topic;

        node_->declare_parameter("planner_timeout", 0.5);
        node_->declare_parameter("pathlimits_topic", "/planning/pathlimits");
        node_->declare_parameter("vehicle_state_topic", "/localization/vehicle_state");
        node_->declare_parameter("stop_topic", "/system/stop");
        node_->declare_parameter("stop_request_topic", "");
        node_->declare_parameter("reset_stop_topic", "");

        node_->get_parameter("planner_timeout", planner_timeout);
        node_->get_parameter("pathlimits_topic", pathlimits_topic);
        node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
        node_->get_parameter("stop_topic", stop_topic);
        node_->get_parameter("stop_request_topic", stop_request_topic);
        node_->get_parameter("reset_stop_topic", reset_stop_topic);

        planner_timeout_ = rclcpp::Duration::from_seconds(planner_timeout);
        has_vehicle_state_ = false;
        has_nonempty_pathlimits_ = false;

        sub_pathlimits_ = node_->create_subscription<common_msgs::msg::HuatPathLimits>(
            pathlimits_topic, 10,
            std::bind(&SafetyMonitor::OnPathLimits, this, std::placeholders::_1));
        sub_vehicle_state_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
            vehicle_state_topic, 10,
            std::bind(&SafetyMonitor::OnVehicleState, this, std::placeholders::_1));
        if (!stop_request_topic.empty()) {
            sub_stop_request_ = node_->create_subscription<common_msgs::msg::HuatStop>(
                stop_request_topic, 10,
                std::bind(&SafetyMonitor::OnStopRequest, this, std::placeholders::_1));
        }
        if (!reset_stop_topic.empty()) {
            sub_reset_ = node_->create_subscription<common_msgs::msg::HuatStop>(
                reset_stop_topic, 1,
                std::bind(&SafetyMonitor::OnResetStop, this, std::placeholders::_1));
            RCLCPP_INFO(node_->get_logger(), "%s",
                        std::format("[safety_monitor] Manual reset enabled on: {}", reset_stop_topic).c_str());
        }
        pub_stop_ = node_->create_publisher<common_msgs::msg::HuatStop>(stop_topic, 1);  // 锁存发布

        // last_pathlimits_time_ starts on first vehicle_state so a silent planner still times out.

        diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
        diag_updater_->add("Safety Monitor Health", this, &SafetyMonitor::DiagnoseHealth);
        diag_updater_->setHardwareID("safety_monitor");

        const std::string init_msg = std::format(
            "[safety_monitor] Started (timeout={:.1f}s, pathlimits={}, stop={}, stop_request={})",
            planner_timeout, pathlimits_topic, stop_topic,
            stop_request_topic.empty() ? "<disabled>" : stop_request_topic);
        RCLCPP_INFO(node_->get_logger(), "%s", init_msg.c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "[safety_monitor] Timeout starts after first vehicle_state; empty pathlimits are not heartbeats");
    }

    void spin() {
        rclcpp::Rate rate(50);  // 50Hz 检查
        while (rclcpp::ok()) {
            checkPlannerLiveness();
            diag_updater_->force_update();
            rate.sleep();
        }
    }

   private:
    void DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (!has_vehicle_state_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No vehicle state received yet");
        } else if (!has_nonempty_pathlimits_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for first non-empty pathlimits");
        } else {
            StopState state = state_machine_.state();
            if (state == StopState::TIMEOUT_STOP) {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                             std::format("Planner timeout stop active (reason: {})", stop_reason_));
            } else if (state == StopState::REQUEST_STOP) {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                             std::format("Stop request active (reason: {}, race finished)", stop_reason_));
            } else {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                             std::format("Running (state: {})", state));
            }
        }
        stat.add("Stop active", stop_active_);
        stat.add("Stop reason", std::format("{}", stop_reason_));
        stat.add("Current speed", std::format("{:.2f} m/s", current_speed_));
        if (has_vehicle_state_) {
            const double remaining = std::max(0.0, planner_timeout_.seconds() - (node_->now() - last_pathlimits_time_).seconds());
            stat.add("Planner timeout remaining (s)", std::format("{:.2f}", remaining));
        }
        stat.add("Has vehicle state", has_vehicle_state_);
    }

    static constexpr const char *StopReasonToString(StopReason reason) noexcept {
        return to_string_view(reason).data();
    }

    void OnPathLimits(const common_msgs::msg::HuatPathLimits::ConstSharedPtr &msg) {
        if (msg->path.empty()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[safety_monitor] Empty pathlimits ignored (not a liveness heartbeat)");
            return;
        }
        last_pathlimits_time_ = node_->now();
        has_nonempty_pathlimits_ = true;
        auto result = state_machine_.onPathReceived();
        if (result.action == StopAction::CLEAR_STOP) {
            RCLCPP_INFO(node_->get_logger(), "%s",
                        std::format("[safety_monitor] Planner resumed (action: {}), clearing timeout stop", result.action).c_str());
            publishStop(false);
        }
    }

    void OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr &msg) {
        if (!has_vehicle_state_) {
            last_pathlimits_time_ = node_->now();
        }
        has_vehicle_state_ = true;
        current_speed_ = msg->v;
    }

    void OnStopRequest(const common_msgs::msg::HuatStop::ConstSharedPtr &msg) {
        if (msg->stop) {
            auto result = state_machine_.onStopRequested();
            if (result.action == StopAction::PUBLISH_STOP) {
                RCLCPP_WARN(node_->get_logger(), "%s",
                            std::format("[safety_monitor] Stop request received (action: {}, reason: {}, race finished), publishing stop",
                                        result.action, result.reason).c_str());
                publishStop(true, StopReason::REQUEST);
            }
        }
    }

    void OnResetStop(const common_msgs::msg::HuatStop::ConstSharedPtr &msg) {
        if (!msg->stop) {
            auto result = state_machine_.onManualReset();
            if (result.action == StopAction::CLEAR_STOP) {
                RCLCPP_WARN(node_->get_logger(), "%s",
                            std::format("[safety_monitor] Manual reset received (action: {}), clearing REQUEST_STOP, returning to IDLE",
                                        result.action).c_str());
                publishStop(false);
            }
        }
    }

    void checkPlannerLiveness() {
        if (!has_vehicle_state_)
            return;  // 先等待车辆状态

        rclcpp::Duration elapsed = node_->now() - last_pathlimits_time_;
        if (elapsed > planner_timeout_) {
            auto result = state_machine_.onTimeoutExpired();
            if (result.action == StopAction::PUBLISH_STOP) {
                const std::string warn_msg = std::format(
                    "[safety_monitor] Planner timeout ({:.1f}s since last non-empty pathlimits, limit: {:.1f}s), publishing stop (reason: {})",
                    elapsed.seconds(), planner_timeout_.seconds(), result.reason);
                RCLCPP_WARN(node_->get_logger(), "%s", warn_msg.c_str());
                publishStop(true, StopReason::TIMEOUT);
            }
        }
    }

    void publishStop(bool stop, StopReason reason = StopReason::NONE) {
        common_msgs::msg::HuatStop msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "map";
        msg.stop = stop;
        pub_stop_->publish(msg);
        stop_active_ = stop;
        stop_reason_ = reason;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr sub_pathlimits_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_vehicle_state_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_stop_request_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_reset_;
    rclcpp::Publisher<common_msgs::msg::HuatStop>::SharedPtr pub_stop_;

    rclcpp::Time last_pathlimits_time_;
    rclcpp::Duration planner_timeout_ = rclcpp::Duration(0, 0);
    bool has_vehicle_state_;
    bool has_nonempty_pathlimits_;
    StopStateMachine state_machine_;
    double current_speed_ = 0.0;
    bool stop_active_ = false;
    StopReason stop_reason_ = StopReason::NONE;

    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    SafetyMonitor monitor;
    monitor.spin();
    return 0;
}
