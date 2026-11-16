#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <format>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <stop_token>
#include <thread>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_path_limits.hpp"
#include "common_msgs/msg/huat_stop.hpp"
#include "interface_contract.h"
#include "interface_contract_qos.hpp"
#include "safety_monitor/input_freshness.h"
#include "safety_monitor/stop_state_machine.h"

class SafetyMonitor {
   public:
    SafetyMonitor() : node_(rclcpp::Node::make_shared("safety_monitor")) {
        const double planner_timeout = node_->declare_parameter("planner_timeout", 0.5);
        // 与 PP safety.state_timeout=0.3s 对齐；不是实车制动距离标定值。
        const double state_timeout = node_->declare_parameter("safety.state_timeout_sec", 0.3);
        inputs_ = safety_monitor::InputFreshness(state_timeout, planner_timeout);
        planner_timeout_sec_ = planner_timeout;
        const auto path_topic = node_->declare_parameter<std::string>("pathlimits_topic", "/planning/pathlimits");
        const auto state_topic =
            node_->declare_parameter<std::string>("vehicle_state_topic", "/localization/vehicle_state");
        const auto stop_topic = node_->declare_parameter<std::string>("stop_topic", "/system/stop");
        const auto request_topic = node_->declare_parameter<std::string>("stop_request_topic", "");
        const auto reset_topic = node_->declare_parameter<std::string>("reset_stop_topic", "");

        sub_pathlimits_ = node_->create_subscription<common_msgs::msg::HuatPathLimits>(
            path_topic, common_msgs::contract::makeQoS(common_msgs::contract::kQosPath),
            [this](const common_msgs::msg::HuatPathLimits::ConstSharedPtr msg) { OnPathLimits(msg); });
        sub_vehicle_state_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
            state_topic, common_msgs::contract::makeQoS(common_msgs::contract::kQosState),
            [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnVehicleState(msg); });
        if (!request_topic.empty()) {
            sub_stop_request_ = node_->create_subscription<common_msgs::msg::HuatStop>(
                request_topic, 10,
                [this](const common_msgs::msg::HuatStop::ConstSharedPtr msg) { OnStopRequest(msg); });
        }
        if (!reset_topic.empty()) {
            sub_reset_ = node_->create_subscription<common_msgs::msg::HuatStop>(
                reset_topic, 1, [this](const common_msgs::msg::HuatStop::ConstSharedPtr msg) { OnResetStop(msg); });
        }
        pub_stop_ = node_->create_publisher<common_msgs::msg::HuatStop>(
            stop_topic, common_msgs::contract::makeQoS(common_msgs::contract::kQosStop));
        state_machine_.onTimeoutExpired();
        publishStop(true);
        diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
        diag_updater_->add("Safety Monitor Health", this, &SafetyMonitor::DiagnoseHealth);
        diag_updater_->setHardwareID("safety_monitor");
        RCLCPP_INFO(node_->get_logger(),
                    "[safety_monitor] Receive leases: state=%.3fs path=%.3fs (steady clock; uncalibrated)",
                    state_timeout, planner_timeout);
    }

    ~SafetyMonitor() { stopWatchdog(); }
    void startWatchdog() {
        if (!watchdog_thread_.joinable())
            watchdog_thread_ = std::jthread([this](std::stop_token st) { watchdogLoop(st); });
    }
    void stopWatchdog() {
        if (watchdog_thread_.joinable()) {
            watchdog_thread_.request_stop();
            watchdog_thread_.join();
        }
    }
    bool isWatchdogRunning() const noexcept { return watchdog_thread_.joinable(); }
    void spin() {
        startWatchdog();
        rclcpp::spin(node_);
        stopWatchdog();
    }

   private:
    static double steadySeconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void watchdogLoop(std::stop_token st) {
        // /clock 暂停不能暂停停车判定或心跳。接收租约与循环均不依赖 ROS 时钟推进。
        rclcpp::WallRate rate(50);
        unsigned heartbeat_tick = 0;
        while (!st.stop_requested() && rclcpp::ok()) {
            {
                std::scoped_lock lock(state_mutex_);
                updateHealth(steadySeconds(), node_->now().seconds());
                if (++heartbeat_tick % 5 == 0)
                    publishStop(stop_active_);
            }
            diag_updater_->force_update();
            rate.sleep();
        }
    }
    void DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        std::scoped_lock lock(state_mutex_);
        const auto health = inputs_.check(steadySeconds(), node_->now().seconds());
        stat.summary(
            stop_active_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK,
            std::format("{} (vehicle={}, planner={})", state_machine_.state(),
                        safety_monitor::freshnessName(health.vehicle), safety_monitor::freshnessName(health.planner)));
        stat.add("Stop active", stop_active_);
        stat.add("Stop reason", std::format("{}", state_machine_.reason()));
        stat.add("Vehicle receive age (steady s)", health.vehicle_age);
        stat.add("Planner receive age (steady s)", health.planner_age);
        stat.add("Vehicle freshness", safety_monitor::freshnessName(health.vehicle));
        stat.add("Planner freshness", safety_monitor::freshnessName(health.planner));
        stat.add("Current speed (m/s)", current_speed_);
    }
    void OnPathLimits(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg) {
        std::scoped_lock lock(state_mutex_);
        const auto ros_now = node_->now();
        const double rx = steadySeconds();
        const auto stamp = rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type());
        const double source_age = (ros_now - stamp).seconds();
        bool valid = msg->path.size() >= 2 && common_msgs::contract::isFrameSupported(msg->header.frame_id) &&
                     std::isfinite(source_age) && source_age >= -0.05 && source_age <= planner_timeout_sec_;
        if (valid) {
            for (const auto& point : msg->path) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                    valid = false;
                    break;
                }
            }
        }
        inputs_.observePath(valid, rx, ros_now.seconds());
        updateHealth(rx, ros_now.seconds());
    }
    void OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
        std::scoped_lock lock(state_mutex_);
        const double rx = steadySeconds();
        const double ros = node_->now().seconds();
        const bool valid =
            safety_monitor::validVehicleState(msg->car_state.x, msg->car_state.y, msg->car_state.theta, msg->v);
        // 无效数据不刷新有效接收租约；旧合法帧最多保留到 state_timeout。
        inputs_.observeVehicle(valid, rx, ros);
        if (valid)
            current_speed_ = msg->v;
        updateHealth(rx, ros);
    }
    // 所有恢复路径共用双租约检查；这里仅清 TIMEOUT，不授予任务行驶许可。
    void updateHealth(double rx, double ros) {
        const bool healthy = inputs_.healthy(rx, ros);
        const auto result = healthy ? state_machine_.onPathReceived() : state_machine_.onTimeoutExpired();
        if (result.action == StopAction::PUBLISH_STOP)
            publishStop(true);
        else if (result.action == StopAction::CLEAR_STOP)
            publishStop(false);
    }
    void OnStopRequest(const common_msgs::msg::HuatStop::ConstSharedPtr& msg) {
        std::scoped_lock lock(state_mutex_);
        if (msg->stop) {
            state_machine_.onStopRequested();
            publishStop(true);
        }
    }
    void OnResetStop(const common_msgs::msg::HuatStop::ConstSharedPtr& msg) {
        std::scoped_lock lock(state_mutex_);
        if (!msg->stop && inputs_.healthy(steadySeconds(), node_->now().seconds())) {
            const auto result = state_machine_.onManualReset();
            if (result.action == StopAction::CLEAR_STOP)
                publishStop(false);
        }
    }
    void publishStop(bool stop) {
        common_msgs::msg::HuatStop msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "map";
        msg.stop = stop;
        pub_stop_->publish(msg);
        stop_active_ = stop;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr sub_pathlimits_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_vehicle_state_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_stop_request_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_reset_;
    rclcpp::Publisher<common_msgs::msg::HuatStop>::SharedPtr pub_stop_;
    safety_monitor::InputFreshness inputs_;
    double planner_timeout_sec_ = 0.5;
    StopStateMachine state_machine_;
    mutable std::mutex state_mutex_;
    double current_speed_ = 0.0;
    bool stop_active_ = true;
    std::jthread watchdog_thread_;
    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    SafetyMonitor monitor;
    monitor.spin();
    return 0;
}
