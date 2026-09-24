// issue #16：最终指令仲裁节点 —— 订阅多个控制源（PP/MPC）与锁存 stop，
// 经 CommandArbiterFilter（去抖/stale）产出**唯一**最终 /vehicle_command（单一出口）。
// 纯编排：不含算法，只做“输入新鲜度 + checksum + 优先级 + 安全降级”，逻辑在共用层可单测。
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include "command_arbitration.h"  // #16：CommandArbiterFilter（复用 #12/#15/#16 契约原语）
#include "common_msgs/msg/huat_stop.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "interface_contract_qos.hpp"  // #14：契约话题 QoS 单一来源
#include "rclcpp/rclcpp.hpp"

using common_msgs::contract::makeQoS;
using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::ArbitrationConfig;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::CommandArbiterFilter;
using common_msgs::vehicle::CommandArbitrator;
using common_msgs::vehicle::ControlSource;
using common_msgs::vehicle::VehicleCommandRaw;

class CommandArbiterNode : public rclcpp::Node {
   public:
    CommandArbiterNode() : Node("command_arbiter_node") {
        source_a_topic_ = declare_parameter<std::string>("topics.source_a", "/control/vehicle_command");
        source_b_topic_ = declare_parameter<std::string>("topics.source_b", "/mpc/vehicle_command");
        output_topic_ = declare_parameter<std::string>("topics.output", "/vehicle_command");
        stop_topic_ = declare_parameter<std::string>("topics.stop", std::string(common_msgs::contract::kTopicStop));

        cfg_.source_timeout_sec = declare_parameter<double>("arbitration.source_timeout_sec", 0.5);
        cfg_.switch_dwell_sec = declare_parameter<double>("arbitration.switch_dwell_sec", 0.0);
        const std::string preferred = declare_parameter<std::string>("arbitration.preferred", "pure_pursuit");
        cfg_.preferred = (preferred == "mpc") ? ControlSource::MPC : ControlSource::PURE_PURSUIT;

        ActuatorCalibration cal;
        cal.max_accel = declare_parameter<double>("actuator.max_accel", 5.0);
        cal.max_decel = declare_parameter<double>("actuator.max_decel", 8.0);
        cal.emergency_brake_raw = declare_parameter<int>("actuator.emergency_brake_raw", 80);
        cal.calibration_version = declare_parameter<std::string>("actuator.calibration_version", "sim-default-0");
        neutral_steering_ = static_cast<std::uint8_t>(declare_parameter<int>("actuator.neutral_steering", 90));
        const double rate = declare_parameter<double>("rate", 50.0);

        arbiter_ = std::make_unique<CommandArbiterFilter>(CommandArbitrator(cal, neutral_steering_));

        const auto cmd_qos = makeQoS(common_msgs::contract::kQosCommand);
        sub_a_ = create_subscription<common_msgs::msg::HuatVehicleCmd>(
            source_a_topic_, cmd_qos,
            [this](const common_msgs::msg::HuatVehicleCmd::SharedPtr m) { onSource(ControlSource::PURE_PURSUIT, m); });
        sub_b_ = create_subscription<common_msgs::msg::HuatVehicleCmd>(
            source_b_topic_, cmd_qos,
            [this](const common_msgs::msg::HuatVehicleCmd::SharedPtr m) { onSource(ControlSource::MPC, m); });
        sub_stop_ = create_subscription<common_msgs::msg::HuatStop>(
            stop_topic_, makeQoS(common_msgs::contract::kQosStop),
            [this](const common_msgs::msg::HuatStop::SharedPtr m) { stop_latched_ = m->stop; });

        pub_out_ = create_publisher<common_msgs::msg::HuatVehicleCmd>(output_topic_, cmd_qos);
        publishSafeBrake();  // 启动即产出确定性安全指令，使最终出口一开始就有发布者

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)), [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "[command_arbiter] %s + %s -> %s (stop=%s, preferred=%s, dwell=%.2fs, timeout=%.2fs, calib=%s)",
                    source_a_topic_.c_str(), source_b_topic_.c_str(), output_topic_.c_str(), stop_topic_.c_str(),
                    preferred.c_str(), cfg_.switch_dwell_sec, cfg_.source_timeout_sec, cal.calibration_version.c_str());
    }

   private:
    static VehicleCommandRaw toRaw(const common_msgs::msg::HuatVehicleCmd& m) {
        VehicleCommandRaw r;
        r.steering = m.steering;
        r.brake_force = m.brake_force;
        r.pedal_ratio = m.pedal_ratio;
        r.gear_position = m.gear_position;
        r.working_mode = m.working_mode;
        r.racing_num = m.racing_num;
        r.racing_status = m.racing_status;
        return r;
    }

    void onSource(ControlSource src, const common_msgs::msg::HuatVehicleCmd::SharedPtr& m) {
        auto& slot = (src == ControlSource::PURE_PURSUIT) ? obs_a_ : obs_b_;
        slot.source = src;
        slot.cmd = toRaw(*m);
        slot.checksum = checksumRaw(slot.cmd);  // 以共用层重算，与源是否自带 checksum 无关
        slot.last_rx_sec = now().seconds();
        slot.present = true;
    }

    void tick() {
        const double t = now().seconds();
        const CommandArbiterFilter::SourceObs obs[] = {obs_a_, obs_b_};
        // 无外部任务事件：can_drive 由 stop 锁存决定（停 → 安全制动；否则进入源仲裁）。
        auto r = arbiter_->update(t, /*stop_active=*/stop_latched_, /*can_drive=*/!stop_latched_, std::span{obs}, cfg_);
        publishRaw(r.cmd);
    }

    void publishSafeBrake() {
        auto r = arbiter_->safeStop(common_msgs::vehicle::ArbitrationReason::STOP_ACTIVE);
        publishRaw(r.cmd);
    }

    void publishRaw(const VehicleCommandRaw& raw) {
        common_msgs::msg::HuatVehicleCmd m;
        m.head1 = common_msgs::vehicle::kCmdHead1;
        m.head2 = common_msgs::vehicle::kCmdHead2;
        m.length = common_msgs::vehicle::kCmdLength;
        m.steering = raw.steering;
        m.brake_force = raw.brake_force;
        m.pedal_ratio = raw.pedal_ratio;
        m.gear_position = raw.gear_position;
        m.working_mode = raw.working_mode;
        m.racing_num = raw.racing_num;
        m.racing_status = raw.racing_status;
        m.checksum = checksumRaw(raw);
        pub_out_->publish(m);
    }

    std::string source_a_topic_, source_b_topic_, output_topic_, stop_topic_;
    ArbitrationConfig cfg_;
    std::uint8_t neutral_steering_{90};
    bool stop_latched_{false};
    CommandArbiterFilter::SourceObs obs_a_{};
    CommandArbiterFilter::SourceObs obs_b_{};
    std::unique_ptr<CommandArbiterFilter> arbiter_;
    rclcpp::Subscription<common_msgs::msg::HuatVehicleCmd>::SharedPtr sub_a_, sub_b_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_stop_;
    rclcpp::Publisher<common_msgs::msg::HuatVehicleCmd>::SharedPtr pub_out_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandArbiterNode>());
    rclcpp::shutdown();
    return 0;
}
