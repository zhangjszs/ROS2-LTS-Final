// issue #16：最终指令仲裁节点 —— 订阅多个控制源（PP/MPC）与锁存 stop，
// 经 TaskSafetyStateMachine（任务×安全）+ CommandArbiterFilter（去抖/stale）产出
// **唯一**最终 /vehicle_command（单一出口），并按控制率发布 /system/state 遥测。
// 纯编排：不含算法，只做“输入新鲜度 + 帧有效性 + 优先级 + 安全降级”，判定逻辑在共用层可单测。
// 安全默认：启动后处于 IDLE（不授予行驶许可），需 arm+start 事件或 task.autostart:=true
// 才能行驶；节点重启/晚加入不会意外得到行驶许可（#16 验收第 1 条）。
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include "command_arbitration.h"  // #16：CommandArbiterFilter（复用 #12/#15/#16 契约原语）
#include "common_msgs/msg/huat_stop.hpp"
#include "common_msgs/msg/huat_system_state.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "interface_contract_qos.hpp"  // #14：契约话题 QoS 单一来源
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "task_state_machine.h"  // #16：TaskSafetyStateMachine

using common_msgs::contract::makeQoS;
using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::ArbitrationConfig;
using common_msgs::vehicle::ArbitrationReason;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::CommandArbiterFilter;
using common_msgs::vehicle::CommandArbitrator;
using common_msgs::vehicle::ControlSource;
using common_msgs::vehicle::TaskSafetyStateMachine;
using common_msgs::vehicle::VehicleCommandRaw;

using StateMsg = common_msgs::msg::HuatSystemState;
using Untrusted = CommandArbiterFilter::Untrusted;

// 机读遥测的数值编码必须与纯 std 共用层的枚举顺序一致（编译期锁定，防止单方面漂移）。
static_assert(static_cast<uint8_t>(common_msgs::vehicle::TaskState::IDLE) == StateMsg::TASK_IDLE);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::TaskState::ARMED) == StateMsg::TASK_ARMED);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::TaskState::RUNNING) == StateMsg::TASK_RUNNING);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::TaskState::FINISHED) == StateMsg::TASK_FINISHED);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::TaskState::FAULT) == StateMsg::TASK_FAULT);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::SafetyState::NORMAL) == StateMsg::SAFETY_NORMAL);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::SafetyState::DEGRADED) == StateMsg::SAFETY_DEGRADED);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::SafetyState::STOP) == StateMsg::SAFETY_STOP);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::StopKind::NONE) == StateMsg::STOPKIND_NONE);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::StopKind::TIMEOUT) == StateMsg::STOPKIND_TIMEOUT);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::StopKind::REQUEST) == StateMsg::STOPKIND_REQUEST);
static_assert(static_cast<uint8_t>(common_msgs::vehicle::StopKind::FAULT) == StateMsg::STOPKIND_FAULT);
static_assert(static_cast<uint8_t>(ControlSource::NONE) == StateMsg::SRC_NONE);
static_assert(static_cast<uint8_t>(ControlSource::PURE_PURSUIT) == StateMsg::SRC_PURE_PURSUIT);
static_assert(static_cast<uint8_t>(ControlSource::MPC) == StateMsg::SRC_MPC);
static_assert(static_cast<uint8_t>(ArbitrationReason::CONTROL) == StateMsg::REASON_CONTROL);
static_assert(static_cast<uint8_t>(ArbitrationReason::STOP_ACTIVE) == StateMsg::REASON_STOP_ACTIVE);
static_assert(static_cast<uint8_t>(ArbitrationReason::TASK_NOT_DRIVING) == StateMsg::REASON_TASK_NOT_DRIVING);
static_assert(static_cast<uint8_t>(ArbitrationReason::NO_TRUSTED_SOURCE) == StateMsg::REASON_NO_TRUSTED_SOURCE);
static_assert(static_cast<uint8_t>(Untrusted::kNone) == StateMsg::UNTRUSTED_NONE);
static_assert(static_cast<uint8_t>(Untrusted::kAbsent) == StateMsg::UNTRUSTED_ABSENT);
static_assert(static_cast<uint8_t>(Untrusted::kBadFrame) == StateMsg::UNTRUSTED_BAD_FRAME);
static_assert(static_cast<uint8_t>(Untrusted::kBadChecksum) == StateMsg::UNTRUSTED_BAD_CHECKSUM);
static_assert(static_cast<uint8_t>(Untrusted::kStale) == StateMsg::UNTRUSTED_STALE);

class CommandArbiterNode : public rclcpp::Node {
   public:
    CommandArbiterNode() : Node("command_arbiter_node") {
        source_a_topic_ = declare_parameter<std::string>("topics.source_a", "/control/vehicle_command");
        source_b_topic_ = declare_parameter<std::string>("topics.source_b", "/mpc/vehicle_command");
        output_topic_ = declare_parameter<std::string>("topics.output", "/vehicle_command");
        stop_topic_ = declare_parameter<std::string>("topics.stop", std::string(common_msgs::contract::kTopicStop));
        state_topic_ =
            declare_parameter<std::string>("topics.state", std::string(common_msgs::contract::kTopicSystemState));
        event_topic_ = declare_parameter<std::string>("topics.task_event", "/system/task_event");

        cfg_.source_timeout_sec = declare_parameter<double>("arbitration.source_timeout_sec", 0.5);
        cfg_.switch_dwell_sec = declare_parameter<double>("arbitration.switch_dwell_sec", 0.0);
        const std::string preferred = declare_parameter<std::string>("arbitration.preferred", "pure_pursuit");
        cfg_.preferred = (preferred == "mpc") ? ControlSource::MPC : ControlSource::PURE_PURSUIT;

        // 行驶许可授予：默认关闭。开发/仿真可显式 autostart；实车应由人工/任务层发 arm+start 事件。
        const bool autostart = declare_parameter<bool>("task.autostart", false);

        ActuatorCalibration cal;
        cal.max_accel = declare_parameter<double>("actuator.max_accel", 5.0);
        cal.max_decel = declare_parameter<double>("actuator.max_decel", 8.0);
        cal.pedal_full_scale = declare_parameter<double>("actuator.pedal_full_scale", 100.0);
        cal.emergency_brake_raw = declare_parameter<int>("actuator.emergency_brake_raw", 80);
        cal.soft_brake_raw = declare_parameter<int>("actuator.soft_brake_raw", 40);
        cal.calibration_version = declare_parameter<std::string>("actuator.calibration_version", "sim-default-0");
        calibration_version_ = cal.calibration_version;
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
        // 锁存 stop（#8/#14）：true → 锁存停；false → 仅解除 TIMEOUT 停，REQUEST/FAULT 保持锁存。
        sub_stop_ = create_subscription<common_msgs::msg::HuatStop>(
            stop_topic_, makeQoS(common_msgs::contract::kQosStop),
            [this](const common_msgs::msg::HuatStop::SharedPtr m) { onStop(*m); });
        // 任务事件（arm/start/finish/abort/reset）：行驶许可的唯一授予入口。
        sub_event_ = create_subscription<std_msgs::msg::String>(
            event_topic_, 10, [this](const std_msgs::msg::String::SharedPtr m) { onTaskEvent(m->data); });

        pub_out_ = create_publisher<common_msgs::msg::HuatVehicleCmd>(output_topic_, cmd_qos);
        pub_state_ = create_publisher<StateMsg>(state_topic_, makeQoS(common_msgs::contract::kQosState));
        publishSafeBrake();  // 启动即产出确定性安全指令，使最终出口一开始就有发布者

        if (autostart) {
            applyEvent("arm");
            applyEvent("start");
            RCLCPP_WARN(get_logger(), "[command_arbiter] task.autostart=true -> RUNNING (development aid)");
        }

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)), [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "[command_arbiter] %s + %s -> %s (stop=%s, state=%s, events=%s, preferred=%s, dwell=%.2fs, "
                    "timeout=%.2fs, calib=%s, task=%s)",
                    source_a_topic_.c_str(), source_b_topic_.c_str(), output_topic_.c_str(), stop_topic_.c_str(),
                    state_topic_.c_str(), event_topic_.c_str(), preferred.c_str(), cfg_.switch_dwell_sec,
                    cfg_.source_timeout_sec, calibration_version_.c_str(),
                    std::string(common_msgs::vehicle::to_string_view(sm_.task())).c_str());
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
        // #16 信任门：帧头/长度/校验和一律取**线上值**原样透传，由 CommandArbiterFilter 判定。
        // 绝不在接收处重算 checksum：重算后 verifyChecksum 永真，篡改/截断帧会被当作可信源接管。
        auto& slot = (src == ControlSource::PURE_PURSUIT) ? obs_a_ : obs_b_;
        slot = common_msgs::vehicle::makeWireSourceObs(src, toRaw(*m), m->head1, m->head2, m->length, m->checksum,
                                                       now().seconds());
    }

    void onStop(const common_msgs::msg::HuatStop& m) {
        // 监控看门狗是“可恢复停车”的来源（HuatStop 不携带 reason，只有 bool）：
        //   stop=true  → TIMEOUT 停（可被监控释放）；
        //   stop=false → onResume 仅解除 TIMEOUT 停。
        // 任务级锁存（abort → FAULT、finish → REQUEST）只能由 reset 事件解除：
        // 监控“恢复”不会把急停类锁存带出来（#16 验收第 3 条）。
        if (m.stop) {
            sm_.onTimeoutStop();
        } else {
            sm_.onResume();
        }
    }

    void onTaskEvent(const std::string& data) { applyEvent(data); }

    void applyEvent(std::string_view raw) {
        // 去空格并小写归一，避免调用方大小写/尾空格差异导致许可授予失败。
        std::string ev;
        for (char ch : raw) {
            if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r')
                ev.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        }
        if (ev == "arm") {
            sm_.onArm();
        } else if (ev == "start") {
            sm_.onStart();
        } else if (ev == "finish") {
            sm_.onFinish();
        } else if (ev == "abort" || ev == "fault") {
            sm_.onFault();
        } else if (ev == "reset") {
            sm_.onReset();
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[command_arbiter] unknown task event '%s' (arm|start|finish|abort|reset)",
                                 std::string(raw).c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[command_arbiter] event=%s -> task=%s safety=%s stop_kind=%s can_drive=%s",
                    std::string(raw).c_str(), std::string(common_msgs::vehicle::to_string_view(sm_.task())).c_str(),
                    std::string(common_msgs::vehicle::to_string_view(sm_.safety())).c_str(),
                    std::string(common_msgs::vehicle::to_string_view(sm_.stopKind())).c_str(),
                    sm_.canDrive() ? "true" : "false");
    }

    void tick() {
        const double t = now().seconds();
        const CommandArbiterFilter::SourceObs obs[] = {obs_a_, obs_b_};
        tallyRejections(t, obs);
        // 统一由任务×安全状态机授予许可（stop 锁存 / 未 arm / FAULT / FINISHED 一律安全制动）。
        auto r = arbiter_->update(t, sm_, std::span{obs}, cfg_);
        publishRaw(r.cmd);
        publishState(t, obs, r);
    }

    // 拒收计数（与实际门同一 classify 实现，不漂移）；kAbsent = 该源尚未发过指令，不计为拒收。
    void tallyRejections(double t, std::span<const CommandArbiterFilter::SourceObs> obs) {
        for (const auto& o : obs) {
            const Untrusted why = CommandArbiterFilter::classify(o, t, cfg_);
            if (why == Untrusted::kNone || why == Untrusted::kAbsent)
                continue;
            // 按源累计（供遥测对账）+ 按原因全局计数（供日志快速定位）。
            auto& per_source = (o.source == ControlSource::MPC) ? rejected_b_total_ : rejected_a_total_;
            ++per_source;
            switch (why) {
                case Untrusted::kBadFrame:
                    ++rejected_frame_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[command_arbiter] %s frame rejected: %s",
                                         common_msgs::vehicle::to_string(o.source), "bad_frame");
                    break;
                case Untrusted::kBadChecksum:
                    ++rejected_checksum_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[command_arbiter] %s frame rejected: %s",
                                         common_msgs::vehicle::to_string(o.source), "bad_checksum");
                    break;
                case Untrusted::kStale:
                    ++rejected_stale_;
                    break;
                case Untrusted::kNone:
                case Untrusted::kAbsent:
                    break;
            }
        }
    }

    void publishSafeBrake() {
        auto r = arbiter_->safeStop(ArbitrationReason::STOP_ACTIVE);
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

    // 故障注入验收的机读证据：状态迁移、仲裁理由、各源信任状态与本帧出口字节一起落盘。
    void publishState(double t, std::span<const CommandArbiterFilter::SourceObs> obs,
                      const common_msgs::vehicle::ArbitrationResult& r) {
        StateMsg s;
        s.header.stamp = now();
        s.header.frame_id = "base_link";
        s.task_state = static_cast<uint8_t>(sm_.task());
        s.safety_state = static_cast<uint8_t>(sm_.safety());
        s.stop_kind = static_cast<uint8_t>(sm_.stopKind());
        s.stop_active = sm_.stopActive();
        s.can_drive = sm_.canDrive();
        s.winner_source = static_cast<uint8_t>(r.winner);
        s.committed_source = static_cast<uint8_t>(arbiter_->winner());
        s.reason = static_cast<uint8_t>(r.reason);
        s.safe_fallback = r.safe_fallback;
        s.source_a_status = statusOf(obs[0], t, cfg_);
        s.source_b_status = statusOf(obs[1], t, cfg_);
        s.source_a_age_sec = ageOf(obs[0], t);
        s.source_b_age_sec = ageOf(obs[1], t);
        s.source_a_rejected = rejected_a_total_;
        s.source_b_rejected = rejected_b_total_;
        s.steering = r.cmd.steering;
        s.brake_force = r.cmd.brake_force;
        s.pedal_ratio = r.cmd.pedal_ratio;
        s.checksum = r.checksum;
        s.calibration_version = calibration_version_;
        pub_state_->publish(s);
    }

    static uint8_t statusOf(const CommandArbiterFilter::SourceObs& o, double t, const ArbitrationConfig& cfg) {
        return static_cast<uint8_t>(CommandArbiterFilter::classify(o, t, cfg));
    }

    static double ageOf(const CommandArbiterFilter::SourceObs& o, double t) {
        return (o.last_rx_sec < 0.0) ? -1.0 : (t - o.last_rx_sec);
    }

    std::string source_a_topic_, source_b_topic_, output_topic_, stop_topic_, state_topic_, event_topic_;
    std::string calibration_version_;
    ArbitrationConfig cfg_;
    std::uint8_t neutral_steering_{90};
    TaskSafetyStateMachine sm_;
    // 按源累计拒收次数（帧头/校验和/超龄），供故障时间线对账。
    std::uint64_t rejected_frame_{};
    std::uint64_t rejected_checksum_{};
    std::uint64_t rejected_stale_{};
    std::uint64_t rejected_a_total_{};
    std::uint64_t rejected_b_total_{};
    CommandArbiterFilter::SourceObs obs_a_{};
    CommandArbiterFilter::SourceObs obs_b_{};
    std::unique_ptr<CommandArbiterFilter> arbiter_;
    rclcpp::Subscription<common_msgs::msg::HuatVehicleCmd>::SharedPtr sub_a_, sub_b_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_stop_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_event_;
    rclcpp::Publisher<common_msgs::msg::HuatVehicleCmd>::SharedPtr pub_out_;
    rclcpp::Publisher<StateMsg>::SharedPtr pub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandArbiterNode>());
    rclcpp::shutdown();
    return 0;
}
