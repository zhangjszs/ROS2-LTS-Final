// ============================================================================
// issue #82/#91：完赛**终点事件源**（直线加速 / 八字绕环双赛制）。
//
// 职责边界（刻意做薄）：本节点只做三件事——观测 `/localization/vehicle_state`、
// 判定首次完赛、向 `/system/task_event` **恰好一次**发布 `finish`。完赛判据一律
// 复用纯 std 核（Core/ROS 边界 ADR）：
//   - acceleration（#80 finish_judge.hpp）：越线 + 停稳，离线 runner 与本节点共用；
//   - skidpad（#90 skidpad_finish_judge.hpp）：绕角累积计圈 + 相位状态机，
//     Update() 首次完赛返回 true（kFinished 吸收态），与 #80 的首次越线语义对称。
//
// 「恰好一次」不靠自建计数器，而由判据核 Update() 的吸收态单调性保证：完赛置位后
// Update() 恒返回 false，故状态消息抖动、重复投递、乱序重放都不会导致二次发布。
//
// 停车不由本节点执行：`finish` 事件进 `command_arbiter_node` 的
// TaskSafetyStateMachine（契约 docs/INTERFACE_CONTRACT.md：`finish` → 锁存
// REQUEST 停，只能由 `reset` 解除），由仲裁器在契约出口 `/vehicle_command` 产出
// 安全制动。本节点**绝不**直接改写车辆状态、发布制动或"瞬移到停止点"。
//
// 判据参数取值来自赛道元数据（#81/#90 的 `tracks.json`），由门禁脚本解析后经参数
// 传入：本节点不硬编码 75/100 或 9.125/2，也不自行解析 JSON——仓内 C++ 无 JSON
// 解析先例，为一个判据引入第三方解析库不成比例。
// ============================================================================
#include <memory>
#include <string>

#include "common_msgs/msg/huat_carstate.hpp"
#include "interface_contract.h"        // #14：契约话题名缺省常量（common_msgs 手写头不带前缀）
#include "interface_contract_qos.hpp"  // #14：契约话题 QoS 由 makeQoS 单一来源构造
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "track_benchmark/finish_judge.hpp"
#include "track_benchmark/skidpad_finish_judge.hpp"

namespace {

namespace contract = common_msgs::contract;  // 话题名/QoS 单一来源

/// #82/#91：完赛事件源节点（按 finish.mode 选择判据核）。
class FinishEventNode : public rclcpp::Node {
   public:
    FinishEventNode() : Node("finish_event_node") {
        const auto mode = declare_parameter<std::string>("finish.mode", "acceleration");
        const bool enabled = declare_parameter<bool>("finish.enabled", true);

        // 契约未给 task_event 定义话题名常量（command_arbiter_node 亦同此缺省），
        // 此处与消费端保持同一字面量，不新增契约项（#82「不改契约话题」）。
        const auto event_topic = declare_parameter<std::string>("topics.task_event", "/system/task_event");
        const auto state_topic =
            declare_parameter<std::string>("topics.vehicle_state", std::string(contract::kTopicVehicleState));

        if (mode == "acceleration") {
            mode_ = Mode::kAcceleration;
            benchmark::FinishRunoutSpec spec;
            spec.enabled = enabled;
            spec.finish_x = declare_parameter<double>("finish.finish_x", 0.0);
            spec.runout_end_m = declare_parameter<double>("finish.runout_end_m", 0.0);
            spec.brake_decel = declare_parameter<double>("finish.brake_decel", 9.0);
            spec.stop_speed_eps = declare_parameter<double>("finish.stop_speed_eps", 0.05);
            if (spec.enabled && spec.runout_end_m <= spec.finish_x) {
                // 缓冲距离非正 → approachSpeedCap 返回 0、判据失去意义。宁可拒绝启动，也不要
                // 让门禁"看起来在跑"却在终点线处永不判停（#80 的 FinishJudge 同样以此为非法输入）。
                RCLCPP_FATAL(get_logger(),
                             "[finish_event] 非法终点元数据：finish_x=%.3f 而 runout_end_m=%.3f（缓冲区末端必须在"
                             "终点线之后）",
                             spec.finish_x, spec.runout_end_m);
                rclcpp::shutdown();
                return;
            }
            accel_judge_ = std::make_unique<benchmark::FinishJudge>(spec);
            RCLCPP_INFO(get_logger(),
                        "[finish_event] mode=acceleration enabled=%s finish_x=%.3f runout_end_m=%.3f "
                        "cap=%.3f m/s (state=%s event=%s)",
                        enabled ? "true" : "false", spec.finish_x, spec.runout_end_m, benchmark::approachSpeedCap(spec),
                        state_topic.c_str(), event_topic.c_str());
        } else if (mode == "skidpad") {
            mode_ = Mode::kSkidpad;
            // #90 元数据经参数传入（值来自 tracks.json 的 skidpad 字段，脚本解析）。
            benchmark::SkidpadFinishSpec spec;
            spec.enabled = enabled;
            spec.right_center_x = declare_parameter<double>("skidpad.right_center_x", 0.0);
            spec.right_center_y = declare_parameter<double>("skidpad.right_center_y", 9.125);
            spec.left_center_x = declare_parameter<double>("skidpad.left_center_x", 0.0);
            spec.left_center_y = declare_parameter<double>("skidpad.left_center_y", -9.125);
            spec.path_radius = declare_parameter<double>("skidpad.path_radius", 9.125);
            spec.required_laps = declare_parameter<int>("skidpad.required_laps", 2);
            spec.exit_x = declare_parameter<double>("skidpad.exit_x", 0.0);
            skidpad_judge_ = std::make_unique<benchmark::SkidpadFinishJudge>(spec);
            if (spec.enabled && !skidpad_judge_->valid()) {
                // 与 acceleration 模式同一防御姿态：非法几何（NaN/非正半径/零圈数/圆心重合）
                // 时判据永不 finished，门禁会"看起来在跑"却永远等不到事件——宁可拒绝启动。
                RCLCPP_FATAL(get_logger(),
                             "[finish_event] 非法 skidpad 元数据：右圆(%.3f,%.3f) 左圆(%.3f,%.3f) r=%.3f laps=%d",
                             spec.right_center_x, spec.right_center_y, spec.left_center_x, spec.left_center_y,
                             spec.path_radius, spec.required_laps);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(get_logger(),
                        "[finish_event] mode=skidpad enabled=%s 右圆(%.3f,%.3f) 左圆(%.3f,%.3f) r=%.3f laps=%d "
                        "exit_x=%.3f (state=%s event=%s)",
                        enabled ? "true" : "false", spec.right_center_x, spec.right_center_y, spec.left_center_x,
                        spec.left_center_y, spec.path_radius, spec.required_laps, spec.exit_x, state_topic.c_str(),
                        event_topic.c_str());
        } else {
            RCLCPP_FATAL(get_logger(), "[finish_event] 未知 finish.mode='%s'（合法值：acceleration | skidpad）",
                         mode.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_event_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
        // 契约话题的订阅 QoS 一律由 makeQoS 构造：与发布端 (RELIABLE, depth 10) 不匹配的
        // 默认 QoS 在部分 DDS 实现下会静默不投递。
        sub_state_ = create_subscription<common_msgs::msg::HuatCarstate>(
            state_topic, contract::makeQoS(contract::kQosState),
            [this](const common_msgs::msg::HuatCarstate::SharedPtr m) { onState(*m); });
    }

   private:
    void onState(const common_msgs::msg::HuatCarstate& m) {
        const double x = m.car_state.x;
        const double v = m.v;
        // 时间基准取状态消息自带的仿真时钟（use_sim_time 下即 /clock），不掺墙钟：
        // 完赛时刻要能与离线 runner 的时间字段对齐。
        const double t = static_cast<double>(m.header.stamp.sec) + static_cast<double>(m.header.stamp.nanosec) * 1e-9;

        // 判据核 Update() 只在**首次**完赛返回 true —— 这就是「恰好一次」的全部依赖。
        bool first_finish = false;
        if (mode_ == Mode::kAcceleration) {
            first_finish = accel_judge_->Update(x, v, t);
            if (accel_judge_->ExceededRunout(x)) {
                // 只告警不干预：判停/判失败是门禁断言的职责，节点不得为"让门禁过"而动作。
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "[finish_event] 已冲出停车缓冲区 x=%.3f（v=%.2f m/s）", x, v);
            }
        } else {
            first_finish = skidpad_judge_->Update(x, m.car_state.y);
        }

        if (first_finish) {
            std_msgs::msg::String ev;
            ev.data = "finish";
            pub_event_->publish(ev);
            finish_published_++;
            RCLCPP_WARN(get_logger(),
                        "[finish_event] 完赛首次判定 @ t=%.3fs（仿真钟）x=%.3f y=%.3f → 发布 finish 第 %d 次", t, x,
                        m.car_state.y, finish_published_);
        }
    }

    enum class Mode { kAcceleration, kSkidpad };
    Mode mode_{Mode::kAcceleration};
    std::unique_ptr<benchmark::FinishJudge> accel_judge_;           // acceleration 模式
    std::unique_ptr<benchmark::SkidpadFinishJudge> skidpad_judge_;  // skidpad 模式
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_state_;
    int finish_published_{0};
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FinishEventNode>());
    rclcpp::shutdown();
    return 0;
}
