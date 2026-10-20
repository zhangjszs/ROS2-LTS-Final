// ============================================================================
// issue #82：直线加速 ROS 闭环的**终点事件源**。
//
// 职责边界（刻意做薄）：本节点只做三件事——观测 `/localization/vehicle_state`、
// 判定首次越线、向 `/system/task_event` **恰好一次**发布 `finish`。完赛判据本身
// 一律复用 #80 的纯 std 核 `track_benchmark/finish_judge.hpp`：该头明确写着「供
// 离线 runner（#80）与 ROS 闭环门禁（#82）共用同一份判据——两处各写一套必然漂移」，
// 本节点就是它在 ROS 侧的第一个消费者。
//
// 「恰好一次」不靠自建计数器，而由 `FinishJudge::Update()` 的 `crossed_` 单调性
// 保证：越线置位后 `Update()` 恒返回 false，故状态消息抖动、重复投递、乱序重放
// 都不会导致二次发布（#82 验收第 3 条）。
//
// 停车不由本节点执行：`finish` 事件进 `command_arbiter_node` 的
// TaskSafetyStateMachine（契约 docs/INTERFACE_CONTRACT.md：`finish` → 锁存
// REQUEST 停，只能由 `reset` 解除），由仲裁器在契约出口 `/vehicle_command` 产出
// 安全制动。本节点**绝不**直接改写车辆状态、发布制动或"瞬移到停止点"
// （#82「明确不包含」：不为判据写瞬移/直接置零后门）。
//
// 终点线与缓冲区末端的取值来自赛道元数据（#81 的 `tracks.json`），由门禁脚本解析
// 后经参数传入：本节点不硬编码 75 / 100，也不自行解析 JSON——仓内 C++ 无 JSON
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

namespace {

namespace contract = common_msgs::contract;  // 话题名/QoS 单一来源

/// #82：终点事件源节点。
class FinishEventNode : public rclcpp::Node {
   public:
    FinishEventNode() : Node("finish_event_node") {
        spec_.enabled = declare_parameter<bool>("finish.enabled", true);
        spec_.finish_x = declare_parameter<double>("finish.finish_x", 0.0);
        spec_.runout_end_m = declare_parameter<double>("finish.runout_end_m", 0.0);
        spec_.brake_decel = declare_parameter<double>("finish.brake_decel", 9.0);
        spec_.stop_speed_eps = declare_parameter<double>("finish.stop_speed_eps", 0.05);

        // 契约未给 task_event 定义话题名常量（command_arbiter_node 亦同此缺省），
        // 此处与消费端保持同一字面量，不新增契约项（#82「不改契约话题」）。
        const auto event_topic = declare_parameter<std::string>("topics.task_event", "/system/task_event");
        const auto state_topic =
            declare_parameter<std::string>("topics.vehicle_state", std::string(contract::kTopicVehicleState));

        if (spec_.enabled && spec_.runout_end_m <= spec_.finish_x) {
            // 缓冲距离非正 → approachSpeedCap 返回 0、判据失去意义。宁可拒绝启动，也不要
            // 让门禁"看起来在跑"却在终点线处永不判停（#80 的 FinishJudge 同样以此为非法输入）。
            RCLCPP_FATAL(
                get_logger(),
                "[finish_event] 非法终点元数据：finish_x=%.3f 而 runout_end_m=%.3f（缓冲区末端必须在终点线之后）",
                spec_.finish_x, spec_.runout_end_m);
            rclcpp::shutdown();
            return;
        }

        judge_ = std::make_unique<benchmark::FinishJudge>(spec_);

        pub_event_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
        // 契约话题的订阅 QoS 一律由 makeQoS 构造：与发布端 (RELIABLE, depth 10) 不匹配的
        // 默认 QoS 在部分 DDS 实现下会静默不投递。
        sub_state_ = create_subscription<common_msgs::msg::HuatCarstate>(
            state_topic, contract::makeQoS(contract::kQosState),
            [this](const common_msgs::msg::HuatCarstate::SharedPtr m) { onState(*m); });

        RCLCPP_INFO(get_logger(),
                    "[finish_event] enabled=%s finish_x=%.3f runout_end_m=%.3f cap=%.3f m/s (state=%s event=%s)",
                    spec_.enabled ? "true" : "false", spec_.finish_x, spec_.runout_end_m,
                    benchmark::approachSpeedCap(spec_), state_topic.c_str(), event_topic.c_str());
    }

   private:
    void onState(const common_msgs::msg::HuatCarstate& m) {
        const double x = m.car_state.x;
        const double v = m.v;
        // 时间基准取状态消息自带的仿真时钟（use_sim_time 下即 /clock），不掺墙钟：
        // 越线时刻要能与离线 runner 的 finish_time_s 对齐。
        const double t = static_cast<double>(m.header.stamp.sec) + static_cast<double>(m.header.stamp.nanosec) * 1e-9;

        // Update() 只在**首次**越线返回 true —— 这就是「恰好一次」的全部依赖。
        if (judge_->Update(x, v, t)) {
            std_msgs::msg::String ev;
            ev.data = "finish";
            pub_event_->publish(ev);
            finish_published_++;
            RCLCPP_WARN(get_logger(),
                        "[finish_event] 越过终点线 x=%.3f >= finish_x=%.3f @ t=%.3fs（仿真钟）→ 发布 finish 第 %d 次",
                        x, spec_.finish_x, judge_->cross_time_s(), finish_published_);
        }

        if (judge_->ExceededRunout(x)) {
            // 只告警不干预：判停/判失败是门禁断言的职责，节点不得为"让门禁过"而动作。
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "[finish_event] 已冲出停车缓冲区 x=%.3f > runout_end_m=%.3f（v=%.2f m/s）", x,
                                  spec_.runout_end_m, v);
        }
    }

    benchmark::FinishRunoutSpec spec_;
    std::unique_ptr<benchmark::FinishJudge> judge_;
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
