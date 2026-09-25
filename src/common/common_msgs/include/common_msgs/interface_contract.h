#pragma once
// ============================================================================
// 统一接口契约（issue #14）：轨迹 / 车辆状态 / 控制指令 的单一事实来源。
// 纯 std 依赖（不引入 rclcpp/ros），任意模块可安全 include。
//
// 目的：把过去散落在各节点的“话题名 / 坐标系 / 单位 / 有效性哨兵 / 非法数据判定”
// 收敛为集中常量 + 无副作用校验器，使消费者在接入时显式解释语义，而非隐式假设。
//
// 具体缺陷修复历史见各 Bug issue：#2(转向编码)、#3(坐标系)、#11(零速 vs 缺失)、
// #12(测量时间/来源年龄)。本头提供它们共用的判定原语。
// 详见 docs/INTERFACE_CONTRACT.md。
// ============================================================================
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>

namespace common_msgs {
namespace contract {

// —— 话题名（默认约定；节点仍可用参数覆盖，但缺省应引用此处而非散落字面量）——
inline constexpr std::string_view kTopicVehicleState = "/localization/vehicle_state";
inline constexpr std::string_view kTopicPathLimits = "/planning/pathlimits";
inline constexpr std::string_view kTopicVehicleCommand = "/vehicle_command";
inline constexpr std::string_view kTopicStop = "/system/stop";
// #16：任务×安全状态 + 仲裁遥测出口（由 command_arbiter_node 按控制率发布）；
// 非控制链路必需，但故障注入验收的机读证据必须带单一来源。
inline constexpr std::string_view kTopicSystemState = "/system/state";

// —— 坐标系唯一约定（issue #3）——
//   map：全局/世界系（赛道、全局规划）；base_link：车体系（原点即车辆，航向 0）。
//   TF 由定位/仿真发布；消费者按消息 header.frame_id 显式选择解释路径。
inline constexpr std::string_view kFrameMap = "map";
inline constexpr std::string_view kFrameBaseLink = "base_link";

// —— 有效性哨兵（issue #11/#12）：不得与合法值混用 ——
inline constexpr double kUnknownSpeed = std::numeric_limits<double>::quiet_NaN();  // 缺失速度用 NaN，绝非 0
inline constexpr double kUnknownDeviceTime = -1.0;                                 // 设备时间未知（<0）

// —— 时钟域（issue #14/#24）：header.stamp = 发布时刻；设备测量时刻经 device_* 字段透传，
//    use_sim_time 下由 /clock 驱动，两者不得互相替代。

[[nodiscard]] inline constexpr bool isFrameSupported(std::string_view frame) noexcept {
    return frame == kFrameMap || frame == kFrameBaseLink;
}

[[nodiscard]] inline constexpr bool isFinite(double v) noexcept {
    return std::isfinite(v);
}

// 几何轨迹：至少 2 点，且所有 x/y 有限。
[[nodiscard]] inline bool geometryValid(std::span<const double> xs, std::span<const double> ys) noexcept {
    if (xs.size() < 2 || xs.size() != ys.size())
        return false;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (!isFinite(xs[i]) || !isFinite(ys[i]))
            return false;
    }
    return true;
}

// 参考速度数组（issue #11）：与路径等长时视为“显式有效”，逐点须有限且 >=0（允许 0=停车）。
// 返回 true 表示应使用 target_speeds；false 表示缺失（消费者按契约不得静默把缺失当 0）。
[[nodiscard]] inline bool targetSpeedsEffective(std::span<const double> target_speeds, std::size_t path_size) noexcept {
    if (target_speeds.size() != path_size || path_size == 0)
        return false;
    for (double s : target_speeds) {
        if (!isFinite(s) || s < 0.0)
            return false;  // 含 NaN/负值 => 该数组不可信
    }
    return true;
}

// 消费者逐点参考速度选择（issue #14）：这是“速度载体唯一由 target_speeds 表达”的集中判定点。
//   - has_explicit_speeds==true：返回 target_speed（来自 target_speeds[i]，允许 0=停车，valid 当且仅当有限且 >=0）；
//   - 否则：缺失速度，回退到调用方配置的 default_reference_speed（仅当 >0.1 才视为可行驶，
//     否则 valid=false 触发降级/停车）。
// 关键契约：本函数**从不接受几何 Point.z 作速度输入** —— z 仅作几何占位，杜绝旧“z 藏速度”回退复活。
struct SpeedSelection {
    double speed;
    bool valid;
};
[[nodiscard]] inline constexpr SpeedSelection selectReferenceSpeed(bool has_explicit_speeds, double target_speed,
                                                                   double default_reference_speed) noexcept {
    if (has_explicit_speeds)
        return {target_speed, isFinite(target_speed) && target_speed >= 0.0};
    return {default_reference_speed, isFinite(default_reference_speed) && default_reference_speed > 0.1};
}

// 来源年龄校验（issue #12）：age 为非负且不超过容差才视为新鲜。tolerance<0 表示禁用该检查。
[[nodiscard]] inline constexpr bool sourceAgeAcceptable(double age_sec, double tolerance_sec) noexcept {
    if (tolerance_sec < 0.0)
        return true;  // 检查禁用
    return isFinite(age_sec) && age_sec >= 0.0 && age_sec <= tolerance_sec;
}

// QoS 契约（描述符，非 rclcpp 类型；消费者据此构造 rclcpp::QoS）：
//   state/command/path：reliable / volatile / 有限深度（高频遥测，最新为准）。
//   stop：reliable / transient_local / KeepLast(1)（锁存，保证重启/晚加入收到最后停车态，见 #8）。
struct QosSpec {
    std::size_t depth;
    bool reliable;
    bool transient_local;
};
inline constexpr QosSpec kQosState{10, true, false};
inline constexpr QosSpec kQosPath{1, true, false};
inline constexpr QosSpec kQosCommand{10, true, false};
inline constexpr QosSpec kQosStop{1, true, true};

}  // namespace contract
}  // namespace common_msgs
