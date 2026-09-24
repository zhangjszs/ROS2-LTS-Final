#pragma once
// ============================================================================
// issue #22：MPC 路径参考点构建（core，纯 std、无 ROS context）。
//
// 从 mpc_controller_node.cpp 的 OnPath 内联实现抽出，逐条件等价：
//   坐标系门禁（contract::isFrameSupported）→ target_speeds 有效性探测与
//   逐点参考速度选择（contract::targetSpeedsEffective / selectReferenceSpeed，
//   绝不读几何 Point.z）→ 相邻点航向角（atan2）与曲率（离散近似）→
//   std::vector<ReferencePoint>。
//
// 依赖仅 std + 契约层 + mpc_types.hpp（不含 rclcpp / ROS 消息 / Eigen），
// 可由 gtest 脱离 ROS 独立编译验证；调用方保留副作用与置位时机
// （frame 拒绝 → has_path_=false；点数不足 → 不改路径有效性）。
// ============================================================================
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "interface_contract.h"  // 仓库约定：common_msgs 手写头不带前缀
#include "mpc_controller/mpc_types.hpp"

namespace mpc_core {

// 平面点视图的最小约束（ROS geometry_msgs Point / 测试桩结构均可满足）。
template <typename P>
concept PlanarPointLike = requires(const P& p) {
    { p.x } -> std::convertible_to<double>;
    { p.y } -> std::convertible_to<double>;
};

// 构建结果状态：
//   Ok            — 构建成功，points 为该帧参考路径；
//   RejectedFrame — header.frame_id 不受支持（既非 map 也非 base_link），调用方应进入降级；
//   TooFewPoints  — 点数 < 2，无法估计航向/曲率；调用方不得更新路径有效性。
enum class ReferencePathStatus : std::uint8_t {
    Ok = 0,
    RejectedFrame = 1,
    TooFewPoints = 2,
};

struct ReferencePathResult {
    // 兜底为 TooFewPoints（静默不生效）；构建函数在所有出口显式设置状态。
    ReferencePathStatus status{ReferencePathStatus::TooFewPoints};
    // 路径坐标系标记（仅非 RejectedFrame 时有意义）：base_link 局部路径契约下车辆视为原点
    bool in_base_frame{false};
    std::vector<mpc::ReferencePoint> points;  // 仅 Ok 时为构建结果
};

/**
 * @brief 由原始路径点 + 目标速度数组构建 MPC 参考路径（纯函数，无副作用）。
 * @param frame                 路径消息 header.frame_id；仅 map / base_link 受支持
 * @param path                  原始路径几何点（至少 2 点才产出结果）
 * @param target_speeds         显式目标速度数组；与 path 等长且逐点有限非负时才生效
 * @param default_reference_speed 缺失显式速度时的回退参考速度（>0.1 才视为可行驶）
 */
template <PlanarPointLike P>
[[nodiscard]] ReferencePathResult BuildReferencePath(std::string_view frame, std::span<const P> path,
                                                     std::span<const double> target_speeds,
                                                     double default_reference_speed) {
    ReferencePathResult result;
    if (!common_msgs::contract::isFrameSupported(frame)) {
        result.status = ReferencePathStatus::RejectedFrame;
        return result;
    }
    result.in_base_frame = (frame == common_msgs::contract::kFrameBaseLink);
    if (path.size() < 2) {
        result.status = ReferencePathStatus::TooFewPoints;
        return result;
    }

    const bool has_explicit_speeds = common_msgs::contract::targetSpeedsEffective(target_speeds, path.size());

    auto& pts = result.points;
    pts.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
        mpc::ReferencePoint pt;
        pt.x = static_cast<double>(path[i].x);
        pt.y = static_cast<double>(path[i].y);
        // #14：速度载体唯一为 target_speeds；缺失时回退到配置的默认参考速度，绝不读 Point.z。
        const double target_speed = has_explicit_speeds ? target_speeds[i] : 0.0;
        const auto speed =
            common_msgs::contract::selectReferenceSpeed(has_explicit_speeds, target_speed, default_reference_speed);
        pt.speed = speed.speed;
        pt.speed_valid = speed.valid;

        // 航向角由相邻点差分给出；末点继承前一点航向。
        if (i + 1 < path.size()) {
            const double dx = static_cast<double>(path[i + 1].x) - pt.x;
            const double dy = static_cast<double>(path[i + 1].y) - pt.y;
            pt.theta = std::atan2(dy, dx);
        } else if (!pts.empty()) {
            pt.theta = pts.back().theta;
        }

        // 曲率 = 航向变化 / 弧长（离散近似）；首末点与退化段（ds≈0）记 0。
        if (i > 0 && i + 1 < path.size()) {
            const double dtheta = mpc::NormalizeAngle(pt.theta - pts.back().theta);
            const double ds = std::hypot(static_cast<double>(path[i].x) - static_cast<double>(path[i - 1].x),
                                         static_cast<double>(path[i].y) - static_cast<double>(path[i - 1].y));
            pt.curvature = (ds > 1e-3) ? (dtheta / ds) : 0.0;
        } else {
            pt.curvature = 0.0;
        }

        pts.push_back(pt);
    }
    result.status = ReferencePathStatus::Ok;
    return result;
}

// —— 局部/全局坐标系契约（issue #3）：base_link 下车辆视为原点（航向 0），
//    map 下用车辆全局位姿作为参考系原点，与 Pure Pursuit 同语义。 ——
struct ReferenceOrigin {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

[[nodiscard]] inline constexpr ReferenceOrigin SelectReferenceOrigin(bool path_in_base_frame, double current_x,
                                                                     double current_y, double current_theta) noexcept {
    if (path_in_base_frame) {
        return ReferenceOrigin{0.0, 0.0, 0.0};
    }
    return ReferenceOrigin{current_x, current_y, current_theta};
}

}  // namespace mpc_core
