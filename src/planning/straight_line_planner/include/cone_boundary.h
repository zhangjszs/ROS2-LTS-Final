#pragma once
// ============================================================================
// issue #22：straight_line_planner 锥桶边界 core（纯 std、无 ROS context）。
//
// 从 line_detector.cpp 的 ClusterCones 与 straight_line_planner_node.cpp 的
// IsBoundaryPlausible / single_side_ok / BuildPathLimits 内联分离谓词抽出，
// 逐条件等价：
//   1) 锥桶按 base_link 横向 y 分离左右（y < -margin 归左、y > margin 归右；
//      中心带与 NaN 不进入任何一侧；索引保持输入顺序）；
//   2) 左右边界的可接受性判定（双侧 valid → 斜率限 → 截距顺序 → 原点宽度）；
//   3) 单侧可用性判定（恰一侧 valid 且其斜率在限内）。
//
// 依赖仅 std（<cmath>/<concepts>/<cstddef>/<cstdint>/<span>/<vector>），
// 可由 gtest 脱离 ROS 独立编译验证；日志等 ROS 副作用留在节点侧
// （判定返回首个失败原因枚举，供调用方逐条分流）。
// ============================================================================
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace slp_core {

// 锥桶视图的最小约束：base_link 下的横向坐标 y（车体系：左负右正）。
template <typename C>
concept BaseLinkConeLike = requires(const C& c) {
    { c.position_base_link.y } -> std::convertible_to<double>;
};

// 单侧边界视图的最小约束（对应 LineParams：y = slope * x + intercept）。
template <typename L>
concept LineParamsLike = requires(const L& l) {
    { l.slope } -> std::convertible_to<double>;
    { l.intercept } -> std::convertible_to<double>;
    { l.valid } -> std::convertible_to<bool>;
};

// 左右分离结果：左/右锥桶在原始输入中的索引，均保持输入顺序。
// 中心带（|y| <= center_margin）与 NaN（两个比较都为 false）不进入任何一侧。
struct ConeSideSplit {
    std::vector<std::size_t> left;
    std::vector<std::size_t> right;
};

/**
 * @brief 按 base_link 横向坐标把锥桶分离为左右两侧（纯函数，无副作用）。
 * @param cones         锥桶视图（仅读取 position_base_link.y）
 * @param center_margin 车道中心忽略带半宽：y < -center_margin 归左、y > center_margin 归右
 */
template <BaseLinkConeLike C>
[[nodiscard]] ConeSideSplit SplitConesBySide(std::span<const C> cones, double center_margin) {
    ConeSideSplit split;
    split.left.reserve(cones.size());
    split.right.reserve(cones.size());
    for (std::size_t i = 0; i < cones.size(); ++i) {
        const double y = static_cast<double>(cones[i].position_base_link.y);
        if (y < -center_margin) {
            split.left.push_back(i);
        } else if (y > center_margin) {
            split.right.push_back(i);
        }
    }
    return split;
}

// 原点处最小允许车道宽度（m）。原实现内联常量（0.5），集中于此避免散落硬编码。
inline constexpr double kMinWidthAtOrigin = 0.5;

// 判定阈值：默认值与节点 declare_parameter 的默认一致（0.3 / 3.0）；
// 节点始终显式传入运行时参数，此处默认仅服务于独立调用与单测。
struct PlausibilityThresholds {
    double max_abs_slope{0.3};
    double max_intercept_diff{3.0};
};

// 判定失败原因，与原节点逐条 DEBUG 日志一一对应；None 表示判定通过。
enum class PlausibilityIssue : std::uint8_t {
    None = 0,
    NotBothValid = 1,
    LeftSlopeExceeds = 2,
    RightSlopeExceeds = 3,
    InterceptOrderViolated = 4,
    WidthOutOfRange = 5,
};

// 判定结论：plausible 为最终结果；issue 为首个失败原因（供调用方分流日志）。
struct PlausibilityVerdict {
    bool plausible{false};
    PlausibilityIssue issue{PlausibilityIssue::NotBothValid};
};

/**
 * @brief 判定左右边界是否可接受（纯函数，无副作用）。
 *
 * 判定顺序与原实现逐条一致：双侧 valid → |左斜率| 限 → |右斜率| 限 →
 * 右截距 > 左截距（base_link 中右为 +y）→ 原点宽度 ∈ [kMinWidthAtOrigin, 2*max_intercept_diff]。
 * NaN 行为亦与原实现一致（例如 NaN 斜率在“大于上限”比较中为 false，不在此处收紧）。
 */
template <LineParamsLike L, LineParamsLike R>
[[nodiscard]] PlausibilityVerdict EvaluateBoundaryPlausibility(const L& left, const R& right,
                                                               const PlausibilityThresholds& thresholds) {
    PlausibilityVerdict verdict;
    if (!left.valid || !right.valid) {
        verdict.issue = PlausibilityIssue::NotBothValid;
        return verdict;
    }
    if (std::abs(static_cast<double>(left.slope)) > thresholds.max_abs_slope) {
        verdict.issue = PlausibilityIssue::LeftSlopeExceeds;
        return verdict;
    }
    if (std::abs(static_cast<double>(right.slope)) > thresholds.max_abs_slope) {
        verdict.issue = PlausibilityIssue::RightSlopeExceeds;
        return verdict;
    }
    // 右侧截距应大于左侧截距（在 base_link 中右侧为 +y，左侧为 -y）。
    if (!(static_cast<double>(right.intercept) > static_cast<double>(left.intercept))) {
        verdict.issue = PlausibilityIssue::InterceptOrderViolated;
        return verdict;
    }
    const double width_at_origin = static_cast<double>(right.intercept) - static_cast<double>(left.intercept);
    if (width_at_origin < kMinWidthAtOrigin || width_at_origin > 2.0 * thresholds.max_intercept_diff) {
        verdict.issue = PlausibilityIssue::WidthOutOfRange;
        return verdict;
    }
    verdict.plausible = true;
    verdict.issue = PlausibilityIssue::None;
    return verdict;
}

/**
 * @brief 单侧可用性判定（对应节点 single_side_ok 的纯逻辑）：
 * 恰有一侧 valid，且该侧斜率绝对值不超过 max_abs_slope。
 */
template <LineParamsLike L, LineParamsLike R>
[[nodiscard]] bool IsSingleSideUsable(const L& left, const R& right, double max_abs_slope) {
    if (left.valid == right.valid) {
        return false;
    }
    const double slope = left.valid ? static_cast<double>(left.slope) : static_cast<double>(right.slope);
    return std::abs(slope) <= max_abs_slope;
}

}  // namespace slp_core
