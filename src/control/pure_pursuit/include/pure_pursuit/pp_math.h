#pragma once

#include <algorithm>
#include <cmath>
#include <compare>
#include <concepts>
#include <limits>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace pp_math {

// C++20 空间点概念约束：支持任意具有 x, y 坐标的二维/三维点类型（如 geometry_msgs::msg::Point, HuatCone 等）
template <typename T>
concept Point2DLike = requires(const T& p) {
    { p.x } -> std::convertible_to<double>;
    { p.y } -> std::convertible_to<double>;
};

// 将 idx 钳位到三点法有效范围 [1, n-2]
inline int clampKappaIdx(int idx, int n) {
    return std::max(1, std::min(idx, n - 2));
}

// 三点法曲率估算：κ = 2*area / (|ab|*|bc|*|ca|) (C++20 std::span 零拷贝视图)
inline double estimateCurvature(std::span<const double> refx, std::span<const double> refy, int idx) {
    const int n = static_cast<int>(refx.size());
    if (n < 3 || idx < 1 || idx >= n - 1)
        return 0.0;

    double ax = refx[idx - 1], ay = refy[idx - 1];
    double bx = refx[idx], by = refy[idx];
    double cx = refx[idx + 1], cy = refy[idx + 1];

    double ab = std::hypot(bx - ax, by - ay);
    double bc = std::hypot(cx - bx, cy - by);
    double ca = std::hypot(ax - cx, ay - cy);

    // area2 = 叉积模 = 2×三角形面积；κ = 4×area / (|ab|×|bc|×|ca|) = 2×area2 / denom
    double area2 = std::abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay));
    if (area2 < 1e-9)
        return 0.0;

    double denom = ab * bc * ca;
    if (denom < 1e-9)
        return 0.0;

    return 2.0 * area2 / denom;
}

struct NearestIndexResult {
    int idx = -1;
    double dist_sq = 0.0;

    auto operator<=>(const NearestIndexResult&) const = default;
};

// 在 [search_start, search_end) 内找距 (cx, cy) 最近的路径点
// C++20 现代 Ranges 算法与投影重构：基于 std::views::iota 与 std::ranges::min_element，无堆内存分配
inline NearestIndexResult findNearestIndex(std::span<const double> refx, std::span<const double> refy, double cx,
                                           double cy, int search_start, int search_end) {
    NearestIndexResult result;
    const int n = static_cast<int>(refx.size());
    if (n == 0 || static_cast<int>(refy.size()) != n || search_start >= search_end || search_start < 0)
        return result;
    search_end = std::min(search_end, n);

    auto indices = std::views::iota(search_start, search_end);
    auto compute_dist_sq = [&](int i) -> double {
        double dx = cx - refx[i];
        double dy = cy - refy[i];
        return dx * dx + dy * dy;
    };

    auto best_it = std::ranges::min_element(indices, {}, compute_dist_sq);
    if (best_it != indices.end()) {
        result.idx = *best_it;
        result.dist_sq = compute_dist_sq(result.idx);
    }
    return result;
}

// 基于折线累积距离的流式前瞻点搜索 (C++20 Ranges pipeline)
// 利用 std::views::iota 生成段索引区间，并通过 std::views::transform 惰性计算每段折线距离
inline int findLookaheadIndex(std::span<const double> refx, std::span<const double> refy,
                              int current_idx, double lookahead) {
    const int n = static_cast<int>(refx.size());
    if (current_idx < 0 || n == 0 || static_cast<int>(refy.size()) != n)
        return 0;

    double distance_sum = 0.0;
    int target_idx = current_idx;

    auto segments = std::views::iota(current_idx, std::max(current_idx, n - 1))
        | std::views::transform([&](int i) {
            double dx = refx[i + 1] - refx[i];
            double dy = refy[i + 1] - refy[i];
            return std::make_pair(i + 1, std::hypot(dx, dy));
        });

    for (const auto& [next_idx, seg_dist] : segments) {
        if (distance_sum + seg_dist <= lookahead) {
            distance_sum += seg_dist;
            target_idx = next_idx;
        } else {
            break;
        }
    }
    return target_idx;
}

// 基于欧氏距离的流式前瞻点搜索 (C++20 管道式流水线: iota -> transform -> filter)
// 惰性求值：仅在迭代时计算距离，首个满足 lookahead 门限的点即刻终止管道，零多余运算
inline int findLookaheadIndexEuclidean(std::span<const double> refx, std::span<const double> refy,
                                       int current_idx, double lookahead) {
    const int n = static_cast<int>(refx.size());
    if (current_idx < 0 || n == 0 || static_cast<int>(refy.size()) != n)
        return 0;

    const double lookahead_sq = lookahead * lookahead;
    const double ox = refx[current_idx];
    const double oy = refy[current_idx];

    auto ahead_points = std::views::iota(current_idx, n)
        | std::views::transform([&](int i) {
            double dx = refx[i] - ox;
            double dy = refy[i] - oy;
            return std::make_pair(i, dx * dx + dy * dy);
        })
        | std::views::filter([lookahead_sq](const auto& pt) {
            return pt.second >= lookahead_sq;
        });

    auto it = ahead_points.begin();
    if (it != ahead_points.end()) {
        return (*it).first;
    }
    return n - 1;
}

// ── Point2DLike 连续点集视图重载 ──────────────────────────────────────────

// 三点法曲率估算（Point2DLike 点集视图）
template <Point2DLike PointType>
inline double estimateCurvature(std::span<PointType> points, int idx) {
    const int n = static_cast<int>(points.size());
    if (n < 3 || idx < 1 || idx >= n - 1)
        return 0.0;

    double ax = points[idx - 1].x, ay = points[idx - 1].y;
    double bx = points[idx].x, by = points[idx].y;
    double cx = points[idx + 1].x, cy = points[idx + 1].y;

    double ab = std::hypot(bx - ax, by - ay);
    double bc = std::hypot(cx - bx, cy - by);
    double ca = std::hypot(ax - cx, ay - cy);

    double area2 = std::abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay));
    if (area2 < 1e-9)
        return 0.0;

    double denom = ab * bc * ca;
    if (denom < 1e-9)
        return 0.0;

    return 2.0 * area2 / denom;
}

// 在 [search_start, search_end) 内找距 (cx, cy) 最近的路径点（Point2DLike 点集视图）
template <Point2DLike PointType>
inline NearestIndexResult findNearestIndex(std::span<PointType> points, double cx,
                                           double cy, int search_start, int search_end) {
    NearestIndexResult result;
    const int n = static_cast<int>(points.size());
    if (n == 0 || search_start >= search_end || search_start < 0)
        return result;
    search_end = std::min(search_end, n);

    auto indices = std::views::iota(search_start, search_end);
    auto compute_dist_sq = [&](int i) -> double {
        double dx = cx - points[i].x;
        double dy = cy - points[i].y;
        return dx * dx + dy * dy;
    };

    auto best_it = std::ranges::min_element(indices, {}, compute_dist_sq);
    if (best_it != indices.end()) {
        result.idx = *best_it;
        result.dist_sq = compute_dist_sq(result.idx);
    }
    return result;
}

// 基于折线累积距离的前瞻点搜索（Point2DLike 点集视图）
template <Point2DLike PointType>
inline int findLookaheadIndex(std::span<PointType> points,
                              int current_idx, double lookahead) {
    const int n = static_cast<int>(points.size());
    if (current_idx < 0 || n == 0)
        return 0;

    double distance_sum = 0.0;
    int target_idx = current_idx;

    auto segments = std::views::iota(current_idx, std::max(current_idx, n - 1))
        | std::views::transform([&](int i) {
            double dx = points[i + 1].x - points[i].x;
            double dy = points[i + 1].y - points[i].y;
            return std::make_pair(i + 1, std::hypot(dx, dy));
        });

    for (const auto& [next_idx, seg_dist] : segments) {
        if (distance_sum + seg_dist <= lookahead) {
            distance_sum += seg_dist;
            target_idx = next_idx;
        } else {
            break;
        }
    }
    return target_idx;
}

// 基于欧氏距离的前瞻点搜索（Point2DLike 点集视图）
template <Point2DLike PointType>
inline int findLookaheadIndexEuclidean(std::span<PointType> points,
                                       int current_idx, double lookahead) {
    const int n = static_cast<int>(points.size());
    if (current_idx < 0 || n == 0)
        return 0;

    const double lookahead_sq = lookahead * lookahead;
    const double ox = points[current_idx].x;
    const double oy = points[current_idx].y;

    auto ahead_points = std::views::iota(current_idx, n)
        | std::views::transform([&](int i) {
            double dx = points[i].x - ox;
            double dy = points[i].y - oy;
            return std::make_pair(i, dx * dx + dy * dy);
        })
        | std::views::filter([lookahead_sq](const auto& pt) {
            return pt.second >= lookahead_sq;
        });

    auto it = ahead_points.begin();
    if (it != ahead_points.end()) {
        return (*it).first;
    }
    return n - 1;
}

inline bool isBaseLinkFrame(const std::string& frame_id) {
    return frame_id == "base_link";
}

}  // namespace pp_math
