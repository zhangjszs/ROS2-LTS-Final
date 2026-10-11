#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <ranges>
#include <span>
#include <vector>

namespace skidpad {

struct Point2D {
    double x;
    double y;
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
    auto operator<=>(const Point2D&) const = default;
};

struct CenterlineParams {
    double max_pair_dist{6.0};
    double path_spacing{0.5};
    double path_lookahead{20.0};
};

inline void AppendMidpointsCore(std::span<const Point2D> primary, std::span<const Point2D> secondary, double max_dist,
                                std::vector<Point2D>& out) {
    if (primary.empty() || secondary.empty()) {
        return;
    }

    const double max_dist_sq = max_dist * max_dist;

    for (const auto& p : primary) {
        const float qx = static_cast<float>(p.x);
        const float qy = static_cast<float>(p.y);
        float best_dist_sq = std::numeric_limits<float>::infinity();
        size_t best_idx = 0;
        bool found = false;

        for (size_t j = 0; j < secondary.size(); ++j) {
            const float sx = static_cast<float>(secondary[j].x);
            const float sy = static_cast<float>(secondary[j].y);
            const double dx = static_cast<double>(qx - sx);
            const double dy = static_cast<double>(qy - sy);
            const float d2 = static_cast<float>((dx * dx) + (dy * dy));
            if (d2 < best_dist_sq) {
                best_dist_sq = d2;
                best_idx = j;
                found = true;
            }
        }

        if (found && best_dist_sq <= max_dist_sq) {
            out.emplace_back((p.x + secondary[best_idx].x) * 0.5, (p.y + secondary[best_idx].y) * 0.5);
        }
    }
}

inline std::vector<Point2D> ComputeCenterlineCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                  const CenterlineParams& p) {
    if (left.empty() || right.empty()) {
        return {};
    }

    std::vector<Point2D> centerline;
    centerline.reserve(left.size() + right.size());

    AppendMidpointsCore(left, right, p.max_pair_dist, centerline);
    AppendMidpointsCore(right, left, p.max_pair_dist, centerline);

    if (centerline.empty()) {
        return {};
    }

    // 从原点最近邻串起来，避免按 x 排序在圆环上锯齿化
    std::vector<Point2D> ordered;
    ordered.reserve(centerline.size());
    std::vector<bool> used(centerline.size(), false);
    auto take_nearest = [&](double x0, double y0) -> int {
        auto unused_indices =
            std::views::iota(size_t{0}, centerline.size()) | std::views::filter([&used](size_t i) { return !used[i]; });
        auto it = std::ranges::min_element(unused_indices, {}, [&](size_t i) {
            const double dx = centerline[i].x - x0;
            const double dy = centerline[i].y - y0;
            return (dx * dx) + (dy * dy);
        });
        if (it == unused_indices.end()) {
            return -1;
        }
        return static_cast<int>(*it);
    };
    int start = take_nearest(0.0, 0.0);
    while (start >= 0) {
        used[static_cast<size_t>(start)] = true;
        ordered.push_back(centerline[static_cast<size_t>(start)]);
        start = take_nearest(ordered.back().x, ordered.back().y);
    }
    centerline.swap(ordered);

    const double dedup_sq = (p.path_spacing * 0.5) * (p.path_spacing * 0.5);
    std::vector<Point2D> deduped;
    deduped.reserve(centerline.size());
    for (const auto& pt : centerline) {
        if (deduped.empty()) {
            deduped.push_back(pt);
            continue;
        }
        const double dx = pt.x - deduped.back().x;
        const double dy = pt.y - deduped.back().y;
        if ((dx * dx) + (dy * dy) > dedup_sq) {
            deduped.push_back(pt);
        }
    }

    std::vector<Point2D> result;
    result.reserve(deduped.size());
    const double lookahead_sq = p.path_lookahead * p.path_lookahead;
    std::ranges::copy_if(
        deduped, std::back_inserter(result), [lookahead_sq](double d2) { return d2 <= lookahead_sq; },
        [](const Point2D& pt) { return (pt.x * pt.x) + (pt.y * pt.y); });
    return result;
}

}  // namespace skidpad
