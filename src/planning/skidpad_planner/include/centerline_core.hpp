#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
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

struct Circle2D {
    double cx{0.0};
    double cy{0.0};
    double r{0.0};
    bool valid{false};
    auto operator<=>(const Circle2D&) const = default;
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

inline Circle2D FitCircleTaubinCore(std::span<const Point2D> cones) {
    Circle2D result;
    const int n = static_cast<int>(cones.size());
    if (n < 3) {
        return result;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& c : cones) {
        mean_x += c.x;
        mean_y += c.y;
    }
    mean_x /= static_cast<double>(n);
    mean_y /= static_cast<double>(n);

    double Mxx = 0.0;
    double Myy = 0.0;
    double Mxy = 0.0;
    double Mxz = 0.0;
    double Myz = 0.0;
    double Mzz = 0.0;
    for (const auto& c : cones) {
        const double u = c.x - mean_x;
        const double v = c.y - mean_y;
        const double z = (u * u) + (v * v);
        Mxx += u * u;
        Myy += v * v;
        Mxy += u * v;
        Mxz += u * z;
        Myz += v * z;
        Mzz += z * z;
    }
    Mxx /= static_cast<double>(n);
    Myy /= static_cast<double>(n);
    Mxy /= static_cast<double>(n);
    Mxz /= static_cast<double>(n);
    Myz /= static_cast<double>(n);
    Mzz /= static_cast<double>(n);
    const double Mz = Mxx + Myy;
    if (Mz < 1e-10) {
        return result;
    }

    Eigen::Matrix3d A;
    A << Mzz / (4.0 * Mz), Mxz / (4.0 * Mz), Myz / (4.0 * Mz), Mxz, Mxx, Mxy, Myz, Mxy, Myy;

    Eigen::EigenSolver<Eigen::Matrix3d> es(A);

    double min_eval = std::numeric_limits<double>::max();
    Eigen::Vector3d min_evec = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(es.eigenvalues()[i].imag()) > 1e-8) {
            continue;
        }
        const double eval = es.eigenvalues()[i].real();
        if (eval > 1e-10 && eval < min_eval) {
            min_eval = eval;
            min_evec = es.eigenvectors().col(i).real();
        }
    }
    if (min_eval == std::numeric_limits<double>::max()) {
        return result;
    }

    const double A_coef = min_evec(0);
    if (std::abs(A_coef) < 1e-10) {
        return result;
    }

    const double cx_c = -min_evec(1) / (2.0 * A_coef);
    const double cy_c = -min_evec(2) / (2.0 * A_coef);
    const double r_sq = (cx_c * cx_c) + (cy_c * cy_c) + Mz;
    if (r_sq <= 0.0) {
        return result;
    }

    result.cx = cx_c + mean_x;
    result.cy = cy_c + mean_y;
    result.r = std::sqrt(r_sq);
    result.valid = true;
    return result;
}

inline std::vector<Point2D> ComputeCircleCenterlineCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                        const CenterlineParams& p) {
    // Parameter guard: turn the float-to-int UB (`static_cast<int>(+inf)` when
    // path_spacing == 0, plus NaN slip-through of naive `<= 0.0` checks) into
    // defined empty output so the caller takes its existing fallback path.
    if (!(p.path_spacing > 0.0) || !(p.path_lookahead > 0.0)) {
        return {};
    }
    const double steps_exact = p.path_lookahead / p.path_spacing;
    if (!std::isfinite(steps_exact)) {
        return {};
    }

    constexpr double kDefaultHalfTrackWidth = 1.5;
    constexpr double kMinRadius = 3.0;
    constexpr double kMaxRadius = 25.0;

    const Circle2D lc = FitCircleTaubinCore(left);
    const Circle2D rc = FitCircleTaubinCore(right);

    Circle2D center_circle;
    if (lc.valid && rc.valid) {
        center_circle.cx = (lc.cx + rc.cx) * 0.5;
        center_circle.cy = (lc.cy + rc.cy) * 0.5;
        center_circle.r = (lc.r + rc.r) * 0.5;
        center_circle.valid = true;
    } else if (lc.valid) {
        center_circle.cx = lc.cx;
        center_circle.cy = lc.cy;
        center_circle.r = std::max(lc.r - kDefaultHalfTrackWidth, lc.r * 0.5);
        center_circle.valid = true;
    } else if (rc.valid) {
        center_circle.cx = rc.cx;
        center_circle.cy = rc.cy;
        center_circle.r = std::max(rc.r - kDefaultHalfTrackWidth, rc.r * 0.5);
        center_circle.valid = true;
    }

    if (!center_circle.valid || center_circle.r < kMinRadius || center_circle.r > kMaxRadius) {
        return {};
    }

    const double start_angle = std::atan2(-center_circle.cy, -center_circle.cx);
    const double arc_step = p.path_spacing / center_circle.r;
    auto first_x = [&](double dir) {
        const double angle = start_angle + (dir * arc_step);
        return center_circle.cx + (center_circle.r * std::cos(angle));
    };
    const double dir = (first_x(1.0) >= first_x(-1.0)) ? 1.0 : -1.0;
    const int num_steps = static_cast<int>(steps_exact) + 1;

    std::vector<Point2D> path;
    path.reserve(static_cast<size_t>(num_steps));
    for (int i = 1; i <= num_steps; ++i) {
        const double angle = start_angle + ((dir * arc_step) * static_cast<double>(i));
        const double px = center_circle.cx + (center_circle.r * std::cos(angle));
        const double py = center_circle.cy + (center_circle.r * std::sin(angle));
        path.emplace_back(px, py);
    }
    return path;
}

}  // namespace skidpad
