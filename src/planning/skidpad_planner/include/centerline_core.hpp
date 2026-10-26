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
    // #95：拟合残差（RMS，米）。真正的圆弧残差≈0；共线点/退化输入的代数拟合
    // 残差很大 —— 这是「拟合出来的根本不是圆」的可判定信号（部分弧拟合失稳的
    // 第一道护栏）。
    double rms_residual{0.0};
    auto operator<=>(const Circle2D&) const = default;
};

struct CenterlineParams {
    double max_pair_dist{6.0};
    double path_spacing{0.5};
    double path_lookahead{20.0};
    // #95：走廊中线识别的两个几何判据容差（米）。
    // - phase_tol：候选圆必须满足「自车在该圆上」（‖ego − C‖ ≈ 壁半径）的容差；
    // - concentric_tol：左右两组拟合圆的**圆心距**小于此值即判为「同一条走廊的
    //   内外两壁」（此时半径取平均是几何正确的），否则判为「左右落在不同的瓣」。
    double phase_tol{3.0};
    double concentric_tol{1.0};
    // #95：圆拟合残差上限（RMS，米）——超过即判「拟合出来的不是圆」，不得用。
    double max_fit_rms{0.5};
};

struct IcpApfConfig {
    double center_margin = 0.3;
    int icp_max_iter = 20;
    double icp_convergence = 0.001;
    double k_a = 0.5;
    double k_r = 2.0;
    double d_0 = 2.0;
    double path_spacing = 0.5;
    double path_lookahead = 20.0;
    int apf_max_iter = 10;
    double apf_step_limit = 0.2;
    double icp_max_match_dist = 3.0;
};

struct ConfigParamFix {
    const char* name;
    double bad_value;
    double used_value;
};

struct SanitizedSkidpadConfig {
    IcpApfConfig cfg;
    std::vector<ConfigParamFix> fixes;
};

// #89：skidpad 核心边界参数清洗纯函数。非法输入回退默认值并记录 fix。
[[nodiscard]] inline SanitizedSkidpadConfig SanitizeSkidpadConfig(const IcpApfConfig& cfg) {
    const IcpApfConfig defaults{};
    SanitizedSkidpadConfig out;
    out.cfg = cfg;

    auto fix_if_invalid_double = [&out](const char* name, double& value, double fallback, bool valid) {
        if (!valid) {
            out.fixes.push_back(ConfigParamFix{name, value, fallback});
            value = fallback;
        }
    };

    auto fix_if_invalid_int = [&out](const char* name, int& value, int fallback, bool valid) {
        if (!valid) {
            out.fixes.push_back(ConfigParamFix{name, static_cast<double>(value), static_cast<double>(fallback)});
            value = fallback;
        }
    };

    fix_if_invalid_double("center_margin", out.cfg.center_margin, defaults.center_margin,
                          std::isfinite(out.cfg.center_margin) && out.cfg.center_margin >= 0.0);
    fix_if_invalid_double("k_a", out.cfg.k_a, defaults.k_a, std::isfinite(out.cfg.k_a) && out.cfg.k_a >= 0.0);
    fix_if_invalid_double("k_r", out.cfg.k_r, defaults.k_r, std::isfinite(out.cfg.k_r) && out.cfg.k_r >= 0.0);
    fix_if_invalid_double("d_0", out.cfg.d_0, defaults.d_0, std::isfinite(out.cfg.d_0) && out.cfg.d_0 > 0.0);
    fix_if_invalid_double("path_station_spacing", out.cfg.path_spacing, defaults.path_spacing,
                          std::isfinite(out.cfg.path_spacing) && out.cfg.path_spacing > 0.0);
    fix_if_invalid_double("path_lookahead_distance", out.cfg.path_lookahead, defaults.path_lookahead,
                          std::isfinite(out.cfg.path_lookahead) && out.cfg.path_lookahead > 0.0);
    fix_if_invalid_double("apf_step_limit", out.cfg.apf_step_limit, defaults.apf_step_limit,
                          std::isfinite(out.cfg.apf_step_limit) && out.cfg.apf_step_limit > 0.0);
    fix_if_invalid_int("icp_max_iterations", out.cfg.icp_max_iter, defaults.icp_max_iter, out.cfg.icp_max_iter > 0);

    return out;
}

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

// #95：**互为最近邻**的跨墙配对（只保留双向一致的配对）。
//
// AppendMidpointsCore 的配对距离上限较宽（max_pair_dist = 6 m，而走廊宽只有
// 3 m），于是会出现「单向配对」：某侧锥桶的最近对侧锥桶，与对侧锥桶的最近
// 同侧锥桶**不是同一个**。这类配对的中点不在走廊中线上（实测在直线段产出
// y ≈ −1.1 m 的伪中点），并且与正常中点交错分布 → 路径点序来回跳。
//
// 判据：只有当 left[i] 的最近对侧点是 right[j]、**且** right[j] 的最近同侧点
// 正是 left[i]（互为最近邻）时，这一对才成立。本函数保留这些中点，丢掉单向配对。
inline std::vector<Point2D> AppendMutualMidpointsCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                      double max_dist) {
    std::vector<Point2D> out;
    if (left.empty() || right.empty() || !(max_dist > 0.0)) {
        return out;
    }
    const double max_dist_sq = max_dist * max_dist;

    // right[j] → 最近的 left 下标（用于互判）
    std::vector<int> nearest_left(right.size(), -1);
    std::vector<double> nearest_left_d2(right.size(), std::numeric_limits<double>::max());
    for (size_t j = 0; j < right.size(); ++j) {
        for (size_t i = 0; i < left.size(); ++i) {
            const double dx = static_cast<double>(left[i].x - right[j].x);
            const double dy = static_cast<double>(left[i].y - right[j].y);
            const double d2 = (dx * dx) + (dy * dy);
            if (d2 < nearest_left_d2[j]) {
                nearest_left_d2[j] = d2;
                nearest_left[j] = static_cast<int>(i);
            }
        }
    }

    // left[i] → 最近的 right 下标；只有互为最近邻才保留中点。
    for (size_t i = 0; i < left.size(); ++i) {
        int best_j = -1;
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t j = 0; j < right.size(); ++j) {
            const double dx = static_cast<double>(left[i].x - right[j].x);
            const double dy = static_cast<double>(left[i].y - right[j].y);
            const double d2 = (dx * dx) + (dy * dy);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_j = static_cast<int>(j);
            }
        }
        if (best_j < 0 || best_d2 > max_dist_sq) {
            continue;
        }
        if (nearest_left[static_cast<size_t>(best_j)] != static_cast<int>(i)) {
            continue;  // 单向配对 → 丢弃
        }
        out.emplace_back((left[i].x + right[static_cast<size_t>(best_j)].x) * 0.5,
                         (left[i].y + right[static_cast<size_t>(best_j)].y) * 0.5);
    }
    return out;
}

// #95：走廊中线点的**单调排序**。
//
// 旧实现是「从原点最近邻贪心串接」：每步无条件取最近的未用点。在八字切点附近
// 走廊两瓣的几何在 x 轴上重叠，贪心会把中线点来回串（实测每次生成出现 2~4 次
// x 回跳，如 2.88 → 15.43 → 2.90），Pure Pursuit 的窗口化最近点搜索因此锁到
// 远处点 → 报「Approaching path end (40/41), braking」。
//
// 改为**方向感知行走**：从离自车最近的点出发，每步只在「当前前进方向」上取下一
// 个点（带距离惩罚），保证沿走廊单向推进、点序单调连续。
inline std::vector<Point2D> OrderAlongCorridorCore(std::span<const Point2D> points, Point2D ego, double max_step) {
    std::vector<Point2D> ordered;
    const size_t n = points.size();
    if (n == 0) {
        return ordered;
    }
    if (!(max_step > 0.0)) {
        return ordered;
    }

    std::vector<bool> used(n, false);
    size_t cur = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        const double dx = points[i].x - ego.x;
        const double dy = points[i].y - ego.y;
        const double d2 = (dx * dx) + (dy * dy);
        if (d2 < best_d2) {
            best_d2 = d2;
            cur = i;
        }
    }

    // base_link 系的航向是 +x（自车在原点）。
    double dir_x = 1.0;
    double dir_y = 0.0;
    constexpr double kDistancePenalty = 0.45;

    while (true) {
        used[cur] = true;
        ordered.push_back(points[cur]);
        const Point2D& c = points[cur];

        int best = -1;
        double best_score = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < n; ++i) {
            if (used[i]) {
                continue;
            }
            const double dx = points[i].x - c.x;
            const double dy = points[i].y - c.y;
            const double d = std::hypot(dx, dy);
            if (d <= 1e-9 || d > max_step) {
                continue;
            }
            const double align = ((dx * dir_x) + (dy * dir_y)) / d;
            const double score = align - (kDistancePenalty * (d / max_step));
            if (score > best_score) {
                best_score = score;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) {
            break;
        }
        const double dx = points[best].x - c.x;
        const double dy = points[best].y - c.y;
        const double d = std::hypot(dx, dy);
        dir_x = dx / d;
        dir_y = dy / d;
        cur = static_cast<size_t>(best);
    }
    return ordered;
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

    // #95：本函数的**排序语义保持 ROS1 遗留行为不变**——它是 #24 跨版本语义差分
    // 夹具（test_icp_apf_planner.cpp 的 DifferentialEquivalenceWithLegacyPCL，
    // 1000 组固定种子随机用例逐点比对）的事实来源，改它等于改差分基线。
    // 点序单调由上层 ComputeCorridorPathCore 用 OrderAlongCorridorCore 重排实现
    // （不丢点：此处输出是输入中点集合的一个排列）。
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

    // #95：拟合残差（RMS）—— 圆弧上的点残差≈0；共线/退化输入的代数拟合残差很大。
    double sum_sq = 0.0;
    for (const auto& c : cones) {
        const double d = std::hypot(c.x - result.cx, c.y - result.cy);
        const double e = d - result.r;
        sum_sq += e * e;
    }
    result.rms_residual = std::sqrt(sum_sq / static_cast<double>(n));

    result.valid = true;
    return result;
}

// #95：候选圆是否「自车在其上」。
//
// 走廊中线是车辆实际行驶的那条线 → 自车必在其上。对一个拟合出的**壁圆**
// （半径 = 中线半径 ± 走廊半宽）来说，判据是 ‖ego − C‖ ≈ r_wall。
// 这条判据同时否掉两类历史缺陷：
//   1) 左右两圆被平均成单圆：平均圆的圆心落在自车处（‖ego − C‖ ≈ 0）而半径
//      仍是 ~9 m → |0 − 9.125| 远超容差 → 直接被否；
//   2) 拟合到另一个瓣的圆：自车不在那条圆上 → 同样被否。
inline bool CircleHoldsEgo(const Circle2D& c, Point2D ego, double tol, double max_rms = 0.5) {
    if (!c.valid) {
        return false;
    }
    // #95：拟合残差过大 = 拟合出来的根本不是圆（例如两条平行直线的代数拟合），
    // 此时「自车在圆上」的相位判据没有意义，直接否掉。
    if (!(c.rms_residual <= max_rms)) {
        return false;
    }
    if (!(tol >= 0.0) || !std::isfinite(tol)) {
        return false;
    }
    const double d = std::hypot(ego.x - c.cx, ego.y - c.cy);
    return std::abs(d - c.r) <= tol;
}

inline std::vector<Point2D> ComputeCircleCenterlineCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                        const CenterlineParams& p, Point2D ego = Point2D{0.0, 0.0}) {
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

    // ── #95：左右两个圆**不得**被平均成一个圆 ──────────────────────────────────
    // 八字是两个圆，不是一个圆。左右两组锥桶在切点附近分别属于**不同的瓣**
    // （左瓣圆心 cy≈−9.125、右瓣圆心 cy≈+9.125），旧实现直接取平均 → 圆心的
    // cy 被抵消到 0，生成一个以自车为圆心的虚假同心圆，路径整体离车身 ~9 m，
    // 触发 Pure Pursuit「Crosstrack 10.03m exceeds limit 6.00m, refusing goal」。
    //
    // 判定规则（几何上可证）：
    //   a) 左右拟合圆**同心**（圆心距 ≤ concentric_tol）→ 它们是同一条走廊的
    //      内外两壁 → 半径取平均是几何正确的 → 保留（历史行为不变）。
    //   b) 左右拟合圆**不同心** → 它们属于不同的瓣 → **绝不允许平均**；
    //      改用相位感知选择：候选圆必须「自车在其上」。命中者，中线圆心取该圆
    //      圆心、中线**半径取 ‖ego − C|**（自车直接给出中线半径，无需知道这
    //      是内壁还是外壁），弧线**从自车起步**；两个候选都不命中 → 返回空，
    //      交回退路径（中点法）。
    Circle2D center_circle;
    bool start_from_ego = false;
    if (lc.valid && rc.valid) {
        const double center_sep = std::hypot(lc.cx - rc.cx, lc.cy - rc.cy);
        // #95：拟合质量护栏 —— 残差过大说明拟合出来的根本不是圆（例如两条平行
        // 直线的代数拟合），此时既不能平均也不能选，直接交回退路径。
        const bool fit_ok = (lc.rms_residual <= p.max_fit_rms) && (rc.rms_residual <= p.max_fit_rms);
        if (fit_ok && center_sep <= p.concentric_tol) {
            center_circle.cx = (lc.cx + rc.cx) * 0.5;
            center_circle.cy = (lc.cy + rc.cy) * 0.5;
            center_circle.r = (lc.r + rc.r) * 0.5;
            center_circle.valid = true;
        } else {
            const Circle2D* candidate = nullptr;
            if (CircleHoldsEgo(lc, ego, p.phase_tol, p.max_fit_rms)) {
                candidate = &lc;
            } else if (CircleHoldsEgo(rc, ego, p.phase_tol, p.max_fit_rms)) {
                candidate = &rc;
            }
            if (candidate == nullptr) {
                // 没有「自车在其上」的候选圆 → 交回退路径（绝不用平均圆兜底）。
                return {};
            }
            center_circle.cx = candidate->cx;
            center_circle.cy = candidate->cy;
            center_circle.r = std::hypot(ego.x - candidate->cx, ego.y - candidate->cy);
            center_circle.valid = true;
            start_from_ego = true;
        }
    } else if (lc.valid) {
        if (CircleHoldsEgo(lc, ego, p.phase_tol, p.max_fit_rms)) {
            center_circle.cx = lc.cx;
            center_circle.cy = lc.cy;
            center_circle.r = std::hypot(ego.x - lc.cx, ego.y - lc.cy);
            center_circle.valid = true;
            start_from_ego = true;
        } else {
            center_circle.cx = lc.cx;
            center_circle.cy = lc.cy;
            center_circle.r = std::max(lc.r - kDefaultHalfTrackWidth, lc.r * 0.5);
            center_circle.valid = true;
        }
    } else if (rc.valid) {
        if (CircleHoldsEgo(rc, ego, p.phase_tol, p.max_fit_rms)) {
            center_circle.cx = rc.cx;
            center_circle.cy = rc.cy;
            center_circle.r = std::hypot(ego.x - rc.cx, ego.y - rc.cy);
            center_circle.valid = true;
            start_from_ego = true;
        } else {
            center_circle.cx = rc.cx;
            center_circle.cy = rc.cy;
            center_circle.r = std::max(rc.r - kDefaultHalfTrackWidth, rc.r * 0.5);
            center_circle.valid = true;
        }
    }

    if (!center_circle.valid || center_circle.r < kMinRadius || center_circle.r > kMaxRadius) {
        return {};
    }

    // 起点角度：相位感知命中时**从自车起步**（路径必须含自车，否则 Pure Pursuit
    // 的横偏判据会拿远处点比 → refusing goal）；否则取圆上离自车最近的点。
    const double start_angle = start_from_ego ? std::atan2(ego.y - center_circle.cy, ego.x - center_circle.cx)
                                              : std::atan2(-center_circle.cy, -center_circle.cx);
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

// #95：把路径**锚到自车**并等距重采样。
//
// Pure Pursuit 用 `findNearestIndex` 在整条路径上找最近点，横偏判据拿该点与
// 车身原点比（max_crosstrack_m = 6.0 m）。路径若整体离车身 ~9 m（左右圆被平均
// 成单圆的直接后果），就会报「Crosstrack 10.03m exceeds limit 6.00m, refusing
// goal」并硬刹车 —— 车根本没起步，闭环无从谈起。
//
// 本函数把自车放到折线首点，再沿折线按 path_spacing 等距重采样，于是：
//   - 路径含自车 → 横偏 ≈ 0；
//   - 相邻点间距恒为 path_spacing（不超 1.5× 的间距判据）；
//   - 点序沿走廊单调（由 OrderAlongCorridorCore 保证）。
inline std::vector<Point2D> AnchorPathAtEgoCore(std::span<const Point2D> path, Point2D ego, const CenterlineParams& p) {
    if (path.empty() || !(p.path_spacing > 0.0)) {
        return {};
    }
    std::vector<Point2D> poly;
    poly.reserve(path.size() + 1);
    poly.emplace_back(ego);
    for (const auto& pt : path) {
        if (std::hypot(pt.x - ego.x, pt.y - ego.y) > 1e-6) {
            poly.push_back(pt);
        }
    }
    if (poly.size() < 2) {
        return {};
    }

    std::vector<double> cum(poly.size(), 0.0);
    for (size_t i = 1; i < poly.size(); ++i) {
        cum[i] = cum[i - 1] + std::hypot(poly[i].x - poly[i - 1].x, poly[i].y - poly[i - 1].y);
    }
    const double total = cum.back();
    if (!(total > 0.0)) {
        return {};
    }

    std::vector<Point2D> out;
    out.reserve(static_cast<size_t>(total / p.path_spacing) + 2);
    size_t seg = 0;
    for (double s = 0.0; s <= total + 1e-9; s += p.path_spacing) {
        while (seg + 1 < poly.size() && cum[seg + 1] < s) {
            ++seg;
        }
        if (seg + 1 >= poly.size()) {
            break;
        }
        const double seg_len = cum[seg + 1] - cum[seg];
        const double t = (seg_len > 1e-12) ? ((s - cum[seg]) / seg_len) : 0.0;
        out.emplace_back(poly[seg].x + (t * (poly[seg + 1].x - poly[seg].x)),
                         poly[seg].y + (t * (poly[seg + 1].y - poly[seg].y)));
    }
    return out;
}

// #95：用**上一帧路径**补齐前视长度。
//
// 传感器在紧弯（八字瓣顶 R=9.125 m）上只能看到 ~5 m 走廊；此时中点法的可见
// 中线点很少（实测切点附近只剩 10~12 点），Pure Pursuit 的
// `end_decel_points = 10` 判据就会触发 →「Approaching path end (2/12), braking」
// → 车被刹停，闭环推进中断。
//
// 上一帧路径（已随自车推进换算到当前 base_link）里**自车前方**的那段，正是
// 上一帧看到的、更靠前的走廊 —— 它把前视长度接上（时间维度的累积）。
// 只取「比新路径末端更远」的点，按到自车的距离升序拼接，因此点序仍单调。
inline std::vector<Point2D> ExtendWithPrevPathCore(std::span<const Point2D> path, std::span<const Point2D> prev_path,
                                                   Point2D ego, const CenterlineParams& p) {
    if (path.empty() || prev_path.empty() || !(p.path_lookahead > 0.0)) {
        return std::vector<Point2D>(path.begin(), path.end());
    }
    double reach = 0.0;
    for (const auto& pt : path) {
        reach = std::max(reach, std::hypot(pt.x - ego.x, pt.y - ego.y));
    }

    std::vector<std::pair<double, Point2D>> extra;
    for (const auto& pt : prev_path) {
        const double d = std::hypot(pt.x - ego.x, pt.y - ego.y);
        if (d > reach + 1e-6 && d <= p.path_lookahead) {
            extra.emplace_back(d, pt);
        }
    }
    if (extra.empty()) {
        return std::vector<Point2D>(path.begin(), path.end());
    }
    std::ranges::sort(extra, [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<Point2D> out(path.begin(), path.end());
    for (const auto& e : extra) {
        out.push_back(e.second);
    }
    return out;
}

// #95：走廊中线路径的产出结果（供节点层与单测读取判据来源）。
struct CorridorPathResult {
    std::vector<Point2D> path;  // base_link 系，含自车、点序单调、等间距
    bool used_circle{false};    // true = 走圆法（相位感知单圆），false = 中点法
    Circle2D midline;           // 选中的中线圆（base_link 系）；未选中时 valid=false
};

// #95：走廊中线的**生产入口**（节点层只调这一个）。
//
// 顺序：相位感知圆法 →（圆法不成立时）中点法 → 锚到自车并等距重采样。
// 圆法不成立 = 没有「自车在其上」的候选圆（含左右落在不同瓣却都不可信的情况），
// 此时**不再用左右平均圆兜底**（那正是 #92 实测的虚假同心圆），而是交中点法。
inline CorridorPathResult ComputeCorridorPathCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                  const CenterlineParams& p, Point2D ego = Point2D{0.0, 0.0},
                                                  std::span<const Point2D> prev_path = {}) {
    CorridorPathResult out;
    std::vector<Point2D> path = ComputeCircleCenterlineCore(left, right, p, ego);
    if (!path.empty()) {
        out.used_circle = true;
        out.path = std::move(path);
        return out;
    }
    path = ComputeCenterlineCore(left, right, p);
    if (path.empty()) {
        return out;
    }
    // #95：点序单调连续 —— 先用「互为最近邻」过滤掉单向配对产生的伪中点，
    // 再做方向感知重排（不丢点：遗留输出是中点集合的一个排列），
    // 最后锚到自车并等距重采样。
    std::vector<Point2D> mutual = AppendMutualMidpointsCore(left, right, p.max_pair_dist);
    if (!mutual.empty()) {
        path = std::move(mutual);
    }
    std::vector<Point2D> ordered = OrderAlongCorridorCore(path, ego, p.max_pair_dist);
    if (ordered.empty()) {
        return out;
    }
    out.path = AnchorPathAtEgoCore(ordered, ego, p);
    // #95：用上一帧路径补齐前视长度（紧弯上传感器只看得到 ~5 m 走廊）。
    out.path = ExtendWithPrevPathCore(out.path, prev_path, ego, p);
    return out;
}

}  // namespace skidpad
