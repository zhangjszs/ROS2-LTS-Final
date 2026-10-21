#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <ranges>
#include <span>
#include <vector>

namespace slp_core {

struct Point2D {
    double x{0.0};
    double y{0.0};
    auto operator<=>(const Point2D&) const = default;
};

struct LineParams {
    double slope{0.0};      // dy/dx
    double intercept{0.0};  // x=0 处的 y 值
    bool valid{false};

    constexpr LineParams() = default;
    constexpr LineParams(double m, double b) : slope(m), intercept(b), valid(true) {}
    constexpr double YAt(double x) const { return slope * x + intercept; }
    auto operator<=>(const LineParams&) const = default;
};

struct DetectedBoundaries {
    LineParams left;
    LineParams right;
    bool success{false};
    auto operator<=>(const DetectedBoundaries&) const = default;
};

struct LineDetectorConfig {
    // 锥桶聚类
    double center_margin = 0.3;

    // RANSAC
    double ransac_inlier_threshold = 0.3;
    int ransac_max_iter = 200;
    double ransac_min_x_spread = 1.0;
    double ransac_max_abs_slope = 0.3;
    int ransac_min_inliers = 3;
    double ransac_min_inlier_ratio = 0.5;

    // Hough
    bool enable_hough = false;
    double hough_rho_resolution = 0.30;
    double hough_min_inlier_ratio = 0.5;

    // 时序滤波
    bool enable_temporal_filter = true;
    double temporal_filter_alpha = 0.3;
    double temporal_filter_jump_threshold = 0.4;
    double temporal_filter_intercept_jump = 4.0;
    int prev_max_age_frames = 10;
};

// 坐标鸭子类型（duck-typing）：同时支持 Point2D（.x/.y）与 ROS HuatCone（.position_base_link.x/.y）
template <typename T>
concept Position2DLike = (requires(const T& p) {
                             { p.x } -> std::convertible_to<double>;
                             { p.y } -> std::convertible_to<double>;
                         }) || (requires(const T& p) {
                             { p.position_base_link.x } -> std::convertible_to<double>;
                             { p.position_base_link.y } -> std::convertible_to<double>;
                         });

template <Position2DLike P>
constexpr double GetX(const P& p) {
    if constexpr (requires { p.x; }) {
        return static_cast<double>(p.x);
    } else {
        return static_cast<double>(p.position_base_link.x);
    }
}

template <Position2DLike P>
constexpr double GetY(const P& p) {
    if constexpr (requires { p.y; }) {
        return static_cast<double>(p.y);
    } else {
        return static_cast<double>(p.position_base_link.y);
    }
}

template <Position2DLike P>
inline double PointToLineDistanceCore(const P& p, double m, double b) {
    const double x = GetX(p);
    const double y = GetY(p);
    return std::abs(m * x - y + b) / std::sqrt(m * m + 1.0);
}

template <Position2DLike P>
inline bool IsLineGoodCore(const LineParams& line, std::span<const P> cones, double thresh,
                           const LineDetectorConfig& cfg) {
    if (!line.valid || cones.empty())
        return false;
    auto inliers = std::ranges::count_if(
        cones, [thresh, &line](const P& c) { return PointToLineDistanceCore(c, line.slope, line.intercept) < thresh; });
    if (inliers < cfg.ransac_min_inliers)
        return false;
    double ratio = static_cast<double>(inliers) / static_cast<double>(cones.size());
    return ratio >= cfg.ransac_min_inlier_ratio;
}

template <Position2DLike P>
inline LineParams LeastSquaresFitCore(std::span<const P> cones, const LineDetectorConfig& cfg) {
    if (cones.size() < 2)
        return LineParams();

    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    const size_t n = cones.size();
    for (const auto& c : cones) {
        double x = GetX(c);
        double y = GetY(c);
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double denom = static_cast<double>(n) * sum_x2 - sum_x * sum_x;
    if (std::abs(denom) < 1e-9)
        return LineParams();

    double m = (static_cast<double>(n) * sum_xy - sum_x * sum_y) / denom;
    double b = (sum_y * sum_x2 - sum_x * sum_xy) / denom;
    if (!std::isfinite(m) || !std::isfinite(b) || std::abs(m) > cfg.ransac_max_abs_slope)
        return LineParams();
    return LineParams(m, b);
}

template <Position2DLike P>
inline LineParams HoughFitCore(std::span<const P> cones, const LineDetectorConfig& cfg) {
    if (cones.size() < 2)
        return LineParams();

    // 参数守卫 1（#62/#78）：hough_min_inlier_ratio 必须是 [0, 1] 内的有限数，否则返回空 LineParams。
    // #62 已覆盖 NaN / +inf / -inf / 负值；#78 补上**超大有限值**（如 1e300）——
    // 那类值会让下方的 static_cast<int>(ceil(ratio * cones.size())) 触发 float→int 溢出 UB
    // （UBSan: 1e+301 is outside the range of representable values of type 'int'）。
    // ratio > 1 判为非法与旧行为等价：此时自适应阈值大于总票数，本就返回空 LineParams。
    if (!std::isfinite(cfg.hough_min_inlier_ratio) || cfg.hough_min_inlier_ratio < 0.0 ||
        cfg.hough_min_inlier_ratio > 1.0) {
        return LineParams();
    }

    const double kThetaMinDeg = 60.0;
    const double kThetaMaxDeg = 120.0;
    const double kThetaStepDeg = 1.0;
    const double rho_resolution = std::max(0.05, cfg.hough_rho_resolution);
    const double kRhoMin = -10.0;
    const double kRhoMax = 10.0;

    // 参数守卫 2（#62）：hough_rho_resolution 若为 +inf 或过大导致桶数 <= 2 时，明确返回空 LineParams
    if (!std::isfinite(rho_resolution) || rho_resolution > (kRhoMax - kRhoMin)) {
        return LineParams();
    }

    int theta_bins = static_cast<int>((kThetaMaxDeg - kThetaMinDeg) / kThetaStepDeg) + 1;
    int rho_bins = static_cast<int>((kRhoMax - kRhoMin) / rho_resolution);
    if (rho_bins <= 2) {
        return LineParams();
    }

    std::vector<std::vector<int>> accumulator(theta_bins, std::vector<int>(rho_bins, 0));

    for (const auto& c : cones) {
        double x = GetX(c);
        double y = GetY(c);
        for (int t = 0; t < theta_bins; ++t) {
            double theta_deg = kThetaMinDeg + t * kThetaStepDeg;
            double theta_rad = theta_deg * std::numbers::pi_v<double> / 180.0;
            double rho = x * std::cos(theta_rad) + y * std::sin(theta_rad);
            int r = static_cast<int>((rho - kRhoMin) / rho_resolution);
            if (r >= 0 && r < rho_bins) {
                ++accumulator[t][r];
            }
        }
    }

    // 3x1 沿 rho 邻域投票累加
    struct HoughVoteCandidate {
        int t;
        int r;
        int votes;
    };
    std::vector<HoughVoteCandidate> candidates;
    candidates.reserve(static_cast<size_t>(theta_bins) * static_cast<size_t>(std::max(0, rho_bins - 2)));
    for (int t = 0; t < theta_bins; ++t) {
        for (int r = 1; r < rho_bins - 1; ++r) {
            int s = accumulator[t][r - 1] + accumulator[t][r] + accumulator[t][r + 1];
            candidates.push_back({t, r, s});
        }
    }

    if (candidates.empty()) {
        return LineParams();
    }

    auto best_cand = std::ranges::max_element(candidates, {}, &HoughVoteCandidate::votes);
    int best_t = best_cand->t;
    int best_r = best_cand->r;
    int best_votes = best_cand->votes;

    int dynamic_min = static_cast<int>(std::ceil(cfg.hough_min_inlier_ratio * static_cast<double>(cones.size())));
    int adaptive_thresh = std::max(3, dynamic_min);

    if (best_votes < adaptive_thresh || best_t < 0) {
        return LineParams();
    }

    double theta_deg = kThetaMinDeg + best_t * kThetaStepDeg;
    double theta_rad = theta_deg * std::numbers::pi_v<double> / 180.0;
    double rho = kRhoMin + best_r * rho_resolution;

    double sin_t = std::sin(theta_rad);
    double cos_t = std::cos(theta_rad);
    if (std::abs(sin_t) < 1e-6)
        return LineParams();

    double slope = -cos_t / sin_t;
    double intercept = rho / sin_t;
    if (std::abs(slope) > cfg.ransac_max_abs_slope) {
        return LineParams();
    }
    return LineParams(slope, intercept);
}

template <Position2DLike P>
inline LineParams RansacFitCore(std::span<const P> cones, const LineDetectorConfig& cfg,
                                std::optional<std::uint64_t> seed = std::nullopt) {
    if (cones.size() < 2 || cfg.ransac_max_iter <= 0)
        return LineParams();

    std::mt19937 local_gen;
    std::mt19937* p_gen = nullptr;
    if (seed.has_value()) {
        local_gen.seed(static_cast<std::mt19937::result_type>(*seed));
        p_gen = &local_gen;
    } else {
        static thread_local std::mt19937 static_gen{std::random_device{}()};
        p_gen = &static_gen;
    }

    std::uniform_int_distribution<size_t> dist(0, cones.size() - 1);

    LineParams best;
    size_t best_inliers = 0;

    for (int iter = 0; iter < cfg.ransac_max_iter; ++iter) {
        size_t i1 = dist(*p_gen);
        size_t i2 = dist(*p_gen);
        if (i1 == i2)
            continue;

        const auto& p1 = cones[i1];
        const auto& p2 = cones[i2];
        double dx = GetX(p2) - GetX(p1);
        if (std::abs(dx) < cfg.ransac_min_x_spread || std::abs(dx) < 1e-9)
            continue;

        double m = (GetY(p2) - GetY(p1)) / dx;
        if (!std::isfinite(m) || std::abs(m) > cfg.ransac_max_abs_slope)
            continue;
        double b = GetY(p1) - m * GetX(p1);

        size_t inliers = static_cast<size_t>(std::ranges::count_if(cones, [&cfg, m, b](const P& c) {
            return PointToLineDistanceCore(c, m, b) < cfg.ransac_inlier_threshold;
        }));
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best = LineParams(m, b);
        }
    }

    if (!best.valid)
        return LineParams();
    if (static_cast<int>(best_inliers) < cfg.ransac_min_inliers)
        return LineParams();

    // 用最佳内点集做最小二乘精化
    std::vector<P> inlier_cones;
    inlier_cones.reserve(best_inliers);
    std::ranges::copy_if(cones, std::back_inserter(inlier_cones), [&cfg, &best](const P& c) {
        return PointToLineDistanceCore(c, best.slope, best.intercept) < cfg.ransac_inlier_threshold;
    });

    std::ranges::sort(inlier_cones, {}, [](const P& c) { return GetX(c); });

    if (static_cast<int>(inlier_cones.size()) >= cfg.ransac_min_inliers) {
        LineParams refined = LeastSquaresFitCore(std::span<const P>{inlier_cones}, cfg);
        if (refined.valid)
            return refined;
    }
    return best;
}

inline LineParams ApplyTemporalFilterCore(const LineParams& current, LineParams& prev, bool& has_prev, int& prev_age,
                                          const LineDetectorConfig& cfg) {
    if (!cfg.enable_temporal_filter) {
        if (current.valid) {
            prev = current;
            has_prev = true;
            prev_age = 0;
        }
        return current;
    }

    if (!current.valid) {
        // 在放弃前继续使用之前的拟合结果几帧
        if (has_prev && prev_age < cfg.prev_max_age_frames) {
            ++prev_age;
            return prev;
        }
        has_prev = false;
        prev_age = 0;
        return current;
    }

    if (!has_prev) {
        prev = current;
        has_prev = true;
        prev_age = 0;
        return current;
    }

    double dslope = std::abs(current.slope - prev.slope);
    double dintercept = std::abs(current.intercept - prev.intercept);
    if (dslope > cfg.temporal_filter_jump_threshold || dintercept > cfg.temporal_filter_intercept_jump) {
        if (prev_age < cfg.prev_max_age_frames) {
            ++prev_age;
            return prev;
        }
        // 连续拒绝次数过多：直接接受新值，避免陷入停滞
        prev = current;
        prev_age = 0;
        return current;
    }

    double a = cfg.temporal_filter_alpha;
    LineParams filtered;
    filtered.slope = a * current.slope + (1.0 - a) * prev.slope;
    filtered.intercept = a * current.intercept + (1.0 - a) * prev.intercept;
    filtered.valid = true;
    prev = filtered;
    prev_age = 0;
    return filtered;
}

}  // namespace slp_core
