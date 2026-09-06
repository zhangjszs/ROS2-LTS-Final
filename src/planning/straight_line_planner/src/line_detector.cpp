#include "line_detector.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

LineDetector::LineDetector(const LineDetectorConfig& cfg) : cfg_(cfg) {}

void LineDetector::ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                                std::vector<common_msgs::msg::HuatCone>& left, std::vector<common_msgs::msg::HuatCone>& right) {
    left.clear();
    right.clear();
    for (const auto& c : cones) {
        double y = c.position_base_link.y;
        if (y < -cfg_.center_margin) {
            left.push_back(c);
        } else if (y > cfg_.center_margin) {
            right.push_back(c);
        }
    }
}

LineParams LineDetector::HoughFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    if (cones.size() < 2)
        return LineParams();

    const double kThetaMinDeg = 60.0;
    const double kThetaMaxDeg = 120.0;
    const double kThetaStepDeg = 1.0;
    const double rho_resolution = std::max(0.05, cfg_.hough_rho_resolution);
    const double kRhoMin = -10.0;
    const double kRhoMax = 10.0;

    int theta_bins = static_cast<int>((kThetaMaxDeg - kThetaMinDeg) / kThetaStepDeg) + 1;
    int rho_bins = static_cast<int>((kRhoMax - kRhoMin) / rho_resolution);

    std::vector<std::vector<int>> accumulator(theta_bins, std::vector<int>(rho_bins, 0));

    for (const auto& c : cones) {
        double x = c.position_base_link.x;
        double y = c.position_base_link.y;
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

    // 3x1 沿 rho 邻域投票累加 -> 容忍锥桶噪声
    int best_t = -1, best_r = -1;
    int best_votes = 0;
    for (int t = 0; t < theta_bins; ++t) {
        for (int r = 1; r < rho_bins - 1; ++r) {
            int s = accumulator[t][r - 1] + accumulator[t][r] + accumulator[t][r + 1];
            if (s > best_votes) {
                best_votes = s;
                best_t = t;
                best_r = r;
            }
        }
    }

    int dynamic_min = static_cast<int>(std::ceil(cfg_.hough_min_inlier_ratio * static_cast<double>(cones.size())));
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
    if (std::abs(slope) > cfg_.ransac_max_abs_slope) {
        return LineParams();
    }
    return LineParams(slope, intercept);
}

bool LineDetector::IsLineGood(const LineParams& line, const std::vector<common_msgs::msg::HuatCone>& cones, double thresh) {
    if (!line.valid || cones.empty())
        return false;
    int inliers = 0;
    for (const auto& c : cones) {
        if (PointToLineDistance(c.position_base_link, line.slope, line.intercept) < thresh) {
            ++inliers;
        }
    }
    if (inliers < cfg_.ransac_min_inliers)
        return false;
    double ratio = static_cast<double>(inliers) / static_cast<double>(cones.size());
    return ratio >= cfg_.ransac_min_inlier_ratio;
}

LineParams LineDetector::RansacFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    if (cones.size() < 2)
        return LineParams();

    static thread_local std::mt19937 gen{std::random_device{}()};
    std::uniform_int_distribution<size_t> dist(0, cones.size() - 1);

    LineParams best;
    size_t best_inliers = 0;

    for (int iter = 0; iter < cfg_.ransac_max_iter; ++iter) {
        size_t i1 = dist(gen);
        size_t i2 = dist(gen);
        if (i1 == i2)
            continue;

        const auto& p1 = cones[i1].position_base_link;
        const auto& p2 = cones[i2].position_base_link;
        double dx = p2.x - p1.x;
        if (std::abs(dx) < cfg_.ransac_min_x_spread)
            continue;

        double m = (p2.y - p1.y) / dx;
        if (std::abs(m) > cfg_.ransac_max_abs_slope)
            continue;
        double b = p1.y - m * p1.x;

        size_t inliers = 0;
        for (const auto& c : cones) {
            if (PointToLineDistance(c.position_base_link, m, b) < cfg_.ransac_inlier_threshold)
                ++inliers;
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best = LineParams(m, b);
        }
    }

    if (!best.valid)
        return LineParams();
    if (static_cast<int>(best_inliers) < cfg_.ransac_min_inliers)
        return LineParams();

    // 用最佳内点集做最小二乘精化，比直接返回两点采样直线更准
    std::vector<common_msgs::msg::HuatCone> inlier_cones;
    inlier_cones.reserve(best_inliers);
    for (const auto& c : cones) {
        if (PointToLineDistance(c.position_base_link, best.slope, best.intercept) < cfg_.ransac_inlier_threshold) {
            inlier_cones.push_back(c);
        }
    }
    if (static_cast<int>(inlier_cones.size()) >= cfg_.ransac_min_inliers) {
        LineParams refined = LeastSquaresFit(inlier_cones);
        if (refined.valid)
            return refined;
    }
    return best;
}

LineParams LineDetector::LeastSquaresFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    if (cones.size() < 2)
        return LineParams();

    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    size_t n = cones.size();
    for (const auto& c : cones) {
        double x = c.position_base_link.x;
        double y = c.position_base_link.y;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double denom = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denom) < 1e-9)
        return LineParams();

    double m = (n * sum_xy - sum_x * sum_y) / denom;
    double b = (sum_y * sum_x2 - sum_x * sum_xy) / denom;
    if (std::abs(m) > cfg_.ransac_max_abs_slope)
        return LineParams();
    return LineParams(m, b);
}

double LineDetector::PointToLineDistance(const geometry_msgs::msg::Point32& p, double m, double b) {
    return std::abs(m * p.x - p.y + b) / std::sqrt(m * m + 1.0);
}

LineParams LineDetector::ApplyTemporalFilter(const LineParams& current, LineParams& prev, bool& has_prev, int& prev_age,
                                             const char* /*side_name*/) {
    if (!cfg_.enable_temporal_filter) {
        if (current.valid) {
            prev = current;
            has_prev = true;
            prev_age = 0;
        }
        return current;
    }

    if (!current.valid) {
        // 在放弃前继续使用之前的拟合结果几帧
        if (has_prev && prev_age < cfg_.prev_max_age_frames) {
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
    if (dslope > cfg_.temporal_filter_jump_threshold || dintercept > cfg_.temporal_filter_intercept_jump) {
        if (prev_age < cfg_.prev_max_age_frames) {
            ++prev_age;
            return prev;
        }
        // 连续拒绝次数过多：直接接受新值，避免陷入停滞
        prev = current;
        prev_age = 0;
        return current;
    }

    double a = cfg_.temporal_filter_alpha;
    LineParams filtered;
    filtered.slope = a * current.slope + (1.0 - a) * prev.slope;
    filtered.intercept = a * current.intercept + (1.0 - a) * prev.intercept;
    filtered.valid = true;
    prev = filtered;
    prev_age = 0;
    return filtered;
}

DetectedBoundaries LineDetector::Detect(const std::vector<common_msgs::msg::HuatCone>& cones) {
    DetectedBoundaries result;
    std::vector<common_msgs::msg::HuatCone> left_cones, right_cones;
    ClusterCones(cones, left_cones, right_cones);

    auto FitSide = [&](const std::vector<common_msgs::msg::HuatCone>& side_cones, const char* /*side_name*/) -> LineParams {
        if (side_cones.size() < 2) {
            return LineParams();
        }

        if (cfg_.enable_hough) {
            LineParams hough = HoughFit(side_cones);
            bool hough_good = IsLineGood(hough, side_cones, cfg_.ransac_inlier_threshold);
            if (hough_good)
                return hough;
        }

        LineParams ransac = RansacFit(side_cones);
        bool ransac_good = IsLineGood(ransac, side_cones, cfg_.ransac_inlier_threshold);
        if (ransac_good)
            return ransac;

        LineParams ls = LeastSquaresFit(side_cones);
        bool ls_good = IsLineGood(ls, side_cones, cfg_.ransac_inlier_threshold * 1.5);
        if (ls_good)
            return ls;

        return LineParams();
    };

    LineParams raw_left = FitSide(left_cones, "Left");
    LineParams raw_right = FitSide(right_cones, "Right");

    result.left = ApplyTemporalFilter(raw_left, prev_left_, has_prev_left_, prev_left_age_, "Left");
    result.right = ApplyTemporalFilter(raw_right, prev_right_, has_prev_right_, prev_right_age_, "Right");
    result.success = result.left.valid && result.right.valid;

    return result;
}
