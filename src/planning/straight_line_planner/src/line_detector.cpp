#include "line_detector.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <ranges>
#include <span>

// #22：锥桶左右分离谓词收敛到纯 std core（无 ROS context）
#include "cone_boundary.h"

LineDetector::LineDetector(const LineDetectorConfig& cfg) : cfg_(cfg) {}

void LineDetector::ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                                std::vector<common_msgs::msg::HuatCone>& left,
                                std::vector<common_msgs::msg::HuatCone>& right) {
    left.clear();
    right.clear();
    // #22：分离谓词收敛到 slp_core::SplitConesBySide（纯 std），此处仅做索引→拷贝（保序，与 copy_if 等价）。
    const slp_core::ConeSideSplit split =
        slp_core::SplitConesBySide(std::span<const common_msgs::msg::HuatCone>{cones}, cfg_.center_margin);
    left.reserve(split.left.size());
    right.reserve(split.right.size());
    for (const auto i : split.left) {
        left.push_back(cones[i]);
    }
    for (const auto i : split.right) {
        right.push_back(cones[i]);
    }
}

LineParams LineDetector::HoughFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    return slp_core::HoughFitCore(std::span<const common_msgs::msg::HuatCone>{cones}, cfg_);
}

bool LineDetector::IsLineGood(const LineParams& line, const std::vector<common_msgs::msg::HuatCone>& cones,
                              double thresh) {
    return slp_core::IsLineGoodCore(line, std::span<const common_msgs::msg::HuatCone>{cones}, thresh, cfg_);
}

LineParams LineDetector::RansacFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    return slp_core::RansacFitCore(std::span<const common_msgs::msg::HuatCone>{cones}, cfg_);
}

LineParams LineDetector::LeastSquaresFit(const std::vector<common_msgs::msg::HuatCone>& cones) {
    return slp_core::LeastSquaresFitCore(std::span<const common_msgs::msg::HuatCone>{cones}, cfg_);
}

double LineDetector::PointToLineDistance(const geometry_msgs::msg::Point32& p, double m, double b) {
    return slp_core::PointToLineDistanceCore(p, m, b);
}

LineParams LineDetector::ApplyTemporalFilter(const LineParams& current, LineParams& prev, bool& has_prev, int& prev_age,
                                             const char* /*side_name*/) {
    return slp_core::ApplyTemporalFilterCore(current, prev, has_prev, prev_age, cfg_);
}

DetectedBoundaries LineDetector::Detect(const std::vector<common_msgs::msg::HuatCone>& cones) {
    DetectedBoundaries result;
    std::vector<common_msgs::msg::HuatCone> left_cones, right_cones;
    ClusterCones(cones, left_cones, right_cones);

    auto FitSide = [&](const std::vector<common_msgs::msg::HuatCone>& side_cones,
                       const char* /*side_name*/) -> LineParams {
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
