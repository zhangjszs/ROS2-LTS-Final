#pragma once

#include <common_msgs/msg/huat_cone.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

#include "line_fit_core.hpp"

using slp_core::DetectedBoundaries;
using slp_core::LineDetectorConfig;
using slp_core::LineParams;

class LineDetector {
   public:
    explicit LineDetector(const LineDetectorConfig& cfg);

    DetectedBoundaries Detect(const std::vector<common_msgs::msg::HuatCone>& cones);

    void ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                      std::vector<common_msgs::msg::HuatCone>& left, std::vector<common_msgs::msg::HuatCone>& right);

    LineParams HoughFit(const std::vector<common_msgs::msg::HuatCone>& cones);
    LineParams RansacFit(const std::vector<common_msgs::msg::HuatCone>& cones);

   private:
    LineParams LeastSquaresFit(const std::vector<common_msgs::msg::HuatCone>& cones);

    bool IsLineGood(const LineParams& line, const std::vector<common_msgs::msg::HuatCone>& cones, double thresh);
    static double PointToLineDistance(const geometry_msgs::msg::Point32& p, double m, double b);

    LineParams ApplyTemporalFilter(const LineParams& current, LineParams& prev, bool& has_prev, int& prev_age,
                                   const char* side_name);

    LineDetectorConfig cfg_;

    // 每侧的时序状态
    LineParams prev_left_, prev_right_;
    bool has_prev_left_ = false, has_prev_right_ = false;
    int prev_left_age_ = 0, prev_right_age_ = 0;
};
