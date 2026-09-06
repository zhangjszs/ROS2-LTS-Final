#pragma once

#include <common_msgs/msg/huat_cone.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>

struct LineParams {
    double slope;      // dy/dx
    double intercept;  // x=0 处的 y 值
    bool valid;
    LineParams() : slope(0.0), intercept(0.0), valid(false) {}
    LineParams(double m, double b) : slope(m), intercept(b), valid(true) {}
    double YAt(double x) const {
        return slope * x + intercept;
    }
};

struct DetectedBoundaries {
    LineParams left;
    LineParams right;
    bool success;
    DetectedBoundaries() : success(false) {}
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
    int hough_threshold = 10;
    double hough_rho_resolution = 0.30;
    double hough_min_inlier_ratio = 0.5;

    // 时序滤波
    bool enable_temporal_filter = true;
    double temporal_filter_alpha = 0.3;
    double temporal_filter_jump_threshold = 0.4;
    double temporal_filter_intercept_jump = 4.0;
    int prev_max_age_frames = 10;
};

class LineDetector {
   public:
    explicit LineDetector(const LineDetectorConfig& cfg);

    DetectedBoundaries Detect(const std::vector<common_msgs::msg::HuatCone>& cones);

   private:
    void ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones, std::vector<common_msgs::msg::HuatCone>& left,
                      std::vector<common_msgs::msg::HuatCone>& right);

    LineParams HoughFit(const std::vector<common_msgs::msg::HuatCone>& cones);
    LineParams RansacFit(const std::vector<common_msgs::msg::HuatCone>& cones);
    LineParams LeastSquaresFit(const std::vector<common_msgs::msg::HuatCone>& cones);

    bool IsLineGood(const LineParams& line, const std::vector<common_msgs::msg::HuatCone>& cones, double thresh);
    double PointToLineDistance(const geometry_msgs::msg::Point32& p, double m, double b);

    LineParams ApplyTemporalFilter(const LineParams& current, LineParams& prev, bool& has_prev, int& prev_age,
                                   const char* side_name);

    LineDetectorConfig cfg_;

    // 每侧的时序状态
    LineParams prev_left_, prev_right_;
    bool has_prev_left_ = false, has_prev_right_ = false;
    int prev_left_age_ = 0, prev_right_age_ = 0;
};
