#include "straight_line_planner_node.hpp"

#include <common_msgs/msg/huat_tracklimits.hpp>

#include <algorithm>
#include <cmath>
#include <ranges>

#include "straight_line_geom.h"

StraightLinePlannerNode::StraightLinePlannerNode(rclcpp::Node::SharedPtr node)
    : node_(node), has_last_valid_(false), empty_count_(0) {
    node_->declare_parameter<int>("road_type", 2);
    node_->get_parameter("road_type", road_type_);
    node_->declare_parameter<std::string>("input_cone_map_topic", "/sensors/cones/fused");
    node_->get_parameter("input_cone_map_topic", input_topic_);
    std::string vehicle_state_topic;
    node_->declare_parameter<std::string>("vehicle_state_topic", "/localization/vehicle_state");
    node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
    node_->declare_parameter<std::string>("output_pathlimits_topic", "/planning/straight_line/pathlimits");
    node_->get_parameter("output_pathlimits_topic", output_topic_);
    node_->declare_parameter<int>("max_empty_messages_before_warn", 5);
    node_->get_parameter("max_empty_messages_before_warn", max_empty_messages_);
    node_->declare_parameter<double>("path_station_spacing", 0.5);
    node_->get_parameter("path_station_spacing", path_station_spacing_);
    node_->declare_parameter<double>("path_lookahead_distance", 30.0);
    node_->get_parameter("path_lookahead_distance", path_lookahead_distance_);
    node_->declare_parameter<double>("single_side_lookahead_distance", 10.0);
    node_->get_parameter("single_side_lookahead_distance", single_side_lookahead_distance_);
    node_->declare_parameter<double>("lane_half_width", 1.5);
    node_->get_parameter("lane_half_width", lane_half_width_);
    node_->declare_parameter<double>("plausibility_max_abs_slope", 0.3);
    node_->get_parameter("plausibility_max_abs_slope", plausibility_max_abs_slope_);
    // 最大允许宽度 = 2 * plausibility_max_intercept_diff_，FSAC 直线赛道实测约 4.5m，上限设为 6m
    node_->declare_parameter<double>("plausibility_max_intercept_diff", 3.0);
    node_->get_parameter("plausibility_max_intercept_diff", plausibility_max_intercept_diff_);
    node_->declare_parameter<int>("accumulate_max_cones", 20);
    node_->get_parameter("accumulate_max_cones", accumulate_max_cones_);
    // 积累的锥桶使用 position_base_link（车体系），车辆移动后旧坐标失效；0.5s 内漂移约 1-3m，可接受
    node_->declare_parameter<double>("accumulate_max_age_sec", 0.5);
    node_->get_parameter("accumulate_max_age_sec", accumulate_max_age_sec_);

    LineDetectorConfig cfg;
    node_->declare_parameter<double>("kCenterMargin", cfg.center_margin);
    node_->get_parameter("kCenterMargin", cfg.center_margin);
    center_margin_ = cfg.center_margin;
    node_->declare_parameter<double>("ransac_inlier_threshold", cfg.ransac_inlier_threshold);
    node_->get_parameter("ransac_inlier_threshold", cfg.ransac_inlier_threshold);
    node_->declare_parameter<int>("ransac_max_iterations", cfg.ransac_max_iter);
    node_->get_parameter("ransac_max_iterations", cfg.ransac_max_iter);
    node_->declare_parameter<double>("ransac_min_x_spread", cfg.ransac_min_x_spread);
    node_->get_parameter("ransac_min_x_spread", cfg.ransac_min_x_spread);
    node_->declare_parameter<double>("ransac_max_abs_slope", cfg.ransac_max_abs_slope);
    node_->get_parameter("ransac_max_abs_slope", cfg.ransac_max_abs_slope);
    node_->declare_parameter<int>("ransac_min_inliers", cfg.ransac_min_inliers);
    node_->get_parameter("ransac_min_inliers", cfg.ransac_min_inliers);
    node_->declare_parameter<double>("ransac_min_inlier_ratio", cfg.ransac_min_inlier_ratio);
    node_->get_parameter("ransac_min_inlier_ratio", cfg.ransac_min_inlier_ratio);

    node_->declare_parameter<bool>("enable_hough", cfg.enable_hough);
    node_->get_parameter("enable_hough", cfg.enable_hough);
    node_->declare_parameter<int>("hough_threshold", cfg.hough_threshold);
    node_->get_parameter("hough_threshold", cfg.hough_threshold);
    node_->declare_parameter<double>("hough_rho_resolution", cfg.hough_rho_resolution);
    node_->get_parameter("hough_rho_resolution", cfg.hough_rho_resolution);
    node_->declare_parameter<double>("hough_min_inlier_ratio", cfg.hough_min_inlier_ratio);
    node_->get_parameter("hough_min_inlier_ratio", cfg.hough_min_inlier_ratio);

    node_->declare_parameter<bool>("enable_temporal_filter", cfg.enable_temporal_filter);
    node_->get_parameter("enable_temporal_filter", cfg.enable_temporal_filter);
    node_->declare_parameter<double>("temporal_filter_alpha", cfg.temporal_filter_alpha);
    node_->get_parameter("temporal_filter_alpha", cfg.temporal_filter_alpha);
    node_->declare_parameter<double>("temporal_filter_jump_threshold", cfg.temporal_filter_jump_threshold);
    node_->get_parameter("temporal_filter_jump_threshold", cfg.temporal_filter_jump_threshold);
    node_->declare_parameter<double>("temporal_filter_intercept_jump", cfg.temporal_filter_intercept_jump);
    node_->get_parameter("temporal_filter_intercept_jump", cfg.temporal_filter_intercept_jump);
    node_->declare_parameter<int>("prev_max_age_frames", cfg.prev_max_age_frames);
    node_->get_parameter("prev_max_age_frames", cfg.prev_max_age_frames);

    line_detector_ = std::make_unique<LineDetector>(cfg);

    cone_map_sub_ = node_->create_subscription<common_msgs::msg::HuatMap>(
        input_topic_, 1, std::bind(&StraightLinePlannerNode::OnConeMapMessage, this, std::placeholders::_1));
    car_state_sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
        vehicle_state_topic, 1, std::bind(&StraightLinePlannerNode::OnCarStateMessage, this, std::placeholders::_1));
    path_limits_pub_ = node_->create_publisher<common_msgs::msg::HuatPathLimits>(output_topic_, 1);

    RCLCPP_INFO(node_->get_logger(),
                "[straight_line_planner] Initialized. road_type=%d, hough=%s, temporal_filter=%s, max|slope|=%.2f",
                road_type_, cfg.enable_hough ? "on" : "off", cfg.enable_temporal_filter ? "on" : "off",
                cfg.ransac_max_abs_slope);
}

void StraightLinePlannerNode::Run() {
    rclcpp::spin(node_);
}

geometry_msgs::msg::Point StraightLinePlannerNode::MakePoint(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

void StraightLinePlannerNode::PublishEmptyPathLimits() {
    common_msgs::msg::HuatPathLimits msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    msg.replan = false;
    path_limits_pub_->publish(msg);
}

bool StraightLinePlannerNode::IsBoundaryPlausible(const DetectedBoundaries& b) const {
    if (!b.left.valid || !b.right.valid)
        return false;
    if (std::abs(b.left.slope) > plausibility_max_abs_slope_) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[straight_line_planner] Plausibility fail: left slope=%.3f > max=%.3f", b.left.slope,
                     plausibility_max_abs_slope_);
        return false;
    }
    if (std::abs(b.right.slope) > plausibility_max_abs_slope_) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[straight_line_planner] Plausibility fail: right slope=%.3f > max=%.3f", b.right.slope,
                     plausibility_max_abs_slope_);
        return false;
    }
    // 右侧截距应大于左侧截距（在 base_link 中右侧为 +y，左侧为 -y）
    if (!(b.right.intercept > b.left.intercept)) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[straight_line_planner] Plausibility fail: right_int=%.3f <= left_int=%.3f", b.right.intercept,
                     b.left.intercept);
        return false;
    }
    double width_at_origin = b.right.intercept - b.left.intercept;
    if (width_at_origin < 0.5 || width_at_origin > 2.0 * plausibility_max_intercept_diff_) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[straight_line_planner] Plausibility fail: width=%.3f out of [0.5, %.3f]", width_at_origin,
                     2.0 * plausibility_max_intercept_diff_);
        return false;
    }
    return true;
}

void StraightLinePlannerNode::BuildPathLimits(const DetectedBoundaries& boundaries,
                                              const std::vector<common_msgs::msg::HuatCone>& cones, double lookahead,
                                              common_msgs::msg::HuatPathLimits& out) {
    out.header.stamp = node_->now();
    out.header.frame_id = "base_link";

    double x_start = 0.0;
    double x_end = lookahead;
    int num_stations = static_cast<int>((x_end - x_start) / path_station_spacing_) + 1;
    out.path.reserve(num_stations);

    bool has_left = boundaries.left.valid;
    bool has_right = boundaries.right.valid;

    for (int i = 0; i < num_stations; ++i) {
        double x = x_start + i * path_station_spacing_;
        double y_center;
        if (has_left && has_right) {
            double y_left = boundaries.left.YAt(x);
            double y_right = boundaries.right.YAt(x);
            y_center = (y_left + y_right) * 0.5;
        } else if (has_left) {
            // 左侧在 base_link 中为 -y -> 向右偏移 +lane_half_width 得到中心线
            y_center = boundaries.left.YAt(x) + lane_half_width_;
        } else if (has_right) {
            y_center = boundaries.right.YAt(x) - lane_half_width_;
        } else {
            y_center = 0.0;
        }
        out.path.push_back(MakePoint(x, y_center, 0.0));
    }

    out.tracklimits.left.clear();
    out.tracklimits.right.clear();
    std::ranges::copy_if(cones, std::back_inserter(out.tracklimits.left),
                         [this](float y) { return y < -center_margin_; },
                         [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::copy_if(cones, std::back_inserter(out.tracklimits.right),
                         [this](float y) { return y > center_margin_; },
                         [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::sort(out.tracklimits.left, {}, [](const auto& c) { return c.position_base_link.x; });
    std::ranges::sort(out.tracklimits.right, {}, [](const auto& c) { return c.position_base_link.x; });
    out.replan = true;
}

void StraightLinePlannerNode::TrimConeAccumulator(const rclcpp::Time& now) {
    while (!cone_accumulator_.empty() && (now - cone_accumulator_.front().stamp).seconds() > accumulate_max_age_sec_) {
        cone_accumulator_.pop_front();
    }
    while (static_cast<int>(cone_accumulator_.size()) > accumulate_max_cones_) {
        cone_accumulator_.pop_front();
    }
}

bool StraightLinePlannerNode::HandleDegradedBoundaries(const DetectedBoundaries& boundaries,
                                                       const std::vector<common_msgs::msg::HuatCone>& cones) {
    bool left_held = boundaries.left.valid;
    bool right_held = boundaries.right.valid;
    if (!left_held && !right_held) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[straight_line_planner] Boundaries not plausible (left valid=%d slope=%.3f int=%.3f, "
                             "right valid=%d slope=%.3f int=%.3f)",
                             left_held, boundaries.left.slope, boundaries.left.intercept, right_held,
                             boundaries.right.slope, boundaries.right.intercept);
        if (has_last_valid_) {
            last_valid_path_limits_.header.stamp = node_->now();
            path_limits_pub_->publish(last_valid_path_limits_);
        } else {
            PublishEmptyPathLimits();
        }
        return false;
    }

    DetectedBoundaries degraded;
    const char* mode_str;
    if (left_held && right_held) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[straight_line_planner] Width check failed: left_int=%.3f right_int=%.3f "
                             "left_slope=%.3f right_slope=%.3f — holding last valid path",
                             boundaries.left.intercept, boundaries.right.intercept, boundaries.left.slope,
                             boundaries.right.slope);
        if (has_last_valid_) {
            last_valid_path_limits_.header.stamp = node_->now();
            path_limits_pub_->publish(last_valid_path_limits_);
        } else {
            PublishEmptyPathLimits();
        }
        return false;
    } else if (left_held) {
        degraded.left = boundaries.left;
        degraded.success = true;
        mode_str = "left-only";
    } else {
        degraded.right = boundaries.right;
        degraded.success = true;
        mode_str = "right-only";
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[straight_line_planner] Graceful degradation: %s (lookahead=%.1fm)", mode_str,
                         single_side_lookahead_distance_);

    common_msgs::msg::HuatPathLimits path_limits;
    BuildPathLimits(degraded, cones, single_side_lookahead_distance_, path_limits);
    path_limits_pub_->publish(path_limits);
    last_valid_path_limits_ = path_limits;
    has_last_valid_ = true;
    return true;
}

void StraightLinePlannerNode::OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(car_state_mutex_);
    car_x_ = msg->car_state.x;
    car_y_ = msg->car_state.y;
    car_theta_ = msg->car_state.theta;
    has_car_state_ = true;
}

std::vector<common_msgs::msg::HuatCone> StraightLinePlannerNode::AccumulatedConesInBaseLink() const {
    double x, y, theta;
    bool has_state;
    {
        std::lock_guard<std::mutex> lock(car_state_mutex_);
        x = car_x_;
        y = car_y_;
        theta = car_theta_;
        has_state = has_car_state_;
    }
    std::vector<common_msgs::msg::HuatCone> out;
    out.reserve(cone_accumulator_.size());
    for (const auto& entry : cone_accumulator_) {
        common_msgs::msg::HuatCone cone = entry.cone;
        if (has_state) {
            const double dx = cone.position_global.x - x;
            const double dy = cone.position_global.y - y;
            double bx = 0.0, by = 0.0;
            GlobalDeltaToBaseLink(dx, dy, theta, &bx, &by);
            cone.position_base_link.x = static_cast<float>(bx);
            cone.position_base_link.y = static_cast<float>(by);
        }
        out.push_back(cone);
    }
    return out;
}

void StraightLinePlannerNode::OnConeMapMessage(const common_msgs::msg::HuatMap::ConstSharedPtr& msg) {
    if (road_type_ != 2)
        return;

    if (msg->cone.empty()) {
        ++empty_count_;
        if (empty_count_ == max_empty_messages_ + 1)
            RCLCPP_WARN(node_->get_logger(), "[straight_line_planner] Empty cone map for %d consecutive messages", empty_count_);
        if (empty_count_ <= max_empty_messages_ && has_last_valid_) {
            last_valid_path_limits_.header.stamp = node_->now();
            path_limits_pub_->publish(last_valid_path_limits_);
        } else {
            PublishEmptyPathLimits();
        }
        return;
    }
    empty_count_ = 0;

    rclcpp::Time now = node_->now();
    for (const auto& c : msg->cone)
        cone_accumulator_.push_back({c, now});
    TrimConeAccumulator(now);

    std::vector<common_msgs::msg::HuatCone> accumulated_cones = AccumulatedConesInBaseLink();
    RCLCPP_DEBUG(node_->get_logger(), "[straight_line_planner] Received %zu cones, accumulated %zu", msg->cone.size(),
                 accumulated_cones.size());

    DetectedBoundaries boundaries = line_detector_->Detect(accumulated_cones);
    bool plausible = IsBoundaryPlausible(boundaries);
    bool single_side_ok = (boundaries.left.valid != boundaries.right.valid) &&
                          (boundaries.left.valid ? std::abs(boundaries.left.slope) <= plausibility_max_abs_slope_
                                                 : std::abs(boundaries.right.slope) <= plausibility_max_abs_slope_);

    if (!plausible && !single_side_ok) {
        HandleDegradedBoundaries(boundaries, msg->cone);
        return;
    }

    common_msgs::msg::HuatPathLimits path_limits;
    BuildPathLimits(boundaries, msg->cone, plausible ? path_lookahead_distance_ : single_side_lookahead_distance_,
                    path_limits);
    path_limits_pub_->publish(path_limits);
    last_valid_path_limits_ = path_limits;
    has_last_valid_ = true;
}
