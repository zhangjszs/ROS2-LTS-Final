#pragma once

#include <common_msgs/msg/huat_carstate.hpp>
#include <common_msgs/msg/huat_path_limits.hpp>
#include <common_msgs/msg/huat_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <memory>
#include <mutex>

#include "line_detector.hpp"

class StraightLinePlannerNode {
   public:
    StraightLinePlannerNode(rclcpp::Node::SharedPtr node);
    void Run();

   private:
    void OnConeMapMessage(const common_msgs::msg::HuatMap::ConstSharedPtr& msg);
    void OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg);
    void PublishEmptyPathLimits();
    geometry_msgs::msg::Point MakePoint(double x, double y, double z);
    bool IsBoundaryPlausible(const DetectedBoundaries& b) const;
    void BuildPathLimits(const DetectedBoundaries& boundaries, const std::vector<common_msgs::msg::HuatCone>& cones,
                         double lookahead, common_msgs::msg::HuatPathLimits& out);
    void TrimConeAccumulator(const rclcpp::Time& now);
    bool HandleDegradedBoundaries(const DetectedBoundaries& boundaries,
                                  const std::vector<common_msgs::msg::HuatCone>& cones);
    std::vector<common_msgs::msg::HuatCone> AccumulatedConesInBaseLink() const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr cone_map_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr car_state_sub_;
    rclcpp::Publisher<common_msgs::msg::HuatPathLimits>::SharedPtr path_limits_pub_;

    std::unique_ptr<LineDetector> line_detector_;
    common_msgs::msg::HuatPathLimits last_valid_path_limits_;
    bool has_last_valid_;

    int road_type_;
    std::string input_topic_;
    std::string output_topic_;
    int max_empty_messages_;
    int empty_count_;
    double path_station_spacing_;
    double path_lookahead_distance_;
    double single_side_lookahead_distance_;
    double lane_half_width_;
    double plausibility_max_abs_slope_;
    double plausibility_max_intercept_diff_;
    double center_margin_;

    // 锥桶积累: 用于稀疏场景下积累足够锥桶后再拟合
    struct ConeEntry {
        common_msgs::msg::HuatCone cone;
        rclcpp::Time stamp;
    };
    std::deque<ConeEntry> cone_accumulator_;
    int accumulate_max_cones_;
    double accumulate_max_age_sec_;

    mutable std::mutex car_state_mutex_;
    double car_x_ = 0.0;
    double car_y_ = 0.0;
    double car_theta_ = 0.0;
    bool has_car_state_ = false;
};
