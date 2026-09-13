#pragma once

#include <common_msgs/msg/huat_carstate.hpp>
#include <common_msgs/msg/huat_path_limits.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "velocity_profiler/profiler_params.hpp"
#include "velocity_profiler/velocity_profiler.hpp"

namespace velocity_profiler {

class VelocityProfilerNode : public rclcpp::Node {
   public:
    explicit VelocityProfilerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void InitParameters();
    void OnPathLimits(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg);
    void OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg);
    void PublishSpeedMarkers(const std::vector<ProfilePoint>& profile, const std::string& frame_id);

    ProfilerConfig config_;
    VelocityProfiler profiler_;

    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr sub_path_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_state_;
    rclcpp::Publisher<common_msgs::msg::HuatPathLimits>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

    double current_speed_{0.0};
};

}  // namespace velocity_profiler
