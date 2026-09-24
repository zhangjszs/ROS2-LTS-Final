#include "velocity_profiler/velocity_profiler_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "interface_contract.h"  // #14：契约话题名缺省常量（common_msgs 手写头不带前缀）

namespace velocity_profiler {

namespace {

void HsvToRgb(double h, double s, double v, float& r, float& g, float& b) {
    double c = v * s;
    double x = c * (1.0 - std::abs(std::fmod(h / 60.0, 2.0) - 1.0));
    double m = v - c;
    if (h < 60.0) {
        r = static_cast<float>(c + m);
        g = static_cast<float>(x + m);
        b = static_cast<float>(m);
    } else if (h < 120.0) {
        r = static_cast<float>(x + m);
        g = static_cast<float>(c + m);
        b = static_cast<float>(m);
    } else if (h < 180.0) {
        r = static_cast<float>(m);
        g = static_cast<float>(c + m);
        b = static_cast<float>(x + m);
    } else if (h < 240.0) {
        r = static_cast<float>(m);
        g = static_cast<float>(x + m);
        b = static_cast<float>(c + m);
    } else {
        r = static_cast<float>(c + m);
        g = static_cast<float>(m);
        b = static_cast<float>(x + m);
    }
}

}  // namespace

VelocityProfilerNode::VelocityProfilerNode(const rclcpp::NodeOptions& options)
    : Node("velocity_profiler_node", options) {
    InitParameters();

    profiler_.SetLimits(config_.limits);

    sub_path_ = create_subscription<common_msgs::msg::HuatPathLimits>(
        config_.topics.input_path, 10,
        [this](const common_msgs::msg::HuatPathLimits::ConstSharedPtr msg) { OnPathLimits(msg); });

    sub_state_ = create_subscription<common_msgs::msg::HuatCarstate>(
        config_.topics.vehicle_state, 10,
        [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnCarState(msg); });

    pub_path_ = create_publisher<common_msgs::msg::HuatPathLimits>(config_.topics.output_path, 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(config_.topics.speed_markers, 10);

    RCLCPP_INFO(get_logger(),
                "[VelocityProfiler] Initialized. max_v=%.1f m/s, min_v=%.1f m/s, max_ay=%.1f m/s^2, in=%s, out=%s",
                config_.limits.max_velocity, config_.limits.min_velocity, config_.limits.max_lat_accel,
                config_.topics.input_path.c_str(), config_.topics.output_path.c_str());
}

void VelocityProfilerNode::InitParameters() {
    declare_parameter<double>("limits.max_velocity", 20.0);
    declare_parameter<double>("limits.min_velocity", 2.0);
    declare_parameter<double>("limits.max_lat_accel", 9.8);
    declare_parameter<double>("limits.max_lon_accel", 3.5);
    declare_parameter<double>("limits.max_lon_decel", 5.0);
    declare_parameter<int>("limits.curvature_smoothing_window", 5);
    declare_parameter<bool>("limits.enable_friction_circle", true);

    declare_parameter<std::string>("topics.input_path", "/planning/raw_pathlimits");
    declare_parameter<std::string>("topics.output_path", std::string(common_msgs::contract::kTopicPathLimits));
    declare_parameter<std::string>("topics.vehicle_state", std::string(common_msgs::contract::kTopicVehicleState));
    declare_parameter<std::string>("topics.speed_markers", "/planning/viz/speed_markers");

    get_parameter("limits.max_velocity", config_.limits.max_velocity);
    get_parameter("limits.min_velocity", config_.limits.min_velocity);
    get_parameter("limits.max_lat_accel", config_.limits.max_lat_accel);
    get_parameter("limits.max_lon_accel", config_.limits.max_lon_accel);
    get_parameter("limits.max_lon_decel", config_.limits.max_lon_decel);
    get_parameter("limits.curvature_smoothing_window", config_.limits.curvature_smoothing_window);
    get_parameter("limits.enable_friction_circle", config_.limits.enable_friction_circle);

    get_parameter("topics.input_path", config_.topics.input_path);
    get_parameter("topics.output_path", config_.topics.output_path);
    get_parameter("topics.vehicle_state", config_.topics.vehicle_state);
    get_parameter("topics.speed_markers", config_.topics.speed_markers);
}

void VelocityProfilerNode::OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    current_speed_ = msg->v;
}

void VelocityProfilerNode::OnPathLimits(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg) {
    if (msg->path.empty()) {
        pub_path_->publish(*msg);
        return;
    }

    std::vector<std::pair<double, double>> raw_pts;
    raw_pts.reserve(msg->path.size());
    std::ranges::transform(msg->path, std::back_inserter(raw_pts),
                           [](const auto& pt) { return std::make_pair(pt.x, pt.y); });

    // 运行优化算法
    auto profile = profiler_.ComputeProfile(raw_pts, current_speed_);

    common_msgs::msg::HuatPathLimits out_msg = *msg;
    out_msg.target_speeds.resize(profile.size());
    for (size_t i = 0; i < out_msg.path.size() && i < profile.size(); ++i) {
        out_msg.target_speeds[i] = profile[i].target_speed;
    }

    pub_path_->publish(out_msg);

    // 发布 RViz 彩虹速度热力图
    std::string frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    PublishSpeedMarkers(profile, frame_id);
}

void VelocityProfilerNode::PublishSpeedMarkers(const std::vector<ProfilePoint>& profile, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;

    // 1. 速度热力轨迹带 (LINE_STRIP 带渐变色)
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = now();
    line_marker.ns = "velocity_profile_heatmap";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.25;  // 线宽
    line_marker.pose.orientation.w = 1.0;

    const double v_min = config_.limits.min_velocity;
    const double v_max = std::max(v_min + 0.1, config_.limits.max_velocity);

    line_marker.points.reserve(profile.size());
    line_marker.colors.reserve(profile.size());

    for (const auto& pt : profile) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.1;  // 略微高于地面
        line_marker.points.push_back(p);

        double ratio = std::clamp((pt.target_speed - v_min) / (v_max - v_min), 0.0, 1.0);
        double hue = (1.0 - ratio) * 240.0;  // 240度 (蓝) -> 0度 (红)

        std_msgs::msg::ColorRGBA color;
        color.a = 0.95f;
        HsvToRgb(hue, 1.0, 1.0, color.r, color.g, color.b);
        line_marker.colors.push_back(color);
    }
    marker_array.markers.push_back(line_marker);

    // 2. 关键节点速度数值文本标签 (每 8 个点标记一个)
    int text_id = 1;
    for (size_t i = 0; i < profile.size(); i += 8) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = now();
        text_marker.ns = "velocity_profile_labels";
        text_marker.id = text_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = profile[i].x;
        text_marker.pose.position.y = profile[i].y;
        text_marker.pose.position.z = 0.6;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.35;  // 文字大小

        std::ostringstream ss;
        double kmh = profile[i].target_speed * 3.6;
        ss << std::fixed << std::setprecision(1) << kmh << " km/h";
        text_marker.text = ss.str();

        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 0.9f;

        marker_array.markers.push_back(text_marker);
    }

    pub_markers_->publish(marker_array);
}

}  // namespace velocity_profiler
