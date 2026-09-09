#include "track_benchmark/benchmark_node.hpp"

#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <numbers>

namespace benchmark {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;

}  // namespace

BenchmarkNode::BenchmarkNode(const rclcpp::NodeOptions& options) : Node("track_benchmark_node", options) {
    LoadParameters();
    SetupSubscribersAndPublishers();

    // 根据赛道类型自动配置评估中心线与赛道定义
    if (track_type_ == "skidpad") {
        current_track_ = TrackGenerator::GenerateSkidpad();
    } else if (track_type_ == "trackdrive") {
        current_track_ = TrackGenerator::GenerateTrackdriveLoop();
    } else if (track_type_ == "acceleration") {
        current_track_ = TrackGenerator::GenerateAcceleration();
    } else {
        RCLCPP_WARN(get_logger(), "Unknown track_type '%s', defaulting to Skidpad", track_type_.c_str());
        current_track_ = TrackGenerator::GenerateSkidpad();
    }

    evaluator_.SetCenterline(current_track_.centerline);
    evaluator_.SetTrackCones(current_track_.cones);

    // 延迟 1 秒后发布中心线可视化
    path_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        PublishCenterlineVisualization();
        path_timer_->cancel();
    });

    RCLCPP_INFO(get_logger(), "Benchmark initialized for track '%s' (Centerline points: %zu, Cones: %zu)",
                current_track_.name.c_str(), current_track_.centerline.size(), current_track_.cones.size());
}

BenchmarkNode::~BenchmarkNode() {
    // 节点关闭时自动输出最终评测结果并保存报告
    auto summary = evaluator_.GetSummary();
    summary.track_name = current_track_.name;
    summary.controller_name = controller_name_;

    std::string report = KpiEvaluator::GenerateMarkdownReport(summary);
    std::cout << "\n=======================================================\n";
    std::cout << report;
    std::cout << "=======================================================\n";

    if (!report_file_.empty()) {
        std::ofstream out(report_file_);
        if (out.is_open()) {
            out << report;
            RCLCPP_INFO(get_logger(), "Saved benchmark report to %s", report_file_.c_str());
        }
    }
}

void BenchmarkNode::LoadParameters() {
    declare_parameter<std::string>("track_type", "skidpad");
    declare_parameter<std::string>("controller_name", "PurePursuit");
    declare_parameter<std::string>("report_file", "benchmark_report.md");

    get_parameter("track_type", track_type_);
    get_parameter("controller_name", controller_name_);
    get_parameter("report_file", report_file_);
}

void BenchmarkNode::SetupSubscribersAndPublishers() {
    state_sub_ = create_subscription<common_msgs::msg::HuatCarstate>(
        "/localization/vehicle_state", 10,
        [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnVehicleState(msg); });

    cmd_sub_ = create_subscription<common_msgs::msg::HuatVehicleCmd>(
        "/vehicle_command", 10,
        [this](const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr msg) { OnVehicleCommand(msg); });

    hud_pub_ = create_publisher<visualization_msgs::msg::Marker>("/benchmark/hud_marker", 10);
    centerline_pub_ = create_publisher<nav_msgs::msg::Path>("/benchmark/centerline_path", 1);
}

void BenchmarkNode::OnVehicleCommand(const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr& msg) {
    double steer_deg = static_cast<double>(msg->steering) - 90.0;
    current_steer_rad_ = steer_deg * (kPi / 180.0);
}

void BenchmarkNode::OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    double t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    auto step_data =
        evaluator_.Update(msg->car_state.x, msg->car_state.y, msg->car_state.theta, msg->v, current_steer_rad_, t);
    auto summary = evaluator_.GetSummary();

    UpdateHudDisplay(step_data, summary);
}

void BenchmarkNode::UpdateHudDisplay(const KpiStepData& step, const KpiSummary& summary) {
    visualization_msgs::msg::Marker hud;
    hud.header.frame_id = "base_link";
    hud.header.stamp = now();
    hud.ns = "benchmark_hud";
    hud.id = 0;
    hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    hud.action = visualization_msgs::msg::Marker::ADD;

    // 显示在车顶前方 2m, 高度 1.5m 位置
    hud.pose.position.x = 2.0;
    hud.pose.position.y = 0.0;
    hud.pose.position.z = 1.5;

    hud.scale.z = 0.35;  // 文字大小
    hud.color.r = 0.0f;
    hud.color.g = 1.0f;
    hud.color.b = 0.4f;
    hud.color.a = 0.95f;

    hud.text = std::format("[{}] LAP: {} ({:.1f}s) | RMSE: {:.2f}m | Lat-G: {:.2f}g | Speed: {:.1f}km/h | Hits: {}",
                           controller_name_, summary.completed_laps, summary.current_lap_time_s, summary.rmse_lateral_m,
                           step.lateral_accel_g, step.speed * 3.6, summary.cone_collisions);

    hud_pub_->publish(hud);
}

void BenchmarkNode::PublishCenterlineVisualization() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();

    for (const auto& pt : current_track_.centerline) {
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.position.x = pt.x;
        p.pose.position.y = pt.y;
        p.pose.position.z = 0.05;
        path.poses.push_back(p);
    }

    centerline_pub_->publish(path);
}

}  // namespace benchmark
