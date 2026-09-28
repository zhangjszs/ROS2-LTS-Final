#include "track_benchmark/benchmark_node.hpp"

#include <chrono>
#include <format>
#include <fstream>
#include <iostream>

namespace benchmark {

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
    // #17 A/C：告知评估器赛道几何（闭合性/单圈长/合法走廊）与版本标识，供有效圈/越界/回归使用。
    // 闭合性与 track_version 均取自赛道定义（与 runner / 仿真器同源，不再各自决定）。
    const double corridor_half = (current_track_.track_width > 0.0) ? current_track_.track_width * 0.5 : 1.5;
    evaluator_.SetCircuitGeometry(current_track_.total_length, corridor_half, current_track_.closed_circuit);
    evaluator_.SetTrackVersion(current_track_.versionedId());

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
    // #39：拒收计数只进 markdown 报告，不进 fsac.benchmark.kpi/v1 的 JSON 字段集
    // （那份 schema 是三条 v1 基线的口径，加字段属于口径变更，需显式重录基线才能做）。
    summary.rejected_steering_cmds = rejected_steering_cmds_;
    // #17：把累计结果落成终态（finished/timed_out/run_status），使闭环报告与离线 runner 同口径，
    // 而不是永远停在默认的 "running"。
    ApplyTerminalStatus(summary, timed_out_, require_laps_);

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
        // 同时输出机读 JSON（供 CI 回归/基线比较）：同名 .md→.json，否则追加 .json
        std::string json_path = report_file_;
        const std::string md_ext = ".md";
        if (json_path.size() >= md_ext.size() &&
            json_path.compare(json_path.size() - md_ext.size(), md_ext.size(), md_ext) == 0) {
            json_path.replace(json_path.size() - md_ext.size(), md_ext.size(), ".json");
        } else {
            json_path += ".json";
        }
        std::ofstream jout(json_path);
        if (jout.is_open()) {
            jout << KpiEvaluator::GenerateJsonReport(summary);
            RCLCPP_INFO(get_logger(), "Saved machine-readable JSON report to %s", json_path.c_str());
        }
    }
}

void BenchmarkNode::LoadParameters() {
    declare_parameter<std::string>("track_type", "skidpad");
    declare_parameter<std::string>("controller_name", "PurePursuit");
    declare_parameter<std::string>("report_file", "benchmark_report.md");
    // #17：闭环终态判定（见 kpi_evaluator.hpp::ApplyTerminalStatus 的语义）。
    declare_parameter<double>("max_runtime_s", 0.0);
    declare_parameter<int>("require_laps", 0);
    // #39：转角解码标定与控制器/仿真器同名同默认值（steering.*），不再写死。
    declare_parameter<double>("steering.neutral", 90.0);
    declare_parameter<double>("steering.units_per_degree", 1.0);
    declare_parameter<double>("steering.min_raw", 65.0);
    declare_parameter<double>("steering.max_raw", 115.0);

    get_parameter("track_type", track_type_);
    get_parameter("controller_name", controller_name_);
    get_parameter("report_file", report_file_);
    get_parameter("max_runtime_s", max_runtime_s_);
    get_parameter("require_laps", require_laps_);
    get_parameter("steering.neutral", steering_calib_.neutral);
    get_parameter("steering.units_per_degree", steering_calib_.units_per_degree);
    get_parameter("steering.min_raw", steering_calib_.min_raw);
    get_parameter("steering.max_raw", steering_calib_.max_raw);
    if (!steering_calib_.isConfigValid()) {
        RCLCPP_ERROR(get_logger(),
                     "[benchmark] steering 映射配置非法（需四字段有限、0<units、min_raw<=max_raw 且落在 "
                     "[0,255]）: neutral=%g units_per_degree=%g min_raw=%g max_raw=%g；KPI 转角结果不可信",
                     steering_calib_.neutral, steering_calib_.units_per_degree, steering_calib_.min_raw,
                     steering_calib_.max_raw);
    }
}

void BenchmarkNode::SetupSubscribersAndPublishers() {
    state_sub_ = create_subscription<common_msgs::msg::HuatCarstate>(
        "/localization/vehicle_state", 10,
        [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnVehicleState(msg); });

    cmd_sub_ = create_subscription<common_msgs::msg::HuatVehicleCmd>(
        "/vehicle_command", 10,
        [this](const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr msg) { OnVehicleCommand(msg); });

    hud_pub_ = create_publisher<visualization_msgs::msg::Marker>("/benchmark/hud_marker", 10);
    // #17：中心线用锁存(transient_local)发布 —— 只在启动后 1s 发一次，晚加入的订阅者（如闭环
    // 故障冒烟的参考路径喂入节点）也必须拿到，否则会因漏掉这一次 volatile 样本而静默无路径。
    centerline_pub_ = create_publisher<nav_msgs::msg::Path>("/benchmark/centerline_path",
                                                            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
}

void BenchmarkNode::OnVehicleCommand(const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr& msg) {
    // 转向解码统一走 SteeringCalibration（#39：标定改为 steering.* 参数可配，默认与旧写死值逐值一致）。
    // raw 不在量程内的指令不再被静默夹取成 ±满舵计入 KPI，而是拒收并计数。
    const auto decoded = steering_calib_.decodeRadChecked(static_cast<int>(msg->steering));
    if (!decoded.valid) {
        ++rejected_steering_cmds_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[benchmark] 拒收 steering raw=%d（不在 [%.0f,%.0f] 内），不计入转角样本；累计拒收=%llu",
                             static_cast<int>(msg->steering), steering_calib_.min_raw, steering_calib_.max_raw,
                             static_cast<unsigned long long>(rejected_steering_cmds_));
        return;
    }
    current_steer_rad_ = decoded.rad;
}

void BenchmarkNode::OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    double t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    // #17：超过最大运行时长后冻结评测 —— 报告反映超时时刻的累计结果，超时之后的样本不再改变判据。
    if (max_runtime_s_ > 0.0 && t > max_runtime_s_) {
        timed_out_ = true;
        return;
    }
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
