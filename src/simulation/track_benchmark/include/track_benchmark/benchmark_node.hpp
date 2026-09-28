#pragma once

#include <cstdint>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "steering_calibration.h"  // 仓库约定：common_msgs 手写头不带前缀（同 cone_types.h）
#include "track_benchmark/kpi_evaluator.hpp"
#include "track_benchmark/track_generator.hpp"

namespace benchmark {

class BenchmarkNode : public rclcpp::Node {
   public:
    explicit BenchmarkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~BenchmarkNode() override;

   private:
    void LoadParameters();
    void SetupSubscribersAndPublishers();
    void PublishCenterlineVisualization();

    void OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg);
    void OnVehicleCommand(const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr& msg);
    void OnConeMap(const common_msgs::msg::HuatMap::ConstSharedPtr& msg);

    void UpdateHudDisplay(const KpiStepData& step, const KpiSummary& summary);

    KpiEvaluator evaluator_;
    TrackDefinition current_track_;

    // 当前状态
    double current_steer_rad_{0.0};
    // #39：评测侧转角标定也走 steering.* 参数（此前是 `static const SteeringCalibration{}`，
    // 默认 90/1/±25°；接非默认协议的底盘时 KPI 的转角/抖动会静默按错误协议换算）。
    common_msgs::vehicle::SteeringCalibration steering_calib_;
    // 越界 raw（不可解释指令）计数：不计入 KPI 样本，仅报告与告警。
    uint64_t rejected_steering_cmds_{0};
    std::string track_type_{"skidpad"};
    std::string controller_name_{"PurePursuit"};
    std::string report_file_{""};

    // #17：闭环终态判定参数（与离线 runner 的 run_status 语义对齐，使闭环冒烟也能读到
    // 终态而非恒 running）。max_runtime_s<=0 表示不限时；require_laps<=0 表示“至少 1 个
    // 有效圈”即完赛。
    double max_runtime_s_{0.0};
    int require_laps_{0};
    bool timed_out_{false};

    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr state_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatVehicleCmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr map_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace benchmark
