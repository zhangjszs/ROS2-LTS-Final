#pragma once

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
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
    std::string track_type_{"skidpad"};
    std::string controller_name_{"PurePursuit"};
    std::string report_file_{""};

    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr state_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatVehicleCmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr map_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace benchmark
