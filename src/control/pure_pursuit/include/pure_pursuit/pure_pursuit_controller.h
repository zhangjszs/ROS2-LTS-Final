#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "common_msgs/msg/huat_asensing.hpp"
#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_control_command.hpp"
#include "common_msgs/msg/huat_path_limits.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "common_msgs/msg/huat_stop.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "pure_pursuit/input_guard.h"
#include "pure_pursuit/pure_pursuit_params.h"
#include "pure_pursuit/vehicle_command_encoder.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"

class PurePursuitController {
   public:
    PurePursuitController(rclcpp::Node::SharedPtr node);

    void DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void OnPathLimitsMessage(const common_msgs::msg::HuatPathLimits::ConstSharedPtr &msgs);
    void OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr &msgs);
    void OnStopMessage(const common_msgs::msg::HuatStop::ConstSharedPtr &msgs);
    void ComputeControlCommand(common_msgs::msg::HuatControlCommand& cmd, common_msgs::msg::HuatVehicleCmd& finall_cmd);
    void PublishShutdownBrake();

    double controlRate() const;
    double startupDelay() const;

    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

   private:
    int GetGoalIndex();
    int GetLookaheadIndices(int current_idx, double lookahead, std::span<const double> refx,
                            std::span<const double> refy);
    double EstimatePathCurvature(int idx) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_;
    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr sub_path_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr sub_stop_;
    rclcpp::Publisher<common_msgs::msg::HuatVehicleCmd>::SharedPtr pub_finall_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr latency_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr dropped_commands_pub_;
    uint64_t dropped_commands_ = 0;
    double current_x_ = 0.0, current_y_ = 0.0;
    int steering_, pedal_ratio_, racing_num_, racing_status_;
    VehicleCommandEncoder encoder_;
    PurePursuitParams params_;
    InputGuard input_guard_;
    Eigen::Affine3d localTf_;
    bool localTfValid_ = false;
    double filtered_angle_ = 0;
    int path_mode_ = 0;
    int last_goal_idx_ = -1;
    bool stop_requested_ = false;
    bool has_received_state_ = false;
    bool has_received_path_ = false;
    bool path_in_base_link_ = false;
    double last_latency_ = -1.0;
    rclcpp::Time last_path_time_;
    rclcpp::Time last_path_time_prev_;
    rclcpp::Time last_state_time_;
    double current_speed_ = 0.0, long_error_ = 0.0, long_current_ = 0.0, sum_error_ = 0.0;
    std::vector<double> refx_;
    std::vector<double> refy_;
};
