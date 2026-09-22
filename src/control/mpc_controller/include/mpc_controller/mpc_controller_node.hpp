#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_path_limits.hpp"
#include "common_msgs/msg/huat_stop.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "mpc_controller/mpc_model.hpp"
#include "mpc_controller/mpc_params.hpp"
#include "steering_calibration.h"  // 仓库约定：common_msgs 手写头不带前缀（同 cone_types.h）

namespace mpc {

class MpcControllerNode : public rclcpp::Node {
   public:
    explicit MpcControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~MpcControllerNode() override = default;

   private:
    void LoadParameters();
    void SetupSubscribersAndPublishers();
    void ControlLoop();

    void OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg);
    void OnPath(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg);
    void OnStop(const common_msgs::msg::HuatStop::ConstSharedPtr& msg);

    void PublishVehicleCommand(double steering_rad, double accel_mps2);
    void PublishEmergencyBrake();
    void PublishPredictedPath(const std::vector<PredictedPoint>& trajectory);
    void PublishReferencePath(const std::vector<ReferencePoint>& reference);

    MpcConfig config_;
    MpcModel mpc_model_;
    common_msgs::vehicle::SteeringCalibration steering_calib_;

    // 状态记录
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};
    double current_speed_{0.0};

    double prev_steer_rad_{0.0};
    double prev_accel_mps2_{0.0};

    bool has_state_{false};
    bool has_path_{false};
    bool stop_requested_{false};

    rclcpp::Time last_state_time_;
    rclcpp::Time last_path_time_;
    double last_solve_time_ms_{0.0};

    std::vector<ReferencePoint> reference_path_;

    // ROS 2 通信接口
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr state_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr path_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr stop_sub_;

    rclcpp::Publisher<common_msgs::msg::HuatVehicleCmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mpc
