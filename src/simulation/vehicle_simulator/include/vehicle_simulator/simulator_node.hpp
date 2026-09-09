#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "common_msgs/msg/huat_asensing.hpp"
#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_control_command.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "common_msgs/msg/huat_stop.hpp"
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "vehicle_simulator/bicycle_model.hpp"
#include "vehicle_simulator/sensor_simulator.hpp"

namespace simulation {

class SimulatorNode : public rclcpp::Node {
   public:
    explicit SimulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

   private:
    void LoadParameters();
    void SetupPublishersAndSubscribers();
    void SetupTimers();

    void OnVehicleCommand(const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr& msg);
    void OnControlCommand(const common_msgs::msg::HuatControlCommand::ConstSharedPtr& msg);
    void OnStopMessage(const common_msgs::msg::HuatStop::ConstSharedPtr& msg);

    void UpdatePhysics();
    void PublishSensorData();
    void PublishTf();

    // 核心仿真模型
    BicycleModel bicycle_model_;
    SensorSimulator sensor_sim_;

    // 控制指令缓存
    ControlCommand current_cmd_{};
    bool stop_active_{false};

    // 参数
    double sim_rate_{100.0};
    double sensor_rate_{20.0};
    double fov_deg_{120.0};
    double max_range_{15.0};
    double noise_stddev_{0.02};
    std::string track_file_{""};

    // ROS 接口
    rclcpp::Subscription<common_msgs::msg::HuatVehicleCmd>::SharedPtr vehicle_cmd_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatControlCommand>::SharedPtr control_cmd_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatStop>::SharedPtr stop_sub_;

    rclcpp::Publisher<common_msgs::msg::HuatCarstate>::SharedPtr carstate_pub_;
    rclcpp::Publisher<common_msgs::msg::HuatASENSING>::SharedPtr ins_pub_;
    rclcpp::Publisher<common_msgs::msg::HuatMap>::SharedPtr cone_map_pub_;
    rclcpp::Publisher<common_msgs::msg::HuatMap>::SharedPtr global_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr physics_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;

    nav_msgs::msg::Path ground_truth_path_;
};

}  // namespace simulation
