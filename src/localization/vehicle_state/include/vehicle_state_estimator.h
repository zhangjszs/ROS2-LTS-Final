#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "common_msgs/msg/huat_asensing.hpp"
#include "common_msgs/msg/huat_carstate.hpp"

constexpr double kPi = 3.14159265358979;
constexpr double kDegToRad = kPi / 180.0;

class VehicleStateEstimator {
   private:
    /**
     * @brief 用于统计接收惯导消息的次数
     */
    int receive_times_ = 0;

    /**
     * @brief 订阅惯导话题：/INS/ASENSING_INS
     */
    rclcpp::Subscription<common_msgs::msg::HuatASENSING>::SharedPtr ins_sub_;

    /**
     * @brief ros节点
     */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief 发布转换过的话题：/Carstate
     */
    rclcpp::Publisher<common_msgs::msg::HuatCarstate>::SharedPtr state_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /**
     * @brief 用于存储经纬度转换为东北天坐标系的x,y,z
     */
    double enu_xyz_[3];

    /**
     * @brief 第一次接收的经纬度之类的数据是
     */
    double first_lat, first_lon, first_alt;

    /**
     * @brief 标志符，false代表第一次没有接收到惯导消息
     */
    bool is_first_msg_received_ = false;

    /**
     * @brief 用于存储需要发布的消息的信息
     */
    common_msgs::msg::HuatCarstate vehicle_state_msg_;

    /**
     * @brief 用于存储接收消息的信息
     */
    common_msgs::msg::HuatASENSING ins_data_;

    /**
     * @brief 用于存储第一次的角度值，用于转换整个东北天坐标系为以车身起始位置为基准的坐标系
     */
    double standard_azimuth_;

   public:
    explicit VehicleStateEstimator(rclcpp::Node::SharedPtr node);
    void OnInsMessage(const common_msgs::msg::HuatASENSING::ConstSharedPtr msgs);
    void UpdateDiagnostics();

   private:
    // 纯坐标变换：大地坐标 → ENU，无发布副作用
    void GeodeticToEnu(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3]);
    void DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void BroadcastTF(const rclcpp::Time &stamp, double x, double y, double theta);
    void PublishState();

    diagnostic_updater::Updater diag_updater_;

    // P1-E: 方位角多帧平均初始化（circular mean，防止首帧漂移传播）
    int azimuth_init_frames_ = 5;
    int azimuth_sample_count_ = 0;
    double azimuth_sin_sum_ = 0.0;
    double azimuth_cos_sum_ = 0.0;
    bool azimuth_locked_ = false;
};
