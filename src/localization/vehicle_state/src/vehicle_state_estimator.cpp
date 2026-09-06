#include "vehicle_state_estimator.h"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <functional>

VehicleStateEstimator::VehicleStateEstimator(rclcpp::Node::SharedPtr node)
    : node_(node), tf_broadcaster_(*node), diag_updater_(node) {
    std::string ins_topic;
    std::string vehicle_state_topic;
    node_->get_parameter_or<std::string>("ins_topic", ins_topic, "/INS/ASENSING_INS");
    node_->get_parameter_or<std::string>("vehicle_state_topic", vehicle_state_topic, "/localization/vehicle_state");

    node_->get_parameter_or<int>("azimuth_init_frames", azimuth_init_frames_, 5);
    ins_sub_ = node_->create_subscription<common_msgs::msg::HuatASENSING>(
        ins_topic, 1, std::bind(&VehicleStateEstimator::OnInsMessage, this, std::placeholders::_1));
    state_pub_ = node_->create_publisher<common_msgs::msg::HuatCarstate>(vehicle_state_topic, 1);

    diag_updater_.setHardwareID("vehicle_state");
    diag_updater_.add("Vehicle State Health", this, &VehicleStateEstimator::DiagnoseHealth);

    RCLCPP_INFO(node_->get_logger(), "[vehicle_state] Topics: ins=%s state=%s", ins_topic.c_str(), vehicle_state_topic.c_str());
}

void VehicleStateEstimator::UpdateDiagnostics() {
    diag_updater_.force_update();
}

void VehicleStateEstimator::DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (receive_times_ == 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No INS messages received yet");
    } else if (!is_first_msg_received_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "INS received but not initialized");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }
    stat.add("ins_received", receive_times_);
    stat.add("first_msg_initialized", is_first_msg_received_);
    if (is_first_msg_received_) {
        stat.add("current_x", vehicle_state_msg_.car_state.x);
        stat.add("current_y", vehicle_state_msg_.car_state.y);
        stat.add("current_theta", vehicle_state_msg_.car_state.theta);
    } else {
        stat.add("current_x", 0.0);
        stat.add("current_y", 0.0);
        stat.add("current_theta", 0.0);
    }
}

void VehicleStateEstimator::GeodeticToEnu(double lat, double lon, double h, double lat0, double lon0, double h0,
                                          double enu_xyz[3]) {
    constexpr double a = 6378137.0;
    constexpr double b = 6356752.3142;
    constexpr double f = (a - b) / a;
    constexpr double e_sq = f * (2 - f);

    auto ecef = [&](double la, double lo, double alt, double &X, double &Y, double &Z) {
        double s = sin(la);
        double N = a / sqrt(1 - e_sq * s * s);
        X = (alt + N) * cos(la) * cos(lo);
        Y = (alt + N) * cos(la) * sin(lo);
        Z = (alt + (1 - e_sq) * N) * sin(la);
    };

    double x, y, z, x0, y0, z0;
    ecef(lat, lon, h, x, y, z);
    ecef(lat0, lon0, h0, x0, y0, z0);

    double xd = x - x0, yd = y - y0, zd = z - z0;
    double t = -cos(lon0) * xd - sin(lon0) * yd;

    enu_xyz[0] = -sin(lon0) * xd + cos(lon0) * yd;
    enu_xyz[1] = t * sin(lat0) + cos(lat0) * zd;
    enu_xyz[2] = cos(lat0) * cos(lon0) * xd + cos(lat0) * sin(lon0) * yd + sin(lat0) * zd;
}

void VehicleStateEstimator::PublishState() {
    vehicle_state_msg_.header.stamp = node_->now();
    vehicle_state_msg_.header.frame_id = "map";
    state_pub_->publish(vehicle_state_msg_);
    BroadcastTF(vehicle_state_msg_.header.stamp, vehicle_state_msg_.car_state.x, vehicle_state_msg_.car_state.y,
                vehicle_state_msg_.car_state.theta);
}

void VehicleStateEstimator::OnInsMessage(const common_msgs::msg::HuatASENSING::ConstSharedPtr msgs) {
    ins_data_.east_velocity = msgs->east_velocity;
    ins_data_.north_velocity = msgs->north_velocity;
    ins_data_.ground_velocity = msgs->ground_velocity;
    ins_data_.azimuth = msgs->azimuth;
    RCLCPP_DEBUG(node_->get_logger(), "[vehicle_state] INS status: %d", msgs->ins_status);

    // P1-E: 前 N 帧 circular mean 累积，稳定后锁定 standard_azimuth_
    if (!azimuth_locked_) {
        double az_rad = msgs->azimuth * kPi / 180.0;
        azimuth_sin_sum_ += std::sin(az_rad);
        azimuth_cos_sum_ += std::cos(az_rad);
        azimuth_sample_count_++;

        if (azimuth_sample_count_ == 1) {
            // 记录原点坐标（取第一帧位置）
            first_lat = msgs->latitude;
            first_lon = msgs->longitude;
            first_alt = msgs->altitude;
            vehicle_state_msg_.car_state.x = 0;
            vehicle_state_msg_.car_state.y = 0;
            vehicle_state_msg_.car_state.theta = 0;
        }

        if (azimuth_sample_count_ >= azimuth_init_frames_) {
            standard_azimuth_ = std::atan2(azimuth_sin_sum_, azimuth_cos_sum_) * 180.0 / kPi;
            azimuth_locked_ = true;
            RCLCPP_INFO(node_->get_logger(), "[vehicle_state] standard_azimuth locked after %d frames: %.4f deg", azimuth_init_frames_,
                     standard_azimuth_);
        } else {
            // 初始化阶段不发布零位姿，避免纯追踪把原点当有效 map 位姿
            if (!is_first_msg_received_) {
                is_first_msg_received_ = true;
            }
            receive_times_++;
            return;
        }
    }

    is_first_msg_received_ = true;
    if (receive_times_ < 3) {
        receive_times_++;
    }

    RCLCPP_DEBUG(node_->get_logger(), "[vehicle_state] Standard azimuth: %f", standard_azimuth_);
    vehicle_state_msg_.car_state.theta = -(msgs->azimuth - standard_azimuth_) * kPi / 180;
    if (vehicle_state_msg_.car_state.theta > kPi) {
        vehicle_state_msg_.car_state.theta -= 2 * kPi;
    } else if (vehicle_state_msg_.car_state.theta < -kPi) {
        vehicle_state_msg_.car_state.theta += 2 * kPi;
    }
    vehicle_state_msg_.v = std::hypot(ins_data_.east_velocity, ins_data_.north_velocity);

    GeodeticToEnu(msgs->latitude * kPi / 180, msgs->longitude * kPi / 180, msgs->altitude, first_lat * kPi / 180,
                  first_lon * kPi / 180, first_alt, &enu_xyz_[0]);

    // P1-D: 坐标变换完成后，在 OnInsMessage 中统一执行发布（与 GeodeticToEnu 分离）
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, (standard_azimuth_ - 90) * (kPi / 180));
    transform.setRotation(q);
    tf2::Vector3 state(enu_xyz_[0], enu_xyz_[1], 0);
    state = transform * state;
    vehicle_state_msg_.car_state.x = state.x();
    vehicle_state_msg_.car_state.y = state.y();

    PublishState();
}

void VehicleStateEstimator::BroadcastTF(const rclcpp::Time &stamp, double x, double y, double theta) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = "map";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(ts);
}
