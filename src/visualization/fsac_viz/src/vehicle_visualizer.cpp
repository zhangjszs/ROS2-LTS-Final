#include <functional>

#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common_msgs/msg/huat_asensing.hpp"
#include "common_msgs/msg/huat_carstate.hpp"

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kLidarToImuDistance = 1.87;
constexpr double kDegToRad = kPi / 180.0;

class VehicleVisualizer {
   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr state_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatASENSING>::SharedPtr ins_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_body_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wheel_pub_;

    common_msgs::msg::HuatCarstate vehicle_state_;
    bool has_state_ = false;
    double ins_roll_ = 0.0;
    double ins_pitch_ = 0.0;

    void CalcVehicleDirection(double roll, double pitch, double yaw, double &x, double &y, double &z) {
        double tf_roll = roll * kDegToRad;
        double tf_pitch = pitch * kDegToRad;
        x = cos(tf_pitch) * cos(-yaw + kPi / 2.0);
        y = sin(tf_roll) * sin(tf_pitch) * cos(-yaw + kPi / 2.0) + cos(tf_roll) * sin(-yaw + kPi / 2.0);
        z = -tf_roll * sin(-yaw + kPi / 2.0) + tf_pitch * sin(-yaw + kPi / 2.0);
    }

    void OnInsMessage(const common_msgs::msg::HuatASENSING::ConstSharedPtr &msg) {
        ins_roll_ = msg->roll;
        ins_pitch_ = msg->pitch;
    }

    void OnStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr &msg) {
        vehicle_state_ = *msg;
        has_state_ = true;
        PublishCarBody();
        PublishWheels();
    }

    void PublishCarBody() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = node_->now();
        marker.ns = "car_body";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(kLidarToImuDistance * cos(vehicle_state_.car_state.theta),
                                         kLidarToImuDistance * sin(vehicle_state_.car_state.theta), 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, vehicle_state_.car_state.theta);
        transform.setRotation(q);
        tf2::Vector3 pos(vehicle_state_.car_state.x, vehicle_state_.car_state.y, 0);

        tf2::Vector3 v1(0, 0.25, 0);
        tf2::Vector3 v2(0, -0.25, 0);
        tf2::Vector3 v3(-1.5, 0.25, 0);
        tf2::Vector3 v4(-1.5, -0.25, 0);
        v1 = transform * v1;
        v2 = transform * v2;
        v3 = transform * v3;
        v4 = transform * v4;

        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = pos.x() + v1.x();
        p1.y = pos.y() + v1.y();
        p1.z = 0;
        p2.x = pos.x() + v2.x();
        p2.y = pos.y() + v2.y();
        p2.z = 0;
        p3.x = pos.x() + v3.x();
        p3.y = pos.y() + v3.y();
        p3.z = 0;
        p4.x = pos.x() + v4.x();
        p4.y = pos.y() + v4.y();
        p4.z = 0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p4);
        marker.points.push_back(p3);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        car_body_pub_->publish(marker);
    }

    void PublishWheels() {
        double dir_x, dir_y, dir_z;
        CalcVehicleDirection(ins_roll_, ins_pitch_, -(vehicle_state_.car_state.theta - kPi / 2), dir_x, dir_y, dir_z);

        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(kLidarToImuDistance * cos(vehicle_state_.car_state.theta),
                                         kLidarToImuDistance * sin(vehicle_state_.car_state.theta), 0));
        tf2::Quaternion qq;
        qq.setRPY(0, 0, vehicle_state_.car_state.theta);
        transform.setRotation(qq);

        tf2::Vector3 offsets[4] = {tf2::Vector3(-0.2, 0.35, 0), tf2::Vector3(-0.2, -0.35, 0),
                                   tf2::Vector3(-1.2, 0.35, 0), tf2::Vector3(-1.2, -0.35, 0)};
        int ids[4] = {1, 2, 3, 4};

        for (int i = 0; i < 4; i++) {
            tf2::Vector3 v = transform * offsets[i];
            visualization_msgs::msg::Marker wheel;
            wheel.header.frame_id = "velodyne";
            wheel.header.stamp = node_->now();
            wheel.ns = "wheels";
            wheel.id = ids[i];
            wheel.type = visualization_msgs::msg::Marker::CYLINDER;
            wheel.action = visualization_msgs::msg::Marker::ADD;
            wheel.pose.orientation.x = dir_x;
            wheel.pose.orientation.y = dir_y;
            wheel.pose.orientation.z = dir_z;
            wheel.pose.orientation.w = 1;
            wheel.scale.x = 0.5;
            wheel.scale.y = 0.5;
            wheel.scale.z = 0.10;
            wheel.color.a = 1.0;
            wheel.color.r = 0.0;
            wheel.color.g = 1.0;
            wheel.color.b = 0.0;
            wheel.pose.position.x = v.getX() + vehicle_state_.car_state.x;
            wheel.pose.position.y = v.getY() + vehicle_state_.car_state.y;
            wheel_pub_->publish(wheel);
        }
    }

   public:
    VehicleVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        node_->declare_parameter("vehicle_state_topic", "/localization/vehicle_state");
        node_->declare_parameter("ins_topic", "/INS/ASENSING_INS");
        node_->declare_parameter("car_body_marker_topic", "/visualization/car_body");
        node_->declare_parameter("wheel_marker_topic", "/visualization/wheels");

        std::string vehicle_state_topic;
        std::string ins_topic;
        std::string car_body_marker_topic;
        std::string wheel_marker_topic;
        node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
        node_->get_parameter("ins_topic", ins_topic);
        node_->get_parameter("car_body_marker_topic", car_body_marker_topic);
        node_->get_parameter("wheel_marker_topic", wheel_marker_topic);

        state_sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
            vehicle_state_topic, 10,
            [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnStateMessage(msg); });
        ins_sub_ = node_->create_subscription<common_msgs::msg::HuatASENSING>(
            ins_topic, 10,
            [this](const common_msgs::msg::HuatASENSING::ConstSharedPtr msg) { OnInsMessage(msg); });
        car_body_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(car_body_marker_topic, 1);
        wheel_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(wheel_marker_topic, 1);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vehicle_visualizer");
    VehicleVisualizer viz(node);
    rclcpp::spin(node);
    return 0;
}
