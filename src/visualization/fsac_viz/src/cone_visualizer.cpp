#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common_msgs/msg/huat_cone.hpp"
#include "common_msgs/msg/huat_map.hpp"

namespace {

class ConeVisualizer {
   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr cone_map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_marker_pub_;

    void OnConeMap(const common_msgs::msg::HuatMap::ConstSharedPtr& msg) {
        for (const auto& cone : msg->cone) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.lifetime = rclcpp::Duration(0, 0);
            marker.header.stamp = node_->now();
            marker.ns = "cone_markers";
            marker.id = cone.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = cone.position_global.x;
            marker.pose.position.y = cone.position_global.y;
            marker.pose.position.z = cone.position_global.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            cone_marker_pub_->publish(marker);
        }
    }

   public:
    explicit ConeVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        node_->declare_parameter("cone_map_topic", "/sensors/cones/fused");
        node_->declare_parameter("cone_marker_topic", "/visualization/cone_markers");

        std::string cone_map_topic;
        std::string cone_marker_topic;
        node_->get_parameter("cone_map_topic", cone_map_topic);
        node_->get_parameter("cone_marker_topic", cone_marker_topic);

        cone_map_sub_ = node_->create_subscription<common_msgs::msg::HuatMap>(
            cone_map_topic, 10, [this](const common_msgs::msg::HuatMap::ConstSharedPtr msg) { OnConeMap(msg); });
        cone_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(cone_marker_topic, 10);
    }
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cone_visualizer");
    ConeVisualizer viz(node);
    rclcpp::spin(node);
    return 0;
}
