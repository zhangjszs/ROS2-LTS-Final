#include <lidar_cluster.h>

#include <string>

bool LidarCluster::InitializeVisualization() {
    marker_array.markers.clear();
    marker_array_all.markers.clear();
    marker_id_ = 0;
    euc_marker.header.frame_id = "velodyne";
    euc_marker.header.stamp = node_->now();
    euc_marker.ns = "euc";
    euc_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    euc_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    euc_marker.action = visualization_msgs::msg::Marker::ADD;

    bbox_marker.header.frame_id = "velodyne";
    bbox_marker.header.stamp = node_->now();
    bbox_marker.ns = "lines";
    bbox_marker.color.r = 1.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 1.0f;
    bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    bbox_marker.scale.x = 0.01;
    bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    bbox_marker.action = visualization_msgs::msg::Marker::ADD;
    return true;
}

void LidarCluster::PublishClusterMarker(const PointType max, const PointType min, [[maybe_unused]] float euc,
                                        [[maybe_unused]] float intensity_max, [[maybe_unused]] float intensity_min,
                                        [[maybe_unused]] float intensity_mean, bool type, float conf) {
    bbox_marker.points.clear();
    bbox_marker.id = marker_id_;
    euc_marker.id = marker_id_;

    euc_marker.pose.position.x = (max.x + min.x) / 2;
    euc_marker.pose.position.y = (max.y + min.y) / 2;
    euc_marker.pose.position.z = (max.z + min.z) / 2 + 0.5;

    std::string text;
    text = "conf:" + std::to_string(conf) + "\nminH:" + std::to_string(min.z);

    euc_marker.text = text;
    euc_marker.scale.z = 0.5;

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左下

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 右下

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 右上

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左下

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 右下

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 底右上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左下

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 右下

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 右上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左下

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 右下

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 上右上

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 两层中间连接线

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 两层中间连接线 底左下

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p);  // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p);  // 左上

    if (type) {
        euc_marker.color.a = 1.0f;
        euc_marker.color.r = 0.0f;
        euc_marker.color.g = 1.0f;
        euc_marker.color.b = 0.0f;

        marker_array.markers.push_back(bbox_marker);
        marker_array.markers.push_back(euc_marker);
        // marker_array.markers.push_back(intensity_max_marker);  // 已注释：不发布强度最大标记
    } else {
        euc_marker.color.r = 1.0f;
        euc_marker.color.g = 0.0f;
        euc_marker.color.b = 0.0f;
        euc_marker.color.a = 1.0f;
        marker_array_all.markers.push_back(bbox_marker);
        marker_array_all.markers.push_back(euc_marker);
        // marker_array.markers.push_back(intensity_max_marker);  // 已注释：不发布强度最大标记
    }

    ++marker_id_;
    return;
}
