#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <limits>
#include <memory>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "autodrive_msgs/msg/huat_vision_detections.hpp"
#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_cone.hpp"
#include "common_msgs/msg/huat_cone_cluster.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "cone_fusion_utils.h"
#include "cone_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ConeFusion {
   public:
    explicit ConeFusion(rclcpp::Node::SharedPtr node);

   private:
    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<common_msgs::msg::HuatConeCluster,
                                                                             common_msgs::msg::HuatCarstate>;

    void OnSyncedMessages(const common_msgs::msg::HuatConeCluster::ConstSharedPtr cones,
                          const common_msgs::msg::HuatCarstate::ConstSharedPtr state);

    rclcpp::Node::SharedPtr node_;

    message_filters::Subscriber<common_msgs::msg::HuatConeCluster> cone_sub_mf_;
    message_filters::Subscriber<common_msgs::msg::HuatCarstate> car_state_sub_mf_;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    rclcpp::Publisher<common_msgs::msg::HuatMap>::SharedPtr transformed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;

    common_msgs::msg::HuatCarstate last_car_state_;
    bool had_received_ = false;
    bool err_data_ = false;
    std::string err_reason_;
    rclcpp::Time last_car_state_stamp_;
    double vehicle_state_base_jump_threshold_ = 2.0;
    double vehicle_state_speed_margin_ = 2.0;
    double vehicle_state_min_dt_ = 0.02;
    double vehicle_state_max_dt_ = 1.0;
    double vehicle_state_heading_threshold_ = 0.5;
    bool enable_vehicle_state_jump_check_ = true;
    double lidar_to_imu_dist_ = 1.87;

    // 锥桶多重流式清洗参数（C++20 std::views::filter）
    bool enable_cone_filtering_ = true;
    double min_cone_distance_ = 0.5;
    double max_cone_distance_ = 30.0;
    double min_fov_rad_ = -2.0;
    double max_fov_rad_ = 2.0;
    int min_confidence_ = 10;

    int frame_count_ = 0;
    int last_cone_count_ = 0;
    diagnostic_updater::Updater diag_updater_;

    // ApproximateTime 时间同步诊断字段
    rclcpp::Time diag_cone_stamp_;
    rclcpp::Time diag_vehicle_state_stamp_;
    double diag_stamp_delta_sec_ = 0.0;
    double diag_latest_vs_age_sec_ = 0.0;
    double diag_best_match_delta_sec_ = 0.0;

    void OnVisionMessage(const autodrive_msgs::msg::HuatVisionDetections::ConstSharedPtr msgs);
    void DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper& stat);
    bool IsVehicleStateJumpAbnormal(const common_msgs::msg::HuatCarstate& last_state,
                                    const common_msgs::msg::HuatCarstate& current_state, double dt);
    uint32_t ConfidenceToPercent(const common_msgs::msg::HuatConeCluster& msg, size_t index);

    void InjectVisionColor(common_msgs::msg::HuatMap& map, const std::vector<uint8_t>& lidar_sizes);

    rclcpp::Subscription<autodrive_msgs::msg::HuatVisionDetections>::SharedPtr vision_sub_;
    std::mutex vision_mutex_;
    autodrive_msgs::msg::HuatVisionDetections latest_vision_;
    bool has_vision_ = false;

    bool enable_vision_color_injection_ = false;
    // 相机内参（针孔模型）；需标定后填入 YAML，默认值为占位零
    double cam_fx_ = 0.0;
    double cam_fy_ = 0.0;
    double cam_cx_ = 0.0;
    double cam_cy_ = 0.0;
    // 径向畸变系数（Brown-Conrady k1/k2），0 = 无畸变
    double cam_k1_ = 0.0;
    double cam_k2_ = 0.0;
    // 相机相对于 base_link 的偏移（m）；x=前向，y=左向，z=上向
    double cam_offset_x_ = 0.0;
    double cam_offset_y_ = 0.0;
    double cam_offset_z_ = 1.2;
    // bbox 像素距离匹配阈值（pixels）
    double vision_match_pixel_dist_ = 80.0;
    // 仅在 image_quality <= 此值时注入颜色（0=GOOD, 1=DEGRADED）
    int vision_max_quality_ = 1;
    // vision 消息最大有效年龄（秒）
    double vision_max_age_sec_ = 0.2;
};
