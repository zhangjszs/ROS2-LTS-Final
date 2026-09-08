#include "cone_fusion.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <ranges>

ConeFusion::ConeFusion(rclcpp::Node::SharedPtr node)
    : node_(node), diag_updater_(node) {
    node_->declare_parameter("enable_vehicle_state_jump_check", true);
    node_->declare_parameter("vehicle_state_base_jump_threshold", 2.0);
    node_->declare_parameter("vehicle_state_speed_margin", 2.0);
    node_->declare_parameter("vehicle_state_min_dt", 0.02);
    node_->declare_parameter("vehicle_state_max_dt", 1.0);
    node_->declare_parameter("vehicle_state_heading_threshold", 0.5);
    node_->declare_parameter("lidar_to_imu_dist", 1.87);

    // C++20 锥桶流式清洗过滤参数
    node_->declare_parameter("enable_cone_filtering", true);
    node_->declare_parameter("min_cone_distance", 0.5);
    node_->declare_parameter("max_cone_distance", 30.0);
    node_->declare_parameter("min_fov_rad", -2.0);
    node_->declare_parameter("max_fov_rad", 2.0);
    node_->declare_parameter("min_confidence", 10);

    node_->get_parameter("enable_vehicle_state_jump_check", enable_vehicle_state_jump_check_);
    node_->get_parameter("vehicle_state_base_jump_threshold", vehicle_state_base_jump_threshold_);
    node_->get_parameter("vehicle_state_speed_margin", vehicle_state_speed_margin_);
    node_->get_parameter("vehicle_state_min_dt", vehicle_state_min_dt_);
    node_->get_parameter("vehicle_state_max_dt", vehicle_state_max_dt_);
    node_->get_parameter("vehicle_state_heading_threshold", vehicle_state_heading_threshold_);
    node_->get_parameter("lidar_to_imu_dist", lidar_to_imu_dist_);

    node_->get_parameter("enable_cone_filtering", enable_cone_filtering_);
    node_->get_parameter("min_cone_distance", min_cone_distance_);
    node_->get_parameter("max_cone_distance", max_cone_distance_);
    node_->get_parameter("min_fov_rad", min_fov_rad_);
    node_->get_parameter("max_fov_rad", max_fov_rad_);
    node_->get_parameter("min_confidence", min_confidence_);

    std::string input_cones_topic;
    std::string vehicle_state_topic;
    std::string transformed_cones_topic;
    std::string global_map_topic;
    node_->declare_parameter("input_cones_topic", std::string("/sensors/cones/raw"));
    node_->declare_parameter("vehicle_state_topic", std::string("/localization/vehicle_state"));
    node_->declare_parameter("transformed_cones_topic", std::string("/sensors/cones/transformed"));
    node_->declare_parameter("global_map_topic", std::string("/sensors/map/global"));
    node_->get_parameter("input_cones_topic", input_cones_topic);
    node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
    node_->get_parameter("transformed_cones_topic", transformed_cones_topic);
    node_->get_parameter("global_map_topic", global_map_topic);

    // ApproximateTime 同步队列深度（增大可容忍更大的时间差，但增加内存使用）
    int approx_queue = 20;
    node_->declare_parameter("approx_sync_queue", 20);
    node_->get_parameter("approx_sync_queue", approx_queue);

    // 视觉颜色注入参数（默认禁用，等相机标定后在 YAML 中开启）
    node_->declare_parameter("enable_vision_color_injection", false);
    node_->declare_parameter("cam_fx", 0.0);
    node_->declare_parameter("cam_fy", 0.0);
    node_->declare_parameter("cam_cx", 0.0);
    node_->declare_parameter("cam_cy", 0.0);
    node_->declare_parameter("cam_k1", 0.0);
    node_->declare_parameter("cam_k2", 0.0);
    node_->declare_parameter("cam_offset_x", 0.0);
    node_->declare_parameter("cam_offset_y", 0.0);
    node_->declare_parameter("cam_offset_z", 1.2);
    node_->declare_parameter("vision_match_pixel_dist", 80.0);
    node_->declare_parameter("vision_max_quality", 1);
    node_->declare_parameter("vision_max_age_sec", 0.2);

    node_->get_parameter("enable_vision_color_injection", enable_vision_color_injection_);
    node_->get_parameter("cam_fx", cam_fx_);
    node_->get_parameter("cam_fy", cam_fy_);
    node_->get_parameter("cam_cx", cam_cx_);
    node_->get_parameter("cam_cy", cam_cy_);
    node_->get_parameter("cam_k1", cam_k1_);
    node_->get_parameter("cam_k2", cam_k2_);
    node_->get_parameter("cam_offset_x", cam_offset_x_);
    node_->get_parameter("cam_offset_y", cam_offset_y_);
    node_->get_parameter("cam_offset_z", cam_offset_z_);
    node_->get_parameter("vision_match_pixel_dist", vision_match_pixel_dist_);
    node_->get_parameter("vision_max_quality", vision_max_quality_);
    node_->get_parameter("vision_max_age_sec", vision_max_age_sec_);

    std::string vision_detections_topic;
    node_->declare_parameter("vision_detections_topic", std::string("/perception/vision/detections"));
    node_->get_parameter("vision_detections_topic", vision_detections_topic);

    // P1-H: ApproximateTime 同步订阅——两话题在同一回调中共享时间戳，消除 50ms 延迟
    cone_sub_mf_.subscribe(node.get(), input_cones_topic, 5);
    car_state_sub_mf_.subscribe(node.get(), vehicle_state_topic, 10);
    sync_ = std::make_unique<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(static_cast<uint32_t>(approx_queue)), cone_sub_mf_, car_state_sub_mf_);
    sync_->registerCallback([this](const common_msgs::msg::HuatConeCluster::ConstSharedPtr& lidar_msg,
                                  const common_msgs::msg::HuatCarstate::ConstSharedPtr& state_msg) {
        OnSyncedMessages(lidar_msg, state_msg);
    });

    if (enable_vision_color_injection_) {
        vision_sub_ = node_->create_subscription<autodrive_msgs::msg::HuatVisionDetections>(
            vision_detections_topic, 1,
            [this](const autodrive_msgs::msg::HuatVisionDetections::ConstSharedPtr msg) { OnVisionMessage(msg); });
        RCLCPP_INFO(node_->get_logger(), "[cone_fusion] Vision color injection ENABLED (topic=%s, fx=%.1f fy=%.1f)",
                 vision_detections_topic.c_str(), cam_fx_, cam_fy_);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[cone_fusion] Vision color injection disabled (enable_vision_color_injection=false)");
    }
    transformed_pub_ = node_->create_publisher<common_msgs::msg::HuatMap>(transformed_cones_topic, 10);
    global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_topic, 10);

    diag_updater_.add("Cone Fusion Health", this, &ConeFusion::DiagnoseHealth);
    diag_updater_.setHardwareID("cone_fusion");
    RCLCPP_INFO(
        node_->get_logger(),
        "[cone_fusion] Started with ApproximateTime sync (in=%s, state=%s, out=%s, map=%s, "
        "jump_check=%s, base_jump=%.2fm, speed_margin=%.2f, dt=[%.2f,%.2f]s, heading=%.2frad, queue=%d)",
        input_cones_topic.c_str(), vehicle_state_topic.c_str(), transformed_cones_topic.c_str(),
        global_map_topic.c_str(), enable_vehicle_state_jump_check_ ? "true" : "false",
        vehicle_state_base_jump_threshold_, vehicle_state_speed_margin_, vehicle_state_min_dt_, vehicle_state_max_dt_,
        vehicle_state_heading_threshold_, approx_queue);
}

void ConeFusion::DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (!had_received_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No synchronized messages received");
    } else if (err_data_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Vehicle state jump detected");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }
    stat.add("Frames processed", frame_count_);
    stat.add("Last cone count", last_cone_count_);
    stat.add("Vehicle state received", had_received_);
    stat.add("Last jump reason", err_reason_);
    stat.add("diag_cone_stamp", diag_cone_stamp_.nanoseconds() / 1e9);
    stat.add("diag_vehicle_state_stamp", diag_vehicle_state_stamp_.nanoseconds() / 1e9);
    stat.add("diag_stamp_delta_sec", diag_stamp_delta_sec_);
    stat.add("diag_latest_vs_age_sec", diag_latest_vs_age_sec_);
    stat.add("diag_best_match_delta_sec", diag_best_match_delta_sec_);
}

uint32_t ConeFusion::ConfidenceToPercent(const common_msgs::msg::HuatConeCluster &msg, size_t index) {
    return cone_fusion_utils::ConfidenceToPercent(msg.confidence.data(), msg.confidence.size(), index);
}

bool ConeFusion::IsVehicleStateJumpAbnormal(const common_msgs::msg::HuatCarstate &last_state,
                                            const common_msgs::msg::HuatCarstate &current_state, double dt) {
    cone_fusion_utils::VehicleState last_vs{
        .x = last_state.car_state.x,
        .y = last_state.car_state.y,
        .theta = last_state.car_state.theta,
        .v = static_cast<double>(last_state.v)
    };
    cone_fusion_utils::VehicleState curr_vs{
        .x = current_state.car_state.x,
        .y = current_state.car_state.y,
        .theta = current_state.car_state.theta,
        .v = static_cast<double>(current_state.v)
    };
    return cone_fusion_utils::IsVehicleStateJumpAbnormal(
        last_vs, curr_vs, dt, vehicle_state_base_jump_threshold_, vehicle_state_speed_margin_, vehicle_state_min_dt_,
        vehicle_state_max_dt_, vehicle_state_heading_threshold_, &err_reason_);
}

void ConeFusion::OnSyncedMessages(const common_msgs::msg::HuatConeCluster::ConstSharedPtr cones,
                                  const common_msgs::msg::HuatCarstate::ConstSharedPtr state) {
    if (cones->points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[cone_fusion] Empty cone data");
        common_msgs::msg::HuatMap empty_map;
        empty_map.header = cones->header;
        empty_map.header.frame_id = "map";
        transformed_pub_->publish(empty_map);
        last_car_state_ = *state;
        last_car_state_stamp_ = (state->header.stamp.sec == 0 && state->header.stamp.nanosec == 0) ? node_->now() : rclcpp::Time(state->header.stamp);
        had_received_ = true;
        last_cone_count_ = 0;
        diag_updater_.force_update();
        return;
    }

    // 时间同步诊断快照：ApproximateTime 保证 delta 通常 < 10ms
    diag_cone_stamp_ = cones->header.stamp;
    diag_vehicle_state_stamp_ = (state->header.stamp.sec == 0 && state->header.stamp.nanosec == 0) ? node_->now() : rclcpp::Time(state->header.stamp);
    diag_stamp_delta_sec_ = (diag_cone_stamp_ - diag_vehicle_state_stamp_).seconds();
    diag_best_match_delta_sec_ = diag_stamp_delta_sec_;
    diag_latest_vs_age_sec_ = diag_stamp_delta_sec_;

    if (std::fabs(diag_stamp_delta_sec_) > 0.1) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[cone_fusion] ApproxTime sync delta=%.3fs (>0.1s)", diag_stamp_delta_sec_);
    }

    // 跳变检测（使用 ApproximateTime 匹配的位姿，与上一帧对比）
    if (enable_vehicle_state_jump_check_ && had_received_) {
        const double dt = (diag_vehicle_state_stamp_ - last_car_state_stamp_).seconds();
        if (IsVehicleStateJumpAbnormal(last_car_state_, *state, dt)) {
            err_data_ = true;
        }
    }

    // 更新上一帧记录（无论是否跳过，以免下一帧误判）
    last_car_state_ = *state;
    last_car_state_stamp_ = diag_vehicle_state_stamp_;
    had_received_ = true;

    if (err_data_) {
        err_data_ = false;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[cone_fusion] Vehicle state jump detected, skipping frame (%s)", err_reason_.c_str());
        return;
    }

    // 根据时间匹配的车辆位姿构建旋转矩阵
    double theta = state->car_state.theta;
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    Eigen::Matrix2d R;
    R << cos_t, -sin_t, sin_t, cos_t;

    Eigen::Vector2d lidar_offset(lidar_to_imu_dist_, 0.0);
    Eigen::Vector2d vehicle_pos(state->car_state.x, state->car_state.y);
    Eigen::Vector2d T = vehicle_pos + R * lidar_offset;

    common_msgs::msg::HuatMap transformed_map;
    transformed_map.header = cones->header;
    transformed_map.header.frame_id = "map";
    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<uint8_t> lidar_sizes;
    lidar_sizes.reserve(cones->points.size());

    const double min_dist_sq = min_cone_distance_ * min_cone_distance_;
    const double max_dist_sq = max_cone_distance_ * max_cone_distance_;
    const size_t total_points = cones->points.size();

    // C++20 惰性流式清洗流水线：按需组合距离、视场角和置信度过滤，无中间 vector 堆分配
    auto indices = std::views::iota(size_t{0}, total_points);

    auto valid_cone_indices = indices
        | std::views::filter([&](size_t i) {
            const auto &p = cones->points[i];
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        })
        | std::views::filter([&](size_t i) {
            if (!enable_cone_filtering_) return true;
            const auto &p = cones->points[i];
            const double d2 = p.x * p.x + p.y * p.y;
            return d2 >= min_dist_sq && d2 <= max_dist_sq;
        })
        | std::views::filter([&](size_t i) {
            if (!enable_cone_filtering_) return true;
            const auto &p = cones->points[i];
            const double angle = std::atan2(p.y, p.x);
            return angle >= min_fov_rad_ && angle <= max_fov_rad_;
        })
        | std::views::filter([&](size_t i) {
            if (!enable_cone_filtering_) return true;
            return ConfidenceToPercent(*cones, i) >= static_cast<uint32_t>(min_confidence_);
        });

    for (size_t i : valid_cone_indices) {
        Eigen::Vector2d local_xy(cones->points[i].x, cones->points[i].y);
        Eigen::Vector2d global_xy = R * local_xy + T;

        common_msgs::msg::HuatCone cone;
        cone.header = transformed_map.header;
        cone.position_base_link.x = cones->points[i].x + lidar_to_imu_dist_;
        cone.position_base_link.y = cones->points[i].y;
        cone.position_base_link.z = cones->points[i].z;
        cone.position_global.x = global_xy.x();
        cone.position_global.y = global_xy.y();
        cone.position_global.z = cones->points[i].z;
        cone.id = -1;
        cone.confidence = ConfidenceToPercent(*cones, i);
        cone.type = huat_cone::NONE;
        uint8_t lidar_size = huat_cone::SIZE_UNKNOWN;
        if (i < cones->type.size())
            lidar_size = cones->type[i];
        lidar_sizes.push_back(lidar_size);
        transformed_map.cone.push_back(cone);

        pcl::PointXYZ pcl_point;
        pcl_point.x = global_xy.x();
        pcl_point.y = global_xy.y();
        pcl_point.z = cones->points[i].z;
        global_cloud->push_back(pcl_point);
    }

    if (enable_vision_color_injection_) {
        InjectVisionColor(transformed_map, lidar_sizes);
    }

    transformed_pub_->publish(transformed_map);
    frame_count_++;
    last_cone_count_ = transformed_map.cone.size();
    diag_updater_.force_update();

    sensor_msgs::msg::PointCloud2 global_cloud_msg;
    pcl::PCLPointCloud2 pcl_pc2_global; pcl::toPCLPointCloud2(*global_cloud, pcl_pc2_global); pcl_conversions::fromPCL(pcl_pc2_global, global_cloud_msg);
    global_cloud_msg.header.frame_id = "map";
    global_cloud_msg.header.stamp = cones->header.stamp;
    global_map_pub_->publish(global_cloud_msg);
}

void ConeFusion::OnVisionMessage(const autodrive_msgs::msg::HuatVisionDetections::ConstSharedPtr msgs) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    latest_vision_ = *msgs;
    has_vision_ = true;
}

void ConeFusion::InjectVisionColor(common_msgs::msg::HuatMap &map, const std::vector<uint8_t> &lidar_sizes) {
    if (cam_fx_ <= 0.0 || cam_fy_ <= 0.0) {
        return;
    }

    autodrive_msgs::msg::HuatVisionDetections vision;
    bool valid = false;
    {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        if (!has_vision_)
            return;
        vision = latest_vision_;
        valid = true;
    }
    if (!valid)
        return;

    double age = (rclcpp::Time(map.header.stamp) - rclcpp::Time(vision.header.stamp)).seconds();
    if (std::fabs(age) > vision_max_age_sec_) {
        RCLCPP_DEBUG(node_->get_logger(), "[cone_fusion] Vision message stale (age=%.3fs), skipping color injection", age);
        return;
    }

    if (static_cast<int>(vision.image_quality) > vision_max_quality_) {
        return;
    }

    size_t n_vision = vision.x.size();
    if (n_vision == 0)
        return;

    for (size_t i = 0; i < map.cone.size(); ++i) {
        auto &cone = map.cone[i];
        double cx = cone.position_base_link.x - cam_offset_x_;
        double cy = cone.position_base_link.y - cam_offset_y_;
        double cz = cone.position_base_link.z - cam_offset_z_;

        if (cx <= 0.1)
            continue;

        double xn = -cy / cx;
        double yn = -cz / cx;
        if (cam_k1_ != 0.0 || cam_k2_ != 0.0) {
            double r2 = xn * xn + yn * yn;
            double dist = 1.0 + cam_k1_ * r2 + cam_k2_ * r2 * r2;
            xn *= dist;
            yn *= dist;
        }
        double u = cam_fx_ * xn + cam_cx_;
        double v = cam_fy_ * yn + cam_cy_;

        double best_dist = vision_match_pixel_dist_ * vision_match_pixel_dist_;
        int best_idx = -1;
        for (size_t j = 0; j < n_vision; ++j) {
            double du = u - vision.x[j];
            double dv = v - vision.y[j];
            double d2 = du * du + dv * dv;
            if (d2 < best_dist) {
                best_dist = d2;
                best_idx = static_cast<int>(j);
            }
        }

        if (best_idx >= 0 && huat_cone::IsKnownColor(vision.color_types[static_cast<size_t>(best_idx)])) {
            const uint8_t lidar_size = (i < lidar_sizes.size()) ? lidar_sizes[i] : huat_cone::SIZE_UNKNOWN;
            cone.type =
                huat_cone::MergeVisionColorWithLidarSize(vision.color_types[static_cast<size_t>(best_idx)], lidar_size);
        }
    }
}
