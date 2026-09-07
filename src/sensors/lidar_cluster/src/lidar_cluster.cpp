#include "lidar_cluster.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <imu_subscriber.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
using namespace lidar_distortion;

LidarCluster::LidarCluster(rclcpp::Node::SharedPtr node) : node_(node) {
    Init();

    // 初始化订阅者
    sub_point_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, point_cloud_queue_size_,
        std::bind(&LidarCluster::OnPointCloud, this, std::placeholders::_1));
    // vehicle_state 订阅：dynamic_roi 或时序累积自运动补偿均需要位姿
    if (enable_dynamic_roi_ || enable_temporal_accumulation_) {
        sub_vehicle_state_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
            vehicle_state_topic_, 10,
            std::bind(&LidarCluster::OnVehicleState, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Vehicle state subscriber active (dynamic_roi=%s, temporal_accum=%s)",
                 enable_dynamic_roi_ ? "true" : "false", enable_temporal_accumulation_ ? "true" : "false");
    }
    if (enable_dynamic_roi_ || enable_ground_slope_compensation_) {
        sub_asensing_ = node_->create_subscription<common_msgs::msg::HuatASENSING>(
            ins_asensing_topic_, 10,
            std::bind(&LidarCluster::OnAsensing, this, std::placeholders::_1));
        sub_ins_ = node_->create_subscription<common_msgs::msg::HuatInsP2>(
            ins_p2_topic_, 10,
            std::bind(&LidarCluster::OnInsP2, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Pitch/ROI INS: ASENSING=%s ins_p2=%s", ins_asensing_topic_.c_str(),
                 ins_p2_topic_.c_str());
    }

    // 初始化发布者
    cone_pub_ = node_->create_publisher<common_msgs::msg::HuatConeCluster>(output_cones_topic_, 10);

    if (enable_debug_) {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(debug_bounding_boxes_topic_, 1);
        marker_pub_all_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(debug_bounding_boxes_all_topic_, 1);
        pub_debug_passthrough_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_passthrough_topic_, 1);
        pub_debug_clustered_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_clustered_topic_, 1);
        pub_debug_ground_seg_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_ground_seg_topic_, 1);
        pub_debug_satisfied_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_satisfied_topic_, 1);
        lidarClusterPublisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_point_clusters_topic_, 1);
        adjust_check_front = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_adjust_front_topic_, 1);
        adjust_check_back = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_adjust_back_topic_, 1);
        logging_pub = node_->create_publisher<nav_msgs::msg::Path>(debug_log_path_topic_, 10);
        skidpad_detection = node_->create_publisher<sensor_msgs::msg::PointCloud2>(debug_skidpad_detection_topic_, 1);
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Debug topics enabled");
    }
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Initialization complete (input=%s, input_queue=%d, cones=%s, debug_topics=%s)",
             input_topic_.c_str(), point_cloud_queue_size_, output_cones_topic_.c_str(),
             enable_debug_ ? "true" : "false");
}

void LidarCluster::ComputeGroundSegParams(GroundSegParams &params) {
    int effective_num_iter = num_iter_;
    double effective_th_dist = th_dist_;
    if (enable_ground_autotune_) {
        size_t pc_size = cloud_filtered->points.size();
        if (pc_size > static_cast<size_t>(dense_pc_threshold_)) {
            effective_num_iter = dense_num_iter_;
            effective_th_dist = dense_th_dist_;
            RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Ground autotune: dense mode (pc=%zu, iter=%d, th=%.3f)", pc_size,
                      effective_num_iter, effective_th_dist);
        } else {
            effective_num_iter = sparse_num_iter_;
            effective_th_dist = sparse_th_dist_;
            RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Ground autotune: sparse mode (pc=%zu, iter=%d, th=%.3f)", pc_size,
                      effective_num_iter, effective_th_dist);
        }
    }
    if (enable_frame_rate_protection_ && frp_active_) {
        effective_num_iter = std::min(effective_num_iter, frp_reduce_num_iter_);
        RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Frame-rate protection: reduced num_iter to %d", effective_num_iter);
    }

    double effective_th_seeds = th_seeds_;
    if (enable_ground_slope_compensation_) {
        double pitch = std::min(std::fabs(scan_pitch_), slope_pitch_max_);
        effective_th_seeds += pitch * slope_th_seeds_scale_;
        RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Slope compensation: pitch=%.2f th_seeds=%.4f", scan_pitch_, effective_th_seeds);
    }

    params.num_iter = effective_num_iter;
    params.num_lpr = num_lpr_;
    params.th_seeds = effective_th_seeds;
    params.th_dist = effective_th_dist;
    params.sensor_height = sensor_height_;
}

void LidarCluster::AccumulateTemporalFrames() {
    double cur_x, cur_y, cur_theta;
    {
        std::lock_guard<std::mutex> lock(acc_pose_mutex_);
        cur_x = acc_car_x_;
        cur_y = acc_car_y_;
        cur_theta = acc_car_theta_;
    }

    pcl::PointCloud<PointType>::Ptr current_far(new pcl::PointCloud<PointType>);
    float d2_thresh = static_cast<float>(accumulation_min_distance_ * accumulation_min_distance_);
    for (const auto &p : g_not_ground_pc->points) {
        if (p.x * p.x + p.y * p.y >= d2_thresh)
            current_far->push_back(p);
    }

    accumulated_far_clouds_.push_back({current_far, cur_x, cur_y, cur_theta});
    if (accumulated_far_clouds_.size() > static_cast<size_t>(accumulation_frames_)) {
        accumulated_far_clouds_.pop_front();
    }

    size_t extra = 0;
    for (const auto &hist : accumulated_far_clouds_) {
        if (hist.cloud != current_far)
            extra += hist.cloud->size();
    }
    g_not_ground_pc->points.reserve(g_not_ground_pc->size() + extra);

    const double cos_cur = std::cos(-cur_theta);
    const double sin_cur = std::sin(-cur_theta);
    for (const auto &hist : accumulated_far_clouds_) {
        if (hist.cloud == current_far)
            continue;
        const double cos_old = std::cos(hist.car_theta);
        const double sin_old = std::sin(hist.car_theta);
        for (const auto &p : hist.cloud->points) {
            double gx = cos_old * p.x - sin_old * p.y + hist.car_x;
            double gy = sin_old * p.x + cos_old * p.y + hist.car_y;
            double dx = gx - cur_x;
            double dy = gy - cur_y;
            PointType tp = p;
            tp.x = static_cast<float>(cos_cur * dx - sin_cur * dy);
            tp.y = static_cast<float>(sin_cur * dx + cos_cur * dy);
            g_not_ground_pc->points.push_back(tp);
        }
    }
    // 体素降采样：防止累积后点云过密导致聚类性能下降（P3-1b）
    if (accumulation_voxel_size_ > 0.0f && g_not_ground_pc->size() > 100) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(accumulation_voxel_size_, accumulation_voxel_size_, accumulation_voxel_size_);
        voxel.setInputCloud(g_not_ground_pc);
        pcl::PointCloud<PointType>::Ptr downsampled(new pcl::PointCloud<PointType>);
        voxel.filter(*downsampled);
        *g_not_ground_pc = std::move(*downsampled);
    }

    RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Temporal accum: %zu frames, far=%zu, total non-ground=%zu",
              accumulated_far_clouds_.size(), current_far->size(), g_not_ground_pc->size());
}

void LidarCluster::ApplyDistortionAdjustment(pcl::PointCloud<PointType>::Ptr &cloud) {
    if (!use_distortion_adjust_ || !imu_sub_ptr_ || !disAdjust)
        return;
    imu_sub_ptr_->ParseData(unsynced_imu_);
    ImuData synced_imu;
    if (!imu_sub_ptr_->SyncData(unsynced_imu_, synced_imu, cloud_time))
        return;
    disAdjust->SetMotionInfo(scan_period_, synced_imu);
    pcl::PointCloud<PointType>::Ptr adjusted(new pcl::PointCloud<PointType>);
    disAdjust->AdjustCloud(cloud, adjusted);
    cloud = adjusted;
}

void LidarCluster::RunAlgorithm() {
    bool no_cloud_yet = false;
    bool empty_cloud = false;
    bool waiting_for_scan = false;
    pcl::PointCloud<PointType>::Ptr processing_pc(new pcl::PointCloud<PointType>);
    {
        std::lock_guard<std::mutex> lock(lidar_mutex);
        if (!pending_cloud_) {
            waiting_for_scan = true;
            no_cloud_yet = !has_point_clouds_;
        } else {
            pending_cloud_ = false;
            no_cloud_yet = !has_point_clouds_;
            scan_header_ = in_pc.header;
            if (!current_pc_ptr || current_pc_ptr->empty()) {
                empty_cloud = true;
            } else {
                processing_pc.swap(current_pc_ptr);
                current_pc_ptr.reset(new pcl::PointCloud<PointType>());
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(acc_pose_mutex_);
        scan_speed_ = current_speed_;
        scan_pitch_ = current_pitch_;
        scan_has_ins_ = has_ins_p2_;
    }

    if (waiting_for_scan) {
        if (no_cloud_yet) {
            RCLCPP_INFO_ONCE(node_->get_logger(), "[lidar_cluster] Waiting for first point cloud on %s", input_topic_.c_str());
        }
        return;
    }
    if (empty_cloud) {
        no_data_frames_++;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[lidar_cluster] Empty point cloud (no_data_frames=%d)", no_data_frames_);
        common_msgs::msg::HuatConeCluster empty;
        empty.header = scan_header_;
        cone_pub_->publish(empty);
        return;
    }
    no_data_frames_ = 0;

    if (!scan_has_ins_ && (enable_dynamic_roi_ || enable_ground_slope_compensation_)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "[lidar_cluster] No INS pitch yet (ASENSING %s or ins_p2 %s) — "
                          "pitch compensation and dynamic ROI are INACTIVE",
                          ins_asensing_topic_.c_str(), ins_p2_topic_.c_str());
    }

    ApplyDistortionAdjustment(processing_pc);

    auto startTimePassThrough = std::chrono::steady_clock::now();
    preprocessor_->process(processing_pc, frp_active_, scan_pitch_, scan_speed_);
    cloud_filtered = processing_pc;
    auto endTimePassThrough = std::chrono::steady_clock::now();
    double elapsedTimePassThrough =
        std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough).count() /
        1000.0;

    if (enable_debug_) {
        pcl::PCLPointCloud2 pcl_pc2_filtered; pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2_filtered); pcl_conversions::fromPCL(pcl_pc2_filtered, pub_pc);
        pub_pc.header = scan_header_;
        pub_debug_passthrough_->publish(pub_pc);
    }

    if (enable_frame_rate_protection_) {
        frp_active_ = (last_frame_total_ms_ > frp_time_threshold_ms_);
        if (frp_active_)
            RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Frame-rate protection active (last=%.2f ms > thresh=%.2f)", last_frame_total_ms_,
                      frp_time_threshold_ms_);
    }

    auto startTimeSeg = std::chrono::steady_clock::now();
    GroundSegParams gs_params;
    ComputeGroundSegParams(gs_params);
    pcl::PointCloud<PointType>::Ptr ground_pc(new pcl::PointCloud<PointType>());
    ground_strategy_->segment(cloud_filtered, ground_pc, g_not_ground_pc, gs_params);

    if (enable_temporal_accumulation_)
        AccumulateTemporalFrames();

    auto endTimeSeg = std::chrono::steady_clock::now();
    double elapsedTimeSeg =
        std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg - startTimeSeg).count() / 1000.0;

    if (enable_debug_) {
        pcl::PCLPointCloud2 pcl_pc2_g_not_ground; pcl::toPCLPointCloud2(*g_not_ground_pc, pcl_pc2_g_not_ground); pcl_conversions::fromPCL(pcl_pc2_g_not_ground, pub_pc);
        pub_pc.header = scan_header_;
        pub_debug_ground_seg_->publish(pub_pc);
    }

    auto startTimeCluster = std::chrono::steady_clock::now();
    ClusterMethod();
    auto endTimeCluster = std::chrono::steady_clock::now();
    double elapsedTimeCluster =
        std::chrono::duration_cast<std::chrono::microseconds>(endTimeCluster - startTimeCluster).count() / 1000.0;
    double elapsedTime =
        std::chrono::duration_cast<std::chrono::microseconds>(endTimeCluster - startTimePassThrough).count() / 1000.0;

    if (profiler_)
        profiler_->publish(elapsedTimePassThrough, elapsedTimeSeg, elapsedTimeCluster, elapsedTime, no_data_frames_);
    last_frame_total_ms_ = elapsedTime;
}

LidarCluster::~LidarCluster() {
    RequestStop();
}

void LidarCluster::RequestStop() {
    stop_algo_ = true;
    cloud_cv_.notify_all();
}

void LidarCluster::WaitForPendingCloud() {
    std::unique_lock<std::mutex> lock(lidar_mutex);
    cloud_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return pending_cloud_ || stop_algo_.load(); });
}

void LidarCluster::OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr original_cloud_ptr) {
    // 在锁外转换，以缩小临界区
    pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 pcl_pc2_original; pcl_conversions::toPCL(*original_cloud_ptr, pcl_pc2_original); pcl::fromPCLPointCloud2(pcl_pc2_original, *tmp_cloud);

    std::lock_guard<std::mutex> lock(lidar_mutex);
    out_pc.data.clear();
    out_pc.width = 0;
    out_pc.height = 1;
    out_pc.row_step = 0;
    has_point_clouds_ = true;
    pending_cloud_ = true;

    in_pc = *original_cloud_ptr;  // 保存头信息
    current_pc_ptr.swap(tmp_cloud);

    out_pc.header.frame_id = "velodyne";
    out_pc.header.stamp = original_cloud_ptr->header.stamp;
    cloud_time = original_cloud_ptr->header.stamp.sec + original_cloud_ptr->header.stamp.nanosec * 1e-9;
    cloud_cv_.notify_one();
}

void LidarCluster::OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(acc_pose_mutex_);
    current_speed_ = static_cast<double>(msg->v);
    acc_car_x_ = msg->car_state.x;
    acc_car_y_ = msg->car_state.y;
    acc_car_theta_ = msg->car_state.theta;
}

void LidarCluster::OnInsP2(const common_msgs::msg::HuatInsP2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(acc_pose_mutex_);
    if (!has_ins_p2_) {
        has_ins_p2_ = true;
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] %s received — pitch compensation active", ins_p2_topic_.c_str());
    }
    current_pitch_ = static_cast<double>(msg->pitch);
}

void LidarCluster::OnAsensing(const common_msgs::msg::HuatASENSING::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(acc_pose_mutex_);
    if (!has_ins_p2_) {
        has_ins_p2_ = true;
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] %s received — pitch compensation active (pitch=%.2f deg)",
                 ins_asensing_topic_.c_str(), msg->pitch);
    }
    current_pitch_ = msg->pitch;
}
