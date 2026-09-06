#include <ground_segmentation/ransac_ground_strategy.h>
#include <ground_segmentation/svd_ground_strategy.h>
#include <lidar_cluster.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "cone_types.h"
#include "string_utils.h"

void LidarCluster::Init() {
    LoadParams();
    InitStrategy();
    InitState();
}

void LidarCluster::LoadTopicParams() {
    node_->declare_parameter("input_topic", std::string("/velodyne_points"));
    node_->declare_parameter("point_cloud_queue_size", 1);
    node_->declare_parameter("output_cones_topic", std::string("/sensors/cones/raw"));
    node_->declare_parameter("vehicle_state_topic", std::string("/localization/vehicle_state"));
    node_->declare_parameter("ins_p2_topic", std::string("/sensors/ins_p2"));
    node_->declare_parameter("ins_asensing_topic", std::string("/INS/ASENSING_INS"));
    node_->declare_parameter("profiling_topic", std::string("/debug/lidar_cluster/profiling"));
    node_->declare_parameter("debug_bounding_boxes_topic", std::string("/debug/lidar_cluster/bounding_boxes"));
    node_->declare_parameter("debug_bounding_boxes_all_topic", std::string("/debug/lidar_cluster/bounding_boxes_all"));
    node_->declare_parameter("debug_passthrough_topic", std::string("/debug/lidar_cluster/passthrough"));
    node_->declare_parameter("debug_clustered_topic", std::string("/debug/lidar_cluster/clustered"));
    node_->declare_parameter("debug_ground_seg_topic", std::string("/debug/lidar_cluster/ground_seg"));
    node_->declare_parameter("debug_satisfied_topic", std::string("/debug/lidar_cluster/satisfied"));
    node_->declare_parameter("debug_point_clusters_topic", std::string("/debug/lidar_cluster/point_clusters"));
    node_->declare_parameter("debug_adjust_front_topic", std::string("/debug/lidar_cluster/adjust_front"));
    node_->declare_parameter("debug_adjust_back_topic", std::string("/debug/lidar_cluster/adjust_back"));
    node_->declare_parameter("debug_log_path_topic", std::string("/debug/lidar_cluster/log_path"));
    node_->declare_parameter("debug_skidpad_detection_topic", std::string("/debug/lidar_cluster/skidpad_detection"));
    node_->declare_parameter("no_ground_point_topic", std::string("/debug/lidar_cluster/no_ground"));
    node_->declare_parameter("ground_point_topic", std::string("/debug/lidar_cluster/ground"));
    node_->declare_parameter("all_points_topic", std::string("/debug/lidar_cluster/all_points"));

    node_->get_parameter("input_topic", input_topic_);
    node_->get_parameter("point_cloud_queue_size", point_cloud_queue_size_);
    if (point_cloud_queue_size_ < 1) {
        RCLCPP_WARN(node_->get_logger(), "[lidar_cluster] Invalid point_cloud_queue_size=%d, clamping to 1", point_cloud_queue_size_);
        point_cloud_queue_size_ = 1;
    }
    node_->get_parameter("output_cones_topic", output_cones_topic_);
    node_->get_parameter("vehicle_state_topic", vehicle_state_topic_);
    node_->get_parameter("ins_p2_topic", ins_p2_topic_);
    node_->get_parameter("ins_asensing_topic", ins_asensing_topic_);
    node_->get_parameter("profiling_topic", profiling_topic_);
    node_->get_parameter("debug_bounding_boxes_topic", debug_bounding_boxes_topic_);
    node_->get_parameter("debug_bounding_boxes_all_topic", debug_bounding_boxes_all_topic_);
    node_->get_parameter("debug_passthrough_topic", debug_passthrough_topic_);
    node_->get_parameter("debug_clustered_topic", debug_clustered_topic_);
    node_->get_parameter("debug_ground_seg_topic", debug_ground_seg_topic_);
    node_->get_parameter("debug_satisfied_topic", debug_satisfied_topic_);
    node_->get_parameter("debug_point_clusters_topic", debug_point_clusters_topic_);
    node_->get_parameter("debug_adjust_front_topic", debug_adjust_front_topic_);
    node_->get_parameter("debug_adjust_back_topic", debug_adjust_back_topic_);
    node_->get_parameter("debug_log_path_topic", debug_log_path_topic_);
    node_->get_parameter("debug_skidpad_detection_topic", debug_skidpad_detection_topic_);
    node_->get_parameter("no_ground_point_topic", no_ground_topic_);
    node_->get_parameter("ground_point_topic", ground_topic_);
    node_->get_parameter("all_points_topic", all_points_topic_);
}

void LidarCluster::LoadGroundParams() {
    node_->declare_parameter("sensor_height", 0.135);
    node_->declare_parameter("sensor_model", 16);
    node_->declare_parameter("num_iter", 3);
    node_->declare_parameter("num_lpr", 5);
    node_->declare_parameter("th_seeds", 0.03);
    node_->declare_parameter("th_dist", 0.03);
    node_->declare_parameter("road_type", 2);
    node_->declare_parameter("vis", 0);
    node_->declare_parameter("enable_debug_topics", false);
    node_->declare_parameter("use_distortion_adjust", false);
    node_->declare_parameter("scan_period", 0.1f);
    node_->declare_parameter("imu_topic", std::string("/sensors/imu/data"));
    node_->declare_parameter("str_range", std::string("15,30,45,60"));
    node_->declare_parameter("str_seg_distance", std::string("0.5,1.1,1.6,2.1,2.6"));

    node_->get_parameter("sensor_height", sensor_height_);
    node_->get_parameter("sensor_model", sensor_model_);
    node_->get_parameter("num_iter", num_iter_);
    node_->get_parameter("num_lpr", num_lpr_);
    node_->get_parameter("th_seeds", th_seeds_);
    node_->get_parameter("th_dist", th_dist_);
    node_->get_parameter("road_type", road_type_);
    node_->get_parameter("vis", vis_);
    node_->get_parameter("enable_debug_topics", enable_debug_);
    node_->get_parameter("use_distortion_adjust", use_distortion_adjust_);
    node_->get_parameter("scan_period", scan_period_);
    node_->get_parameter("imu_topic", imu_topic_);
    node_->get_parameter("str_range", str_range_);
    node_->get_parameter("str_seg_distance", str_seg_distance_);
}

void LidarCluster::LoadFeatureParams() {
    node_->declare_parameter("min_height", -1.0);
    node_->declare_parameter("max_height", -1.0);
    node_->declare_parameter("min_area", -1.0);
    node_->declare_parameter("max_area", -1.0);
    node_->declare_parameter("max_box_altitude", 0.0);
    node_->declare_parameter("max_tilt_angle", 25.0);
    node_->declare_parameter("cone_confidence_threshold", 0.5);
    node_->declare_parameter("enable_dynamic_roi", false);
    node_->declare_parameter("enable_ground_autotune", false);
    node_->declare_parameter("dense_pc_threshold", 15000);
    node_->declare_parameter("dense_th_dist", 0.025);
    node_->declare_parameter("sparse_th_dist", 0.05);
    node_->declare_parameter("dense_num_iter", 5);
    node_->declare_parameter("sparse_num_iter", 3);
    node_->declare_parameter("enable_profiling", false);
    node_->declare_parameter("profiling_warn_ms", 80.0);
    node_->declare_parameter("profiling_error_ms", 100.0);
    node_->declare_parameter("profiling_warn_consecutive_threshold", 3);
    node_->declare_parameter("enable_temporal_accumulation", false);
    node_->declare_parameter("accumulation_frames", 3);
    node_->declare_parameter("accumulation_min_distance", 15.0);
    node_->declare_parameter("accumulation_voxel_size", 0.1);
    node_->declare_parameter("enable_ground_slope_compensation", false);
    node_->declare_parameter("slope_th_seeds_scale", 0.002);
    node_->declare_parameter("slope_pitch_max", 15.0);
    node_->declare_parameter("enable_frame_rate_protection", false);
    node_->declare_parameter("frp_time_threshold_ms", 80.0);
    node_->declare_parameter("frp_skip_sor", true);
    node_->declare_parameter("frp_reduce_num_iter", 2);
    node_->declare_parameter("enable_cone_type_classification", false);
    node_->declare_parameter("large_min_height", 0.25);
    node_->declare_parameter("large_min_area", 0.12);
    node_->declare_parameter("small_max_height", 0.20);
    node_->declare_parameter("small_max_area", 0.08);
    node_->declare_parameter("enable_single_frame_dedup", true);
    node_->declare_parameter("single_frame_dedup_radius", 0.5);
    node_->declare_parameter("conf_penalty_aspect", 2.0);
    node_->declare_parameter("conf_penalty_over_max_accel", 0.05);
    node_->declare_parameter("conf_penalty_height_over", 7.0);
    node_->declare_parameter("conf_penalty_area_over", 2.0);
    node_->declare_parameter("conf_penalty_height_under", 1.5);
    node_->declare_parameter("conf_penalty_area_under", 2.0);
    node_->declare_parameter("conf_penalty_tilt", 0.5);

    node_->get_parameter("min_height", min_height_);
    node_->get_parameter("max_height", max_height_);
    node_->get_parameter("min_area", min_area_);
    node_->get_parameter("max_area", max_area_);
    node_->get_parameter("max_box_altitude", max_box_altitude_);
    node_->get_parameter("max_tilt_angle", max_tilt_angle_);
    node_->get_parameter("cone_confidence_threshold", cone_confidence_threshold_);
    node_->get_parameter("enable_dynamic_roi", enable_dynamic_roi_);
    node_->get_parameter("enable_ground_autotune", enable_ground_autotune_);
    node_->get_parameter("dense_pc_threshold", dense_pc_threshold_);
    node_->get_parameter("dense_th_dist", dense_th_dist_);
    node_->get_parameter("sparse_th_dist", sparse_th_dist_);
    node_->get_parameter("dense_num_iter", dense_num_iter_);
    node_->get_parameter("sparse_num_iter", sparse_num_iter_);
    node_->get_parameter("enable_profiling", enable_profiling_);

    double profiling_warn_ms = 80.0, profiling_error_ms = 100.0;
    int profiling_warn_consecutive_threshold = 3;
    node_->get_parameter("profiling_warn_ms", profiling_warn_ms);
    node_->get_parameter("profiling_error_ms", profiling_error_ms);
    node_->get_parameter("profiling_warn_consecutive_threshold", profiling_warn_consecutive_threshold);
    if (enable_profiling_)
        profiler_ = std::make_unique<FrameProfiler>(node_, profiling_topic_, profiling_warn_ms,
                                                    profiling_error_ms, profiling_warn_consecutive_threshold);

    node_->get_parameter("enable_temporal_accumulation", enable_temporal_accumulation_);
    node_->get_parameter("accumulation_frames", accumulation_frames_);
    node_->get_parameter("accumulation_min_distance", accumulation_min_distance_);
    node_->get_parameter("accumulation_voxel_size", accumulation_voxel_size_);
    node_->get_parameter("enable_ground_slope_compensation", enable_ground_slope_compensation_);
    node_->get_parameter("slope_th_seeds_scale", slope_th_seeds_scale_);
    node_->get_parameter("slope_pitch_max", slope_pitch_max_);
    node_->get_parameter("enable_frame_rate_protection", enable_frame_rate_protection_);
    node_->get_parameter("frp_time_threshold_ms", frp_time_threshold_ms_);
    node_->get_parameter("frp_skip_sor", frp_skip_sor_);
    node_->get_parameter("frp_reduce_num_iter", frp_reduce_num_iter_);
    node_->get_parameter("enable_cone_type_classification", enable_cone_type_classification_);
    node_->get_parameter("large_min_height", large_min_height_);
    node_->get_parameter("large_min_area", large_min_area_);
    node_->get_parameter("small_max_height", small_max_height_);
    node_->get_parameter("small_max_area", small_max_area_);
    node_->get_parameter("enable_single_frame_dedup", enable_single_frame_dedup_);
    node_->get_parameter("single_frame_dedup_radius", single_frame_dedup_radius_);
    node_->get_parameter("conf_penalty_aspect", conf_penalty_aspect_);
    node_->get_parameter("conf_penalty_over_max_accel", conf_penalty_over_max_accel_);
    node_->get_parameter("conf_penalty_height_over", conf_penalty_height_over_);
    node_->get_parameter("conf_penalty_area_over", conf_penalty_area_over_);
    node_->get_parameter("conf_penalty_height_under", conf_penalty_height_under_);
    node_->get_parameter("conf_penalty_area_under", conf_penalty_area_under_);
    node_->get_parameter("conf_penalty_tilt", conf_penalty_tilt_);
}

void LidarCluster::LoadParams() {
    LoadTopicParams();
    LoadGroundParams();
    LoadFeatureParams();
}

void LidarCluster::InitStrategy() {
    std::string ground_seg_strategy = "svd";
    node_->declare_parameter("ground_segmentation_strategy", std::string("svd"));
    node_->get_parameter("ground_segmentation_strategy", ground_seg_strategy);
    if (ground_seg_strategy == "svd") {
        ground_strategy_.reset(new SvdGroundStrategy());
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Ground segmentation strategy: SVD");
    } else if (ground_seg_strategy == "ransac") {
        ground_strategy_.reset(new RansacGroundStrategy());
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Ground segmentation strategy: RANSAC");
    } else {
        RCLCPP_WARN(node_->get_logger(), "[lidar_cluster] Unknown ground segmentation strategy '%s', falling back to SVD",
                 ground_seg_strategy.c_str());
        ground_strategy_.reset(new SvdGroundStrategy());
    }

    preprocessor_ = std::make_unique<PointCloudPreprocessor>(node_);
}

void LidarCluster::InitState() {
    // 列出所有参数（启动时）
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter sensor_height: %f", sensor_height_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter sensor_model: %d", sensor_model_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter num_iter: %d", num_iter_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter num_lpr: %d", num_lpr_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter th_seeds: %f", th_seeds_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter th_dist: %f", th_dist_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter road_type: %d", road_type_);
    RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Parameter vis: %d", vis_);

    SplitString(str_range_, dis_range);
    SplitString(str_seg_distance_, seg_distances);

    has_point_clouds_ = false;
    g_not_ground_pc.reset(new pcl::PointCloud<PointType>());
    current_pc_ptr.reset(new pcl::PointCloud<PointType>());
    cloud_filtered.reset(new pcl::PointCloud<PointType>());
    skidpad_detection_pc.reset(new pcl::PointCloud<PointType>());

    // 初始化可复用的分段数组（大小将在运行时根据 seg_distances 自适应）
    cloud_segments_array_.clear();

    if (use_distortion_adjust_) {
        imu_sub_ptr_.reset(new ImuSubscriber(node_, imu_topic_, 100));
        disAdjust.reset(new DistortionAdjuster());
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Distortion adjustment enabled (imu_topic=%s, scan_period=%.3f)", imu_topic_.c_str(),
                 scan_period_);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[lidar_cluster] Distortion adjustment disabled");
    }
}

void LidarCluster::SingleFrameDedup(common_msgs::msg::HuatConeCluster &position) {
    int removed = ::SingleFrameDedup(position, single_frame_dedup_radius_);
    if (removed > 0) {
        RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] single-frame dedup: removed %d near-duplicate cones", removed);
    }
}

ScoringParams LidarCluster::MakeScoringParams() const {
    return {conf_penalty_aspect_,
            conf_penalty_over_max_accel_,
            conf_penalty_height_over_,
            conf_penalty_area_over_,
            conf_penalty_height_under_,
            conf_penalty_area_under_,
            conf_penalty_tilt_,
            min_height_,
            max_height_,
            min_area_,
            max_area_,
            max_tilt_angle_,
            road_type_};
}

double LidarCluster::GetConfidence(PointType max, PointType min, Eigen::Vector4f centroid,
                                   const pcl::PointCloud<PointType>::Ptr &cluster_cloud) {
    ScoringParams p = MakeScoringParams();
    return ComputeConfidence(max, min, centroid, cluster_cloud, p);
}

void LidarCluster::SplitString(const std::string &in_string, std::vector<double> &out_array) {
    lidar_cluster::parseCsvDoubles(in_string, out_array);
}

void LidarCluster::EuclideanClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
                                          std::vector<pcl::PointIndices> &cluster_indices,
                                          const double &max_cluster_dis) {
    if (inputcloud->points.size() == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "[lidar_cluster] Euclidean clustering received empty point cloud, exiting");
        return;
    }

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(inputcloud);
    pcl::EuclideanClusterExtraction<PointType> ec;  // 聚类对象
    ec.setClusterTolerance(max_cluster_dis);        // 设置邻近搜索的搜索半径为3cm
    ec.setMinClusterSize(2);                        // 设置一个聚类需要的最少点云数目为100  2
    ec.setMaxClusterSize(50);                       // 最多点云数目为20000        50
    ec.setSearchMethod(tree);                       // 设置点云的搜索机制
    ec.setInputCloud(inputcloud);                   // 设置原始点云
    ec.extract(cluster_indices);                    // 从点云中提取聚类
}

void LidarCluster::EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
                                                  std::vector<std::vector<pcl::PointIndices>> &cluster_indices) {
    size_t num_bins = seg_distances.size();
    if (num_bins == 0 || dis_range.size() + 1 != num_bins) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[lidar_cluster] Mismatched adaptive cluster config: dis_range=%zu, seg_distances=%zu",
                           dis_range.size(), num_bins);
        return;
    }

    // 确保可复用的分段数组大小正确
    if (cloud_segments_array_.size() != num_bins) {
        cloud_segments_array_.resize(num_bins);
        for (size_t i = 0; i < num_bins; i++) {
            if (!cloud_segments_array_[i]) {
                cloud_segments_array_[i].reset(new pcl::PointCloud<PointType>());
            }
        }
    }
    for (size_t i = 0; i < num_bins; i++) {
        cloud_segments_array_[i]->points.clear();
        cloud_segments_array_[i]->points.reserve(inputcloud->points.size());
    }

    // 预计算距离的平方阈值（避免每个点都计算 sqrt）
    std::vector<double> dis_range_sq(dis_range.size());
    for (size_t k = 0; k < dis_range.size(); k++) {
        dis_range_sq[k] = dis_range[k] * dis_range[k];
    }

    for (size_t i = 0; i < inputcloud->points.size(); i++) {
        const auto &p = inputcloud->points[i];
        float origin_dis_sq = p.x * p.x + p.y * p.y;  // 到原点的平方距离
        size_t bin = num_bins - 1;
        for (size_t r = 0; r < dis_range.size(); r++) {
            if (origin_dis_sq < dis_range_sq[r]) {
                bin = r;
                break;
            }
        }
        cloud_segments_array_[bin]->points.push_back(p);
    }

    std::vector<pcl::PointIndices> v;
    for (size_t i = 0; i < num_bins; i++) {
        v.clear();
        EuclideanClusterMethod(cloud_segments_array_[i], v, seg_distances[i]);
        cluster_indices.push_back(v);
    }
}

bool LidarCluster::ProcessCluster(const pcl::PointCloud<PointType>::Ptr &cloud_cluster,
                                  common_msgs::msg::HuatConeCluster &position,
                                  pcl::PointCloud<PointType>::Ptr &final_cluster) {
    Eigen::Vector4f centroid;
    PointType _min, _max;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    pcl::getMinMax3D(*cloud_cluster, _min, _max);

    double confidence = GetConfidence(_max, _min, centroid, cloud_cluster);

    if (confidence <= cone_confidence_threshold_) {
        if (vis_ && vis_init_status)
            PublishClusterMarker(_max, _min, euc, 0, 0, 0, false, confidence);
        return false;
    }

    *final_cluster += *cloud_cluster;

    geometry_msgs::msg::Point32 tmp, tmp1, tmp2;
    tmp.x = centroid[0];
    tmp.y = centroid[1];
    tmp.z = centroid[2];
    euc = float(sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]));
    position.points.push_back(tmp);
    position.confidence.push_back(static_cast<float>(std::max(0.0, std::min(confidence, 1.0))));
    position.obj_dist.push_back(euc);

    uint8_t cone_type = huat_cone::SIZE_UNKNOWN;
    if (enable_cone_type_classification_) {
        double c_height = std::fabs(_max.z - _min.z);
        double c_area = std::fabs(_max.x - _min.x) * std::fabs(_max.y - _min.y);
        if (c_height >= large_min_height_ && c_area >= large_min_area_)
            cone_type = huat_cone::SIZE_LARGE;
        else if (c_height <= small_max_height_ && c_area <= small_max_area_)
            cone_type = huat_cone::SIZE_SMALL;
    }
    position.type.push_back(cone_type);
    tmp1.x = _max.x;
    tmp1.y = _max.y;
    tmp1.z = _max.z;
    tmp2.x = _min.x;
    tmp2.y = _min.y;
    tmp2.z = _min.z;
    position.max_points.push_back(tmp1);
    position.min_points.push_back(tmp2);

    if (vis_ && vis_init_status)
        PublishClusterMarker(_max, _min, euc, 0, 0, 0, true, confidence);

    PointType tmp_pXYZ;
    tmp_pXYZ.x = centroid[0];
    tmp_pXYZ.y = centroid[1];
    tmp_pXYZ.z = centroid[2];
    skidpad_detection_pc->points.push_back(tmp_pXYZ);

    return true;
}

void LidarCluster::PublishDebugTopics() {
    pub_debug_clustered_->publish(pub_pc);
    marker_pub_->publish(marker_array);
    marker_pub_all_->publish(marker_array_all);
    pcl::PCLPointCloud2 pcl_pc2_skidpad; pcl::toPCLPointCloud2(*skidpad_detection_pc, pcl_pc2_skidpad); pcl_conversions::fromPCL(pcl_pc2_skidpad, pub_pc);
    pub_pc.header = scan_header_;
    skidpad_detection->publish(pub_pc);
}

void LidarCluster::ClusterMethod() {
    if (vis_)
        vis_init_status = InitializeVisualization();

    common_msgs::msg::HuatConeCluster position;
    skidpad_detection_pc->points.clear();

    std::vector<std::vector<pcl::PointIndices>> cluster_indices;
    EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices);

    pcl::PointCloud<PointType>::Ptr final_cluster(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
    for (int i = 0; i < (int)cluster_indices.size(); ++i) {
        for (const auto &seg : cluster_indices[i]) {
            cloud_cluster->clear();
            cloud_cluster->points.reserve(seg.indices.size());
            for (int idx : seg.indices)
                cloud_cluster->points.push_back(cloud_segments_array_[i]->points[idx]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            ProcessCluster(cloud_cluster, position, final_cluster);
        }
    }

    if (enable_single_frame_dedup_)
        SingleFrameDedup(position);

    if (enable_debug_) {
        pcl::PCLPointCloud2 pcl_pc2_final; pcl::toPCLPointCloud2(*final_cluster, pcl_pc2_final); pcl_conversions::fromPCL(pcl_pc2_final, pub_pc);
        pub_pc.header = scan_header_;
        PublishDebugTopics();
    }
    position.header = scan_header_;
    cone_pub_->publish(position);
}
