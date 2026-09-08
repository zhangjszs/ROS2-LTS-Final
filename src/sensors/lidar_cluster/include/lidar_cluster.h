#pragma once

#include <common_msgs/msg/huat_asensing.hpp>
#include <common_msgs/msg/huat_carstate.hpp>
#include <common_msgs/msg/huat_cone_cluster.hpp>
#include <common_msgs/msg/huat_ins_p2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ground_segmentation/ground_segmentation_strategy.h>
#include <lidar_cluster_scoring.h>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <preprocessor/point_cloud_preprocessor.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <distortion_adjuster.hpp>
#include <imu_subscriber.hpp>
#include <mutex>
#include <string>
#include <vector>

#include "profiler/frame_profiler.h"

using namespace lidar_distortion;
constexpr int kEightRing = 1;
constexpr int kLinearAcceleration = 2;
constexpr int kHighSpeedTracking = 3;
#include "point_type.h"

class LidarCluster {
   public:
    explicit LidarCluster(rclcpp::Node* node);
    explicit LidarCluster(rclcpp::Node& node) : LidarCluster(&node) {}
    explicit LidarCluster(const rclcpp::Node::SharedPtr& node) : LidarCluster(node.get()) {}
    ~LidarCluster();

    /**
     * @brief 主处理循环：地面分割 → 聚类 → 锥桶分类 → 发布。
     * 由 component 的 algoPoll 线程在收到新点云后调用。
     */
    void RunAlgorithm();
    void WaitForPendingCloud();
    void RequestStop();

   private:
    // ========== 节点 / 通信 ==========
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
    rclcpp::Subscription<common_msgs::msg::HuatInsP2>::SharedPtr sub_ins_;
    rclcpp::Subscription<common_msgs::msg::HuatASENSING>::SharedPtr sub_asensing_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr sub_vehicle_state_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_all_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_passthrough_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_clustered_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_ground_seg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_satisfied_;
    rclcpp::Publisher<common_msgs::msg::HuatConeCluster>::SharedPtr cone_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarClusterPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr adjust_check_front;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr adjust_check_back;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr skidpad_detection;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr logging_pub;

    // ========== 点云缓冲区 ==========
    pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
    pcl::PointCloud<PointType>::Ptr current_pc_ptr;
    pcl::PointCloud<PointType>::Ptr cloud_filtered;
    pcl::PointCloud<PointType>::Ptr skidpad_detection_pc;
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array_;

    // ========== 预分配复用点云缓冲区（消除 20Hz 热路径中的 new / free 堆分配） ==========
    pcl::PointCloud<PointType>::Ptr incoming_cloud_buf_;      // 接收线程专用的暂存缓冲区
    pcl::PointCloud<PointType>::Ptr ready_cloud_buf_;         // 互斥交换区，存放最新完整帧
    pcl::PointCloud<PointType>::Ptr processing_pc_;           // 算法处理线程专用的工作缓冲区
    pcl::PointCloud<PointType>::Ptr ground_pc_;               // 地面分割预分配缓冲区
    pcl::PointCloud<PointType>::Ptr cloud_cluster_;           // 单个聚类预分配缓冲区
    pcl::PointCloud<PointType>::Ptr final_cluster_;           // 最终聚类融合预分配缓冲区
    pcl::PointCloud<PointType>::Ptr distortion_adjusted_pc_;  // 畸变校正预分配缓冲区
    pcl::PointCloud<PointType>::Ptr accum_downsampled_pc_;    // 时序累积降采样预分配缓冲区

    // ========== 消息头 / 可视化 ==========
    sensor_msgs::msg::PointCloud2 out_pc;
    sensor_msgs::msg::PointCloud2 in_pc;
    std_msgs::msg::Header scan_header_;
    double scan_speed_ = 0.0;
    double scan_pitch_ = 0.0;
    bool scan_has_ins_ = false;
    sensor_msgs::msg::PointCloud2 pub_pc;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray marker_array_all;
    visualization_msgs::msg::Marker bbox_marker;
    visualization_msgs::msg::Marker euc_marker, intensity_max_marker, intensity_min_marker, intensity_mean_marker;

    // ========== 坐标变换 / 位姿 ==========
    nav_msgs::msg::Path ros_path_;
    geometry_msgs::msg::PoseStamped pose;
    Eigen::Matrix4f transMat_;
    Eigen::Matrix4f m;

    // ========== 模块 ==========
    std::unique_ptr<FrameProfiler> profiler_;
    std::unique_ptr<PointCloudPreprocessor> preprocessor_;
    std::unique_ptr<GroundSegmentationStrategy> ground_strategy_;
    std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
    std::shared_ptr<DistortionAdjuster> disAdjust;

    // ========== 线程同步 ==========
    std::mutex lidar_mutex;
    std::mutex acc_pose_mutex_;
    std::condition_variable cloud_cv_;
    std::atomic<bool> stop_algo_{false};

    // ========== 话题名 ==========
    std::string no_ground_topic_, ground_topic_, all_points_topic_, input_topic_;
    std::string output_cones_topic_, vehicle_state_topic_, ins_p2_topic_, ins_asensing_topic_;
    std::string profiling_topic_, debug_bounding_boxes_topic_, debug_bounding_boxes_all_topic_;
    std::string debug_passthrough_topic_, debug_clustered_topic_, debug_ground_seg_topic_, debug_satisfied_topic_;
    std::string debug_point_clusters_topic_, debug_adjust_front_topic_, debug_adjust_back_topic_;
    std::string debug_log_path_topic_, debug_skidpad_detection_topic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_, pub_no_ground_, pub_all_points_;
    std::string point_topic_;

    // ========== 状态标志 ==========
    bool has_point_clouds_ = false;
    bool pending_cloud_ = false;
    bool has_ins_p2_ = false;
    bool has_imu_ = false;
    bool enable_debug_ = false;
    bool use_distortion_adjust_ = false;
    bool vis_init_status = false;
    int no_data_frames_ = 0;

    // ========== 畸变校正 ==========
    float scan_period_ = 0.1f;
    std::string imu_topic_;
    std::deque<ImuData> unsynced_imu_;
    double cloud_time = 0.0;

    // ========== 车辆位姿 / INS ==========
    double current_pitch_ = 0.0;
    double current_speed_ = 0.0;
    int frame_count = 0;

    // ========== ROI / 预处理 ==========
    bool enable_dynamic_roi_ = false;
    int point_cloud_queue_size_ = 1;
    int marker_id_ = 0;
    int sensor_model_;
    double sensor_height_;
    int vis_;

    // ========== 锥桶置信度 / 尺寸门限 ==========
    double min_height_, max_height_, min_area_, max_area_, max_box_altitude_;
    double max_tilt_angle_;
    double cone_confidence_threshold_;

    // ========== 地面分割参数 ==========
    int num_seg_ = 1;
    int num_iter_, num_lpr_;
    double th_seeds_, th_dist_;
    int road_type_ = -1;

    // ========== 地面分割自动调参 (P2-12) ==========
    bool enable_ground_autotune_ = false;
    int dense_pc_threshold_ = 15000;
    double dense_th_dist_ = 0.025;
    double sparse_th_dist_ = 0.05;
    int dense_num_iter_ = 5;
    int sparse_num_iter_ = 3;

    // ========== 地面坡度补偿 (P3-3) ==========
    bool enable_ground_slope_compensation_ = false;
    double slope_th_seeds_scale_ = 0.002;
    double slope_pitch_max_ = 15.0;

    // ========== 帧率保护 (P3-4) ==========
    bool enable_frame_rate_protection_ = false;
    double frp_time_threshold_ms_ = 80.0;
    double frp_coarse_leaf_size_ = 0.08;
    bool frp_skip_sor_ = true;
    int frp_reduce_num_iter_ = 2;
    bool frp_active_ = false;
    double last_frame_total_ms_ = 0.0;

    // ========== 锥桶类型分类 (P3-5) ==========
    bool enable_cone_type_classification_ = false;
    double large_min_height_ = 0.25;
    double large_min_area_ = 0.12;
    double small_max_height_ = 0.20;
    double small_max_area_ = 0.08;

    // ========== 单帧近邻去重 (P4-1) ==========
    bool enable_single_frame_dedup_ = true;
    double single_frame_dedup_radius_ = 0.5;

    // ========== 置信度惩罚系数 ==========
    double conf_penalty_aspect_ = 2.0;
    double conf_penalty_over_max_accel_ = 0.05;
    double conf_penalty_height_over_ = 7.0;
    double conf_penalty_area_over_ = 2.0;
    double conf_penalty_height_under_ = 1.5;
    double conf_penalty_area_under_ = 2.0;
    double conf_penalty_tilt_ = 0.5;

    // ========== 聚类分段参数 ==========
    double euc;
    float intensity, intensity_min, intensity_max, intensity_mean;
    int intensity_count;
    std::string str_range_, str_seg_distance_;
    std::vector<double> dis_range;
    std::vector<double> seg_distances;
    geometry_msgs::msg::Point p;  // 复用于 bounding box 可视化

    // ========== 时序远距离累积 (P3-1) ==========
    bool enable_temporal_accumulation_ = false;
    int accumulation_frames_ = 3;
    double accumulation_min_distance_ = 15.0;
    double accumulation_voxel_size_ = 0.1;  // 累积后体素降采样叶大小 (P3-1b)
    struct AccumFrame {
        pcl::PointCloud<PointType>::Ptr cloud;
        double car_x = 0.0, car_y = 0.0, car_theta = 0.0;
    };
    std::deque<AccumFrame> accumulated_far_clouds_;

    // ========== 车辆位姿（供时序累积补偿） ==========
    double acc_car_x_ = 0.0, acc_car_y_ = 0.0, acc_car_theta_ = 0.0;

    // ========== 帧级性能分析 (P1-2 / P2-13) ==========
    bool enable_profiling_ = false;

    // ------------------------------------------------------------------
    // 私有方法：初始化
    // ------------------------------------------------------------------
    void Init();
    void LoadParams();
    void LoadTopicParams();
    void LoadGroundParams();
    void LoadFeatureParams();
    void InitStrategy();
    void InitState();
    bool InitializeVisualization();

    // ------------------------------------------------------------------
    // 私有方法：回调
    // ------------------------------------------------------------------
    void OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_cloud);
    void OnVehicleState(const common_msgs::msg::HuatCarstate::ConstSharedPtr msg);
    void OnInsP2(const common_msgs::msg::HuatInsP2::ConstSharedPtr msg);
    void OnAsensing(const common_msgs::msg::HuatASENSING::ConstSharedPtr msg);

    // ------------------------------------------------------------------
    // 私有方法：聚类流水线
    // ------------------------------------------------------------------
    void ClusterMethod();
    void EuclideanClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
                                std::vector<pcl::PointIndices> &cluster_indices, const double &max_cluster_dis);
    void EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
                                        std::vector<std::vector<pcl::PointIndices>> &cluster_indices);
    bool ProcessCluster(const pcl::PointCloud<PointType>::Ptr &cloud_cluster, common_msgs::msg::HuatConeCluster &position,
                        pcl::PointCloud<PointType>::Ptr &final_cluster);
    void SingleFrameDedup(common_msgs::msg::HuatConeCluster &position);

    // ------------------------------------------------------------------
    // 私有方法：地面分割
    // ------------------------------------------------------------------
    void ComputeGroundSegParams(GroundSegParams &params);
    ScoringParams MakeScoringParams() const;

    // ------------------------------------------------------------------
    // 私有方法：畸变校正
    // ------------------------------------------------------------------
    void ApplyDistortionAdjustment(pcl::PointCloud<PointType>::Ptr &cloud);

    // ------------------------------------------------------------------
    // 私有方法：时序累积
    // ------------------------------------------------------------------
    void AccumulateTemporalFrames();

    // ------------------------------------------------------------------
    // 私有方法：可视化 / 调试发布
    // ------------------------------------------------------------------
    void PublishClusterMarker(const PointType max, const PointType min, float euc, float intensity_max,
                              float intensity_min, float intensity_mean, bool type, float conf);
    void PublishDebugTopics();
    void publishToCheckAdjust(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out);

    // ------------------------------------------------------------------
    // 私有方法：工具
    // ------------------------------------------------------------------
    void SplitString(const std::string &in_string, std::vector<double> &out_array);
    double GetConfidence(PointType max, PointType min, Eigen::Vector4f centroid,
                         const pcl::PointCloud<PointType>::Ptr &cluster_cloud);
};
