#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <atomic>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_cone.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "std_msgs/msg/string.hpp"

enum class TrackState { TENTATIVE, CONFIRMED };

struct MatchDiagStats {
    int matched = 0;
    int insert_suppressed = 0;
    int removed_stale = 0;
    int new_cones = 0;
    int culled_input = 0;
    double match_dist_min = -1.0;
    double match_dist_max = -1.0;
    double match_dist_sum = 0.0;
    int match_dist_n = 0;
    double pos_jump_sum = 0.0;
    double pos_jump_max = 0.0;
    int pos_jump_n = 0;
    double unmatched_dist_min = std::numeric_limits<double>::infinity();

    void record_match(double dist) {
        if (match_dist_n++ == 0)
            match_dist_min = match_dist_max = dist;
        else {
            match_dist_min = std::min(match_dist_min, dist);
            match_dist_max = std::max(match_dist_max, dist);
        }
        match_dist_sum += dist;
        pos_jump_sum += dist;
        pos_jump_n++;
        if (dist > pos_jump_max)
            pos_jump_max = dist;
    }

    double match_dist_avg() const {
        return match_dist_n > 0 ? match_dist_sum / match_dist_n : -1.0;
    }
    double pos_jump_avg() const {
        return pos_jump_n > 0 ? pos_jump_sum / pos_jump_n : 0.0;
    }
};

struct TrackedCone {
    common_msgs::msg::HuatCone cone;
    // 与 CreateTrackedCone 的初始赋值一致（F2-P2-04）
    int tracked_frames = 1;
    int missed_frames = 0;
    int consecutive_matches = 0;
    TrackState state = TrackState::TENTATIVE;
    // P3-2: 卡尔曼滤波器状态（恒定速度模型：x, y, vx, vy）
    Eigen::Vector4f kf_state = Eigen::Vector4f::Zero();
    Eigen::Matrix4f kf_P = Eigen::Matrix4f::Identity();
    bool has_kalman = false;
    rclcpp::Time last_update;
};

class ConeDedup {
   public:
    explicit ConeDedup(rclcpp::Node::SharedPtr node);

    static constexpr int kMaxMatchNeighbors = 5;  // K for nearestKSearch in matching

   private:
    void OnTransformedCone(const common_msgs::msg::HuatMap::ConstSharedPtr msgs);
    void OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr msgs);
    int GetNewId();
    void RebuildKdTree();
    double ComputeDynamicAlpha() const;

    // P3-2: 卡尔曼滤波器辅助函数
    void KalmanInit(TrackedCone &tc, double x, double y);
    void KalmanPredict(TrackedCone &tc, double dt);
    void KalmanUpdate(TrackedCone &tc, double x, double y);

    TrackedCone CreateTrackedCone(const common_msgs::msg::HuatCone &cone, const rclcpp::Time &stamp);

    void ApplyPositionUpdate(TrackedCone &tc, double obs_x, double obs_y, double obs_z,
                             const common_msgs::msg::HuatCone &src, const rclcpp::Time &stamp, double alpha);
    std::vector<size_t> FilterInputByDistance(const std::vector<common_msgs::msg::HuatCone> &cones,
                                              int &culled_count) const;
    std::vector<std::vector<double>> BuildCostMatrix(const std::vector<size_t> &valid_idx,
                                                     const std::vector<common_msgs::msg::HuatCone> &cones, int n_tracks,
                                                     double inf_cost) const;
    void UpdateUnmatchedExistingTracks(const std::vector<bool> &matched_existing, size_t original_size,
                                       const rclcpp::Time &stamp);
    int RemoveStaleTracks();
    void CollectConfirmedCones(common_msgs::msg::HuatMap &out) const;
    void ProcessUnmatchedInputs(const std::vector<size_t> &valid_input_idx,
                                const std::vector<common_msgs::msg::HuatCone> &cones, std::vector<bool> &matched_input,
                                std::vector<bool> &matched_existing, const rclcpp::Time &stamp, double alpha,
                                double radius_sq, MatchDiagStats &stats);
    void ProcessMatchedPairs(const std::vector<size_t> &valid_input_idx,
                             const std::vector<common_msgs::msg::HuatCone> &cones, const std::vector<int> &assignment,
                             const std::vector<std::vector<double>> &cost_mat, std::vector<bool> &matched_existing,
                             std::vector<bool> &matched_input, const rclcpp::Time &stamp, double alpha,
                             MatchDiagStats &stats);
    void PublishStatus(size_t input_size, const MatchDiagStats &stats, size_t published_count);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr transformed_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr car_state_sub_;
    rclcpp::Publisher<common_msgs::msg::HuatMap>::SharedPtr fused_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::vector<TrackedCone> tracked_cones_;
    std::mutex mtx_;
    bool initialized_ = false;
    // 有锥桶增删时设为 true，RebuildKdTree 全量重建后重置为 false
    bool kdtree_needs_full_rebuild_ = true;

    double cone_match_radius_ = 1.0;
    double ema_alpha_ = 0.3;
    int max_miss_frames_ = 3;
    int min_track_frames_ = 1;
    int confirmation_frames_ = 3;
    int max_miss_frames_confirmed_ = 6;

    // 自适应 EMA 参数
    bool enable_adaptive_alpha_ = false;
    double alpha_min_ = 0.1;
    double alpha_max_ = 0.5;
    double speed_ref_ = 15.0;  // alpha 达到 alpha_max 时的速度（m/s）
    double current_speed_ = 0.0;
    double car_x_ = 0.0;
    double car_y_ = 0.0;
    double car_theta_ = 0.0;
    bool has_car_state_ = false;

    // P2-11: 局部滑动窗口地图融合
    bool enable_sliding_window_ = false;
    double sliding_window_radius_ = 30.0;
    double sliding_window_rear_radius_ = 30.0;

    // P3-2: 卡尔曼滤波器跟踪
    bool enable_kalman_ = false;
    double kf_q_ = 0.01;  // 过程噪声
    double kf_r_ = 0.1;   // 测量噪声

    // P1-B: 自运动补偿
    // 车辆运动时 INS 误差会使静止锥桶呈现虚假速度，补偿后减少高速转弯时的预测漂移
    bool enable_ego_motion_compensation_ = false;
    double kf_ego_correction_factor_ = 0.5;  // 补偿系数 [0,1]，越大越激进
    double kf_vel_decay_rate_ = 1.0;         // 速度衰减率 (1/s)，防止速度估计累积
    double car_vx_ = 0.0;                    // 车辆全局速度 x（由 OnCarState 更新）
    double car_vy_ = 0.0;
    double prev_car_x_ = 0.0;
    double prev_car_y_ = 0.0;
    rclcpp::Time prev_car_time_;

    // P4-2: 新增锥桶近邻去重
    bool enable_insert_dedup_ = true;

    double max_tracking_distance_ = 20.0;
    int diag_culled_track_count_ = 0;
};
