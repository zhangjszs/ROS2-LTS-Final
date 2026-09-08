#include "cone_dedup.h"

#include <algorithm>
#include <limits>
#include <span>

#include "cone_dedup_algo.h"

ConeDedup::ConeDedup(rclcpp::Node::SharedPtr node) : node_(node) {
    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    node_->declare_parameter("cone_match_radius", 1.0);
    node_->declare_parameter("ema_alpha", 0.3);
    node_->declare_parameter("max_miss_frames", 3);
    node_->declare_parameter("min_track_frames", 1);
    node_->declare_parameter("enable_adaptive_alpha", false);
    node_->declare_parameter("alpha_min", 0.1);
    node_->declare_parameter("alpha_max", 0.5);
    node_->declare_parameter("speed_ref", 15.0);
    node_->declare_parameter("enable_sliding_window", false);
    node_->declare_parameter("sliding_window_radius", 30.0);
    node_->declare_parameter("enable_kalman", false);
    node_->declare_parameter("kf_q", 0.01);
    node_->declare_parameter("kf_r", 0.1);
    node_->declare_parameter("enable_ego_motion_compensation", false);
    node_->declare_parameter("kf_ego_correction_factor", 0.5);
    node_->declare_parameter("kf_vel_decay_rate", 1.0);
    node_->declare_parameter("enable_insert_dedup", true);
    node_->declare_parameter("max_tracking_distance", 20.0);
    node_->declare_parameter("confirmation_frames", 3);
    node_->declare_parameter("max_miss_frames_confirmed", 6);

    node_->get_parameter("cone_match_radius", cone_match_radius_);
    node_->get_parameter("ema_alpha", ema_alpha_);
    node_->get_parameter("max_miss_frames", max_miss_frames_);
    node_->get_parameter("min_track_frames", min_track_frames_);
    node_->get_parameter("enable_adaptive_alpha", enable_adaptive_alpha_);
    node_->get_parameter("alpha_min", alpha_min_);
    node_->get_parameter("alpha_max", alpha_max_);
    node_->get_parameter("speed_ref", speed_ref_);
    node_->get_parameter("enable_sliding_window", enable_sliding_window_);
    node_->get_parameter("sliding_window_radius", sliding_window_radius_);

    node_->get_parameter("enable_kalman", enable_kalman_);
    node_->get_parameter("kf_q", kf_q_);
    node_->get_parameter("kf_r", kf_r_);
    node_->get_parameter("enable_ego_motion_compensation", enable_ego_motion_compensation_);
    node_->get_parameter("kf_ego_correction_factor", kf_ego_correction_factor_);
    node_->get_parameter("kf_vel_decay_rate", kf_vel_decay_rate_);

    node_->get_parameter("enable_insert_dedup", enable_insert_dedup_);
    node_->get_parameter("max_tracking_distance", max_tracking_distance_);
    node_->get_parameter("confirmation_frames", confirmation_frames_);
    node_->get_parameter("max_miss_frames_confirmed", max_miss_frames_confirmed_);

    std::string transformed_cones_topic;
    std::string vehicle_state_topic;
    std::string fused_cones_topic;
    std::string status_topic;
    node_->declare_parameter("transformed_cones_topic", std::string("/sensors/cones/transformed"));
    node_->declare_parameter("vehicle_state_topic", std::string("/localization/vehicle_state"));
    node_->declare_parameter("fused_cones_topic", std::string("/sensors/cones/fused"));
    node_->declare_parameter("status_topic", std::string("/debug/cone_tracker/status"));
    node_->get_parameter("transformed_cones_topic", transformed_cones_topic);
    node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
    node_->get_parameter("fused_cones_topic", fused_cones_topic);
    node_->get_parameter("status_topic", status_topic);

    transformed_sub_ = node_->create_subscription<common_msgs::msg::HuatMap>(
        transformed_cones_topic, 1,
        [this](common_msgs::msg::HuatMap::ConstSharedPtr msg) { OnTransformedCone(msg); });
    car_state_sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
        vehicle_state_topic, 1,
        [this](common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnCarState(msg); });
    fused_pub_ = node_->create_publisher<common_msgs::msg::HuatMap>(fused_cones_topic, 10);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic, 10);

    RCLCPP_INFO(
        node_->get_logger(),
        "[cone_dedup] Started (match_radius=%.2f, ema_alpha=%.2f, max_miss=%d, min_track=%d, adaptive_alpha=%s, "
        "sliding_window=%s, radius=%.1f, kalman=%s, confirm=%d, max_miss_confirmed=%d, max_dist=%.1f, "
        "in=%s, state=%s, out=%s, status=%s)",
        cone_match_radius_, ema_alpha_, max_miss_frames_, min_track_frames_, enable_adaptive_alpha_ ? "true" : "false",
        enable_sliding_window_ ? "true" : "false", sliding_window_radius_, enable_kalman_ ? "true" : "false",
        confirmation_frames_, max_miss_frames_confirmed_, max_tracking_distance_, transformed_cones_topic.c_str(),
        vehicle_state_topic.c_str(), fused_cones_topic.c_str(), status_topic.c_str());
}

void ConeDedup::OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr msgs) {
    current_speed_ = std::fabs(static_cast<double>(msgs->v));

    // 用相邻帧位置差分估计全局速度，供自运动补偿使用
    if (enable_ego_motion_compensation_ && has_car_state_) {
        double dt_ego = (rclcpp::Time(msgs->header.stamp) - prev_car_time_).seconds();
        if (dt_ego > 0.005 && dt_ego < 0.5) {
            car_vx_ = (msgs->car_state.x - prev_car_x_) / dt_ego;
            car_vy_ = (msgs->car_state.y - prev_car_y_) / dt_ego;
        }
    }
    prev_car_x_ = car_x_;
    prev_car_y_ = car_y_;
    prev_car_time_ = msgs->header.stamp;

    car_x_ = msgs->car_state.x;
    car_y_ = msgs->car_state.y;
    car_theta_ = msgs->car_state.theta;
    has_car_state_ = true;
}

double ConeDedup::ComputeDynamicAlpha() const {
    if (!enable_adaptive_alpha_) {
        return ema_alpha_;
    }
    return cone_dedup_algo::ComputeDynamicAlpha(current_speed_, speed_ref_, alpha_min_, alpha_max_, ema_alpha_);
}

void ConeDedup::KalmanInit(TrackedCone &tc, double x, double y) {
    tc.kf_state << static_cast<float>(x), static_cast<float>(y), 0.0f, 0.0f;
    tc.kf_P = Eigen::Matrix4f::Identity() * 1.0f;
    tc.has_kalman = true;
}

void ConeDedup::KalmanPredict(TrackedCone &tc, double dt) {
    if (!tc.has_kalman || dt <= 0.0)
        return;
    float dt_f = static_cast<float>(dt);
    // 状态转移矩阵（恒定速度模型）
    Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
    F(0, 2) = dt_f;
    F(1, 3) = dt_f;
    // 过程噪声协方差 Q
    float q = static_cast<float>(kf_q_);
    Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
    Q(0, 0) = q;
    Q(1, 1) = q;
    Q(2, 2) = q * 0.1f;
    Q(3, 3) = q * 0.1f;
    // Predict
    tc.kf_state = F * tc.kf_state;
    tc.kf_P = F * tc.kf_P * F.transpose() + Q;

    // 自运动补偿：锥桶静止，速度状态因 INS 误差积累的虚假分量应被抑制。
    // 1. 将 track 的速度估计向 0 衰减（防止速度漂移积累）
    // 2. 用车辆速度加权修正位置预测（补偿坐标系漂移）
    if (enable_ego_motion_compensation_) {
        float decay = static_cast<float>(std::exp(-kf_vel_decay_rate_ * dt));
        tc.kf_state(2) *= decay;
        tc.kf_state(3) *= decay;

        float factor = static_cast<float>(kf_ego_correction_factor_);
        tc.kf_state(0) -= factor * static_cast<float>(car_vx_) * dt_f;
        tc.kf_state(1) -= factor * static_cast<float>(car_vy_) * dt_f;
    }
}

void ConeDedup::KalmanUpdate(TrackedCone &tc, double x, double y) {
    if (!tc.has_kalman)
        return;
    float mx = static_cast<float>(x);
    float my = static_cast<float>(y);
    // 测量矩阵 H（观测 x, y）
    Eigen::Matrix<float, 2, 4> H;
    H << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
    // 测量噪声 R
    float r = static_cast<float>(kf_r_);
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * r;
    // 新息（Innovation）
    Eigen::Vector2f z(mx, my);
    Eigen::Vector2f y_residual = z - H * tc.kf_state;
    Eigen::Matrix2f S = H * tc.kf_P * H.transpose() + R;
    // 卡尔曼增益
    Eigen::Matrix<float, 4, 2> K = tc.kf_P * H.transpose() * S.inverse();
    // 更新状态和协方差
    tc.kf_state = tc.kf_state + K * y_residual;
    tc.kf_P = (Eigen::Matrix4f::Identity() - K * H) * tc.kf_P;
}

void ConeDedup::RebuildKdTree() {
    if (kdtree_needs_full_rebuild_) {
        // 有增删：全量重建，重新分配 cloud_（点序与 tracked_cones_ 一致，索引可直接回查）
        cloud_->points.clear();
        cloud_->points.reserve(tracked_cones_.size());
        for (const auto &tc : tracked_cones_) {
            pcl::PointXYZ pt;
            pt.x = tc.cone.position_global.x;
            pt.y = tc.cone.position_global.y;
            pt.z = 0.0;
            cloud_->points.push_back(pt);
        }
        cloud_->width = cloud_->points.size();
        cloud_->height = 1;
        kdtree_needs_full_rebuild_ = false;
    } else {
        // 仅位置更新（EMA/卡尔曼）：原地同步坐标，跳过内存重分配
        for (size_t i = 0; i < tracked_cones_.size(); ++i) {
            cloud_->points[i].x = tracked_cones_[i].cone.position_global.x;
            cloud_->points[i].y = tracked_cones_[i].cone.position_global.y;
        }
    }
    if (!cloud_->empty()) {
        kdtree_.setInputCloud(cloud_);
    }
}

void ConeDedup::ApplyPositionUpdate(TrackedCone &tc, double obs_x, double obs_y, double obs_z,
                                    const common_msgs::msg::HuatCone &src, const rclcpp::Time &stamp, double alpha) {
    double dt = (stamp - tc.last_update).seconds();
    if (enable_kalman_ && tc.has_kalman && dt > 0.0) {
        KalmanPredict(tc, dt);
        KalmanUpdate(tc, obs_x, obs_y);
        tc.cone.position_global.x = tc.kf_state(0);
        tc.cone.position_global.y = tc.kf_state(1);
    } else {
        tc.cone.position_global.x = alpha * obs_x + (1.0 - alpha) * tc.cone.position_global.x;
        tc.cone.position_global.y = alpha * obs_y + (1.0 - alpha) * tc.cone.position_global.y;
    }
    tc.cone.position_global.z = alpha * obs_z + (1.0 - alpha) * tc.cone.position_global.z;
    tc.cone.header = src.header;
    tc.cone.position_base_link = src.position_base_link;
    tc.cone.confidence = src.confidence;
    tc.cone.type = src.type;
    tc.tracked_frames++;
    tc.missed_frames = 0;
    tc.consecutive_matches++;
    if (tc.state == TrackState::TENTATIVE && tc.consecutive_matches >= confirmation_frames_)
        tc.state = TrackState::CONFIRMED;
    tc.last_update = stamp;
}

std::vector<size_t> ConeDedup::FilterInputByDistance(std::span<const common_msgs::msg::HuatCone> cones,
                                                     int &culled_count) const {
    if (!has_car_state_) {
        std::vector<size_t> valid(cones.size());
        for (size_t i = 0; i < cones.size(); ++i)
            valid[i] = i;
        culled_count = 0;
        return valid;
    }
    std::vector<size_t> valid;
    valid.reserve(cones.size());
    const double max_dist_sq = max_tracking_distance_ * max_tracking_distance_;
    for (size_t i = 0; i < cones.size(); ++i) {
        const double dx = cones[i].position_global.x - car_x_;
        const double dy = cones[i].position_global.y - car_y_;
        if (dx * dx + dy * dy <= max_dist_sq) {
            valid.push_back(i);
        }
    }
    culled_count = static_cast<int>(cones.size() - valid.size());
    return valid;
}

cone_dedup_algo::FlatMatrix<double> ConeDedup::BuildCostMatrix(std::span<const size_t> valid_idx,
                                                            std::span<const common_msgs::msg::HuatCone> cones,
                                                            int n_tracks, double inf_cost) const {
    const int n_in = static_cast<int>(valid_idx.size());
    const double radius_sq = cone_match_radius_ * cone_match_radius_;
    cone_dedup_algo::FlatMatrix<double> cost_mat(static_cast<size_t>(n_in), static_cast<size_t>(n_tracks), inf_cost);
    if (cloud_->empty() || n_in == 0 || n_tracks == 0)
        return cost_mat;

    std::vector<int> point_idx(kMaxMatchNeighbors);
    std::vector<float> point_dist(kMaxMatchNeighbors);
    for (int r = 0; r < n_in; ++r) {
        size_t i = valid_idx[static_cast<size_t>(r)];
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(cones[i].position_global.x);
        pt.y = static_cast<float>(cones[i].position_global.y);
        pt.z = 0.0f;
        int found = kdtree_.nearestKSearch(pt, kMaxMatchNeighbors, point_idx, point_dist);
        for (int k = 0; k < found; ++k) {
            if (static_cast<double>(point_dist[static_cast<size_t>(k)]) > radius_sq)
                break;
            int tidx = point_idx[static_cast<size_t>(k)];
            if (tidx < n_tracks)
                cost_mat(static_cast<size_t>(r), static_cast<size_t>(tidx)) =
                    static_cast<double>(point_dist[static_cast<size_t>(k)]);
        }
    }
    return cost_mat;
}

void ConeDedup::ProcessUnmatchedInputs(std::span<const size_t> valid_input_idx,
                                       std::span<const common_msgs::msg::HuatCone> cones,
                                       std::vector<bool> &matched_input, std::vector<bool> &matched_existing,
                                       const rclcpp::Time &stamp, double alpha, double radius_sq, MatchDiagStats &stats) {
    int anomaly_log_count = 0;
    std::vector<int> idx(kMaxMatchNeighbors);
    std::vector<float> dist(kMaxMatchNeighbors);
    for (size_t i : valid_input_idx) {
        if (matched_input[i])
            continue;
        bool suppressed = false;
        if (!cloud_->empty()) {
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(cones[i].position_global.x);
            pt.y = static_cast<float>(cones[i].position_global.y);
            pt.z = 0.0f;
            int found = kdtree_.nearestKSearch(pt, kMaxMatchNeighbors, idx, dist);
            if (found > 0) {
                double d0 = std::sqrt(static_cast<double>(dist[0]));
                if (d0 < stats.unmatched_dist_min)
                    stats.unmatched_dist_min = d0;
                if (d0 > 3.0 && anomaly_log_count < 2 && idx[0] < static_cast<int>(tracked_cones_.size())) {
                    int track_id = tracked_cones_[idx[0]].cone.id;
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[Tracker Anomaly] ts=%.3f obs=(%.2f,%.2f) nearest_track_id=%d track=(%.2f,%.2f) dist=%.2fm",
                        stamp.nanoseconds() / 1e9, cones[i].position_global.x, cones[i].position_global.y, track_id,
                        tracked_cones_[idx[0]].cone.position_global.x, tracked_cones_[idx[0]].cone.position_global.y,
                        d0);
                    anomaly_log_count++;
                }
            }
            if (enable_insert_dedup_) {
                for (int k = 0; k < found && !suppressed; k++) {
                    if (dist[k] > radius_sq)
                        break;
                    if (idx[k] >= static_cast<int>(matched_existing.size()))
                        continue;
                    if (matched_existing[idx[k]])
                        continue;
                    ApplyPositionUpdate(tracked_cones_[idx[k]], cones[i].position_global.x, cones[i].position_global.y,
                                        cones[i].position_global.z, cones[i], stamp, alpha);
                    matched_existing[idx[k]] = true;
                    suppressed = true;
                    stats.insert_suppressed++;
                }
            }
        }
        if (!suppressed) {
            tracked_cones_.push_back(CreateTrackedCone(cones[i], stamp));
            stats.new_cones++;
            kdtree_needs_full_rebuild_ = true;
        }
    }
    if (stats.insert_suppressed > 0)
        RCLCPP_INFO(node_->get_logger(), "[cone_dedup] insert dedup: suppressed %d near-duplicate cones", stats.insert_suppressed);
}

void ConeDedup::UpdateUnmatchedExistingTracks(const std::vector<bool> &matched_existing, size_t original_size,
                                              const rclcpp::Time &stamp) {
    for (size_t i = 0; i < original_size; i++) {
        if (matched_existing[i])
            continue;
        tracked_cones_[i].missed_frames++;
        if (tracked_cones_[i].consecutive_matches > 0)
            tracked_cones_[i].consecutive_matches--;
        if (enable_kalman_ && tracked_cones_[i].has_kalman) {
            double dt = (stamp - tracked_cones_[i].last_update).seconds();
            if (dt > 0.0) {
                KalmanPredict(tracked_cones_[i], dt);
                tracked_cones_[i].cone.position_global.x = tracked_cones_[i].kf_state(0);
                tracked_cones_[i].cone.position_global.y = tracked_cones_[i].kf_state(1);
                tracked_cones_[i].last_update = stamp;
            }
        }
    }
}

int ConeDedup::RemoveStaleTracks() {
    int removed = 0;
    auto it = tracked_cones_.begin();
    while (it != tracked_cones_.end()) {
        bool should_remove = false;
        if (has_car_state_) {
            double dx = it->cone.position_global.x - car_x_;
            double dy = it->cone.position_global.y - car_y_;
            if (dx * dx + dy * dy > max_tracking_distance_ * max_tracking_distance_) {
                should_remove = true;
                diag_culled_track_count_++;
            }
        }
        if (!should_remove) {
            int miss_limit = (it->state == TrackState::CONFIRMED) ? max_miss_frames_confirmed_ : max_miss_frames_;
            if (it->missed_frames > miss_limit) {
                should_remove = true;
                removed++;
            }
        }
        if (should_remove) {
            it = tracked_cones_.erase(it);
            kdtree_needs_full_rebuild_ = true;
        } else {
            ++it;
        }
    }
    return removed;
}

void ConeDedup::CollectConfirmedCones(common_msgs::msg::HuatMap &out) const {
    for (const auto &tc : tracked_cones_) {
        if (tc.state != TrackState::CONFIRMED || tc.tracked_frames < min_track_frames_)
            continue;
        if (enable_sliding_window_ && has_car_state_) {
            double dx = tc.cone.position_global.x - car_x_;
            double dy = tc.cone.position_global.y - car_y_;
            double longitudinal = dx * std::cos(car_theta_) + dy * std::sin(car_theta_);
            double radius = (longitudinal < 0.0) ? sliding_window_rear_radius_ : sliding_window_radius_;
            if (dx * dx + dy * dy > radius * radius)
                continue;
        }
        out.cone.push_back(tc.cone);
    }
}

void ConeDedup::ProcessMatchedPairs(std::span<const size_t> valid_input_idx,
                                    std::span<const common_msgs::msg::HuatCone> cones,
                                    std::span<const int> assignment,
                                    cone_dedup_algo::MatrixView<const double> cost_mat,
                                    std::vector<bool> &matched_existing, std::vector<bool> &matched_input,
                                    const rclcpp::Time &stamp, double alpha, MatchDiagStats &stats) {
    int n_in = static_cast<int>(valid_input_idx.size());
    for (int r = 0; r < n_in; ++r) {
        int tidx = assignment[static_cast<size_t>(r)];
        if (tidx < 0)
            continue;
        size_t i = valid_input_idx[static_cast<size_t>(r)];
        matched_existing[static_cast<size_t>(tidx)] = true;
        matched_input[i] = true;
        stats.record_match(std::sqrt(cost_mat(static_cast<size_t>(r), static_cast<size_t>(tidx))));
        ApplyPositionUpdate(tracked_cones_[static_cast<size_t>(tidx)], cones[i].position_global.x, cones[i].position_global.y,
                            cones[i].position_global.z, cones[i], stamp, alpha);
    }
    stats.matched = static_cast<int>(std::count(matched_input.begin(), matched_input.end(), true));
    RCLCPP_DEBUG(node_->get_logger(), "[cone_dedup] match stats: input=%zu, matched=%d, unmatched=%zu, tracked=%zu", cones.size(),
              stats.matched, cones.size() - stats.matched, tracked_cones_.size());
}

void ConeDedup::PublishStatus(size_t input_size, const MatchDiagStats &s, size_t published_count) {
    double avg_confidence = 0.0;
    int tentative_count = 0, confirmed_count = 0;
    if (!tracked_cones_.empty()) {
        double sum_conf = 0.0;
        for (const auto &tc : tracked_cones_) {
            sum_conf += tc.cone.confidence;
            if (tc.state == TrackState::CONFIRMED)
                confirmed_count++;
            else
                tentative_count++;
        }
        avg_confidence = sum_conf / tracked_cones_.size();
    }
    bool has_match = (s.match_dist_avg() >= 0.0);
    std_msgs::msg::String status;
    status.data =
        "tracked=" + std::to_string(tracked_cones_.size()) + ", new=" + std::to_string(s.new_cones) +
        ", avg_confidence=" + std::to_string(avg_confidence) + ", input=" + std::to_string(input_size) +
        ", matched=" + std::to_string(s.matched) +
        ", unmatched=" + std::to_string(static_cast<int>(input_size) - s.matched) +
        ", insert_suppressed=" + std::to_string(s.insert_suppressed) +
        ", removed_stale=" + std::to_string(s.removed_stale) + ", published=" + std::to_string(published_count) +
        ", match_dist_min=" + (has_match ? std::to_string(s.match_dist_min) : "-1") +
        ", match_dist_avg=" + (has_match ? std::to_string(s.match_dist_avg()) : "-1") +
        ", match_dist_max=" + (has_match ? std::to_string(s.match_dist_max) : "-1") +
        ", pos_jump_avg=" + std::to_string(s.pos_jump_avg()) + ", pos_jump_max=" + std::to_string(s.pos_jump_max) +
        ", unmatched_dist_min=" + (std::isinf(s.unmatched_dist_min) ? "-1" : std::to_string(s.unmatched_dist_min)) +
        ", culled_input=" + std::to_string(s.culled_input) +
        ", culled_track=" + std::to_string(diag_culled_track_count_) +
        ", tentative=" + std::to_string(tentative_count) + ", confirmed=" + std::to_string(confirmed_count);
    diag_culled_track_count_ = 0;
    status_pub_->publish(status);
}

void ConeDedup::OnTransformedCone(const common_msgs::msg::HuatMap::ConstSharedPtr msgs) {
    common_msgs::msg::HuatMap fused_map;
    fused_map.header = msgs->header;

    double dynamic_alpha = ComputeDynamicAlpha();
    MatchDiagStats stats;

    {
        std::scoped_lock lock(mtx_);

        if (msgs->cone.empty()) {
            for (auto &tc : tracked_cones_) {
                tc.missed_frames++;
                if (tc.consecutive_matches > 0)
                    tc.consecutive_matches--;
                if (enable_kalman_ && tc.has_kalman) {
                    double dt = (rclcpp::Time(msgs->header.stamp) - tc.last_update).seconds();
                    if (dt > 0.0) {
                        KalmanPredict(tc, dt);
                        tc.cone.position_global.x = tc.kf_state(0);
                        tc.cone.position_global.y = tc.kf_state(1);
                        tc.last_update = msgs->header.stamp;
                    }
                }
            }
        } else if (!initialized_) {
            initialized_ = true;
            stats.new_cones = static_cast<int>(msgs->cone.size());
            for (const auto &c : msgs->cone) {
                TrackedCone tc = CreateTrackedCone(c, msgs->header.stamp);
                // F-P1-03: 首帧锥桶直接满足确认条件（CONFIRMED + tracked_frames >= min_track_frames），
                // 避免起步时 /sensors/cones/fused 首帧为空导致规划器收不到锥桶。
                // 后续帧新增插入的锥桶仍走正常 TENTATIVE → CONFIRMED 确认流程。
                tc.tracked_frames = std::max(tc.tracked_frames, min_track_frames_);
                tc.consecutive_matches = std::max(tc.consecutive_matches, confirmation_frames_);
                tc.state = TrackState::CONFIRMED;
                tracked_cones_.push_back(tc);
            }
            kdtree_needs_full_rebuild_ = true;
        } else {
            size_t original_tracked_size = tracked_cones_.size();
            std::vector<bool> matched_existing(original_tracked_size, false);
            std::vector<bool> matched_input(msgs->cone.size(), false);
            const double inf_cost = 1e9;
            // 平方匹配半径：与 nearestKSearch 返回的平方距离直接比较（F2-P2-03）
            const double radius_sq = cone_match_radius_ * cone_match_radius_;

            std::vector<size_t> valid_input_idx = FilterInputByDistance(msgs->cone, stats.culled_input);
            auto cost_mat =
                BuildCostMatrix(valid_input_idx, msgs->cone, static_cast<int>(original_tracked_size), inf_cost);
            std::vector<int> assignment = cone_dedup_algo::HungarianAssign(cost_mat.view(), inf_cost);

            ProcessMatchedPairs(valid_input_idx, msgs->cone, assignment, cost_mat.view(), matched_existing, matched_input,
                                msgs->header.stamp, dynamic_alpha, stats);
            ProcessUnmatchedInputs(valid_input_idx, msgs->cone, matched_input, matched_existing, msgs->header.stamp,
                                   dynamic_alpha, radius_sq, stats);
            UpdateUnmatchedExistingTracks(matched_existing, original_tracked_size, msgs->header.stamp);
        }

        stats.removed_stale = RemoveStaleTracks();
        RebuildKdTree();
        CollectConfirmedCones(fused_map);
        PublishStatus(msgs->cone.size(), stats, fused_map.cone.size());
    }

    if (fused_map.cone.empty() && !msgs->cone.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "[cone_dedup] All %zu input cones are tentative (tracked_frames < %d), nothing published",
                          msgs->cone.size(), min_track_frames_);
    }

    fused_pub_->publish(fused_map);
}

TrackedCone ConeDedup::CreateTrackedCone(const common_msgs::msg::HuatCone &cone, const rclcpp::Time &stamp) {
    TrackedCone tc;
    tc.cone = cone;
    tc.cone.id = GetNewId();
    tc.tracked_frames = 1;
    tc.missed_frames = 0;
    tc.consecutive_matches = 1;
    tc.last_update = stamp;
    if (enable_kalman_) {
        KalmanInit(tc, cone.position_global.x, cone.position_global.y);
    }
    return tc;
}

int ConeDedup::GetNewId() {
    static std::atomic<int> id{-1};
    return ++id;
}
