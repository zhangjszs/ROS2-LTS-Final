#include "lidar_cluster_scoring.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <algorithm>
#include <cmath>
#include <numbers>

double ScoreAspectPenalty(double length, double width, double height, const ScoringParams& p) {
    double penalty = 0.0;
    if (length > height)
        penalty += p.conf_penalty_aspect * (length - height);
    if (width > height)
        penalty += p.conf_penalty_aspect * (width - height);
    return penalty;
}

double ScoreSizePenalty(double height, double area, const ScoringParams& p) {
    double penalty = 0.0;
    if (p.road_type == 1) {
        if (height > p.max_height)
            penalty += p.conf_penalty_over_max_accel;
        if (area > p.max_area)
            penalty += p.conf_penalty_over_max_accel;
    } else {
        if (height > p.max_height)
            penalty += p.conf_penalty_height_over * (height - p.max_height);
        if (area > p.max_area)
            penalty += p.conf_penalty_area_over * (area - p.max_area);
    }
    if (height < p.min_height)
        penalty += p.conf_penalty_height_under * (p.min_height - height);
    if (area < p.min_area)
        penalty += p.conf_penalty_area_under * (p.min_area - area);
    return penalty;
}

double ScoreTiltPenalty(const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p) {
    if (!cloud || cloud->size() < 3)
        return 0.0;
    Eigen::Matrix3f cov;
    Eigen::Vector4f pca_centroid;
    pcl::computeMeanAndCovarianceMatrix(*cloud, cov, pca_centroid);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success)
        return 0.0;
    float cos_theta = std::abs(solver.eigenvectors().col(2).dot(Eigen::Vector3f::UnitZ()));
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
    float tilt_deg = std::acos(cos_theta) * 180.0f / std::numbers::pi_v<float>;
    return tilt_deg > p.max_tilt_angle ? p.conf_penalty_tilt * (tilt_deg - p.max_tilt_angle) / p.max_tilt_angle : 0.0;
}

double ComputeConfidence(PointType max_pt, PointType min_pt, Eigen::Vector4f centroid,
                         const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p) {
    double length = std::fabs(max_pt.x - min_pt.x);
    double width = std::fabs(max_pt.y - min_pt.y);
    double height = std::fabs(max_pt.z - min_pt.z);
    double score = 1.0 - ScoreAspectPenalty(length, width, height, p) - ScoreSizePenalty(height, length * width, p) -
                   ScoreTiltPenalty(cloud, p);
    return score < 0 ? -1.0 : score;
}

int SingleFrameDedup(common_msgs::msg::HuatConeCluster& position, double dedup_radius) {
    if (position.points.size() <= 1)
        return 0;
    int raw_count = static_cast<int>(position.points.size());
    std::vector<bool> removed(position.points.size(), false);
    double dedup_r_sq = dedup_radius * dedup_radius;
    for (size_t i = 0; i < position.points.size(); i++) {
        if (removed[i])
            continue;
        for (size_t j = i + 1; j < position.points.size(); j++) {
            if (removed[j])
                continue;
            double dx = position.points[i].x - position.points[j].x;
            double dy = position.points[i].y - position.points[j].y;
            if (dx * dx + dy * dy < dedup_r_sq) {
                if (position.confidence[i] >= position.confidence[j]) {
                    removed[j] = true;
                } else {
                    removed[i] = true;
                    break;
                }
            }
        }
    }
    common_msgs::msg::HuatConeCluster deduped;
    deduped.header = position.header;
    for (size_t i = 0; i < position.points.size(); i++) {
        if (!removed[i]) {
            deduped.points.push_back(position.points[i]);
            deduped.confidence.push_back(position.confidence[i]);
            deduped.obj_dist.push_back(position.obj_dist[i]);
            deduped.max_points.push_back(position.max_points[i]);
            deduped.min_points.push_back(position.min_points[i]);
            if (i < position.type.size()) {
                deduped.type.push_back(position.type[i]);
            }
        }
    }
    int removed_count = raw_count - static_cast<int>(deduped.points.size());
    position = std::move(deduped);
    return removed_count;
}
