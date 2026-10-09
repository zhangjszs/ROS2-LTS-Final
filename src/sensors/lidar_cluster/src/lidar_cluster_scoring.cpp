#include "lidar_cluster_scoring.h"

#include <cmath>
#include <utility>
#include <vector>

// #59：评分几何核已下沉至纯 std core（lidar_cluster/scoring_core.hpp）。
// 本文件只剩 ROS 侧薄适配层：pcl::PointCloud<PointType> → 点集 → 转发 core 函数；
// SingleFrameDedup 的 msgs 操作留在 ROS 侧。

double ScoreTiltPenalty(const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p) {
    if (!cloud)
        return 0.0;
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud->size());
    for (const auto& pt : *cloud)
        points.emplace_back(pt.x, pt.y, pt.z);
    return lidar_cluster::ScoreTiltPenalty(points, p);
}

double ComputeConfidence(PointType max_pt, PointType min_pt, [[maybe_unused]] Eigen::Vector4f centroid,
                         const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p) {
    const double length = std::fabs(max_pt.x - min_pt.x);
    const double width = std::fabs(max_pt.y - min_pt.y);
    const double height = std::fabs(max_pt.z - min_pt.z);
    std::vector<Eigen::Vector3f> points;
    if (cloud) {
        points.reserve(cloud->size());
        for (const auto& pt : *cloud)
            points.emplace_back(pt.x, pt.y, pt.z);
    }
    return lidar_cluster::ComputeConfidence(length, width, height, points, p);
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
