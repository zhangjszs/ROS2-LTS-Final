#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

#include "common_msgs/msg/huat_cone_cluster.hpp"
#include "lidar_cluster/scoring_core.hpp"
#include "point_type.h"

// #59：纯标量数学与评分参数已下沉为纯 std core（lidar_cluster/scoring_core.hpp）。
// 此处经 using 声明转发，ROS 侧调用点看到的仍是同一份实现，无数值漂移空间。
using lidar_cluster::ScoreAspectPenalty;
using lidar_cluster::ScoreSizePenalty;
using lidar_cluster::ScoringParams;

/**
 * @brief 计算倾斜惩罚（ROS 侧薄适配层）。
 *
 * 把 pcl::PointCloud<PointType> 转为点集后转发 lidar_cluster::ScoreTiltPenalty。
 * 评分数值与行为不变（#59）。
 */
double ScoreTiltPenalty(const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p);

/**
 * @brief 计算锥桶聚类的置信度分数（ROS 侧薄适配层）。
 *
 * 由 AABB 角点算 length/width/height 后转发 lidar_cluster::ComputeConfidence。
 * 分数 = 1.0 - 长宽惩罚 - 尺寸惩罚 - 倾斜惩罚。
 * 返回范围 [-1.0, 1.0]，其中 <0 表示不可信。
 */
double ComputeConfidence(PointType max_pt, PointType min_pt, Eigen::Vector4f centroid,
                         const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p);

/**
 * @brief 单帧近邻去重。
 *
 * 对距离小于 dedup_radius 的锥桶对，保留置信度较高的一个。
 * 返回被移除的锥桶数量。
 */
int SingleFrameDedup(common_msgs::msg::HuatConeCluster& position, double dedup_radius);
