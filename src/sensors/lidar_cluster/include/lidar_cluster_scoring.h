#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

#include "common_msgs/msg/huat_cone_cluster.hpp"
#include "point_type.h"

/**
 * @brief 锥桶置信度评分的聚合参数。
 *
 * 从 LidarCluster 成员变量中提取，使评分函数不依赖类状态，
 * 从而可以独立进行单元测试。
 */
struct ScoringParams {
    // 长宽超出高度的惩罚系数
    double conf_penalty_aspect = 2.0;
    // 加速赛道：超尺寸惩罚
    double conf_penalty_over_max_accel = 0.05;
    // 赛道：高度超标的惩罚系数
    double conf_penalty_height_over = 7.0;
    // 赛道：面积超标的惩罚系数
    double conf_penalty_area_over = 2.0;
    // 高度不足的惩罚系数
    double conf_penalty_height_under = 1.5;
    // 面积不足的惩罚系数
    double conf_penalty_area_under = 2.0;
    // 倾斜超标的惩罚系数
    double conf_penalty_tilt = 0.5;
    // 阈值
    double min_height = -1.0;
    double max_height = -1.0;
    double min_area = -1.0;
    double max_area = -1.0;
    double max_tilt_angle = 25.0;
    int road_type = 2;
};

/**
 * @brief 计算长宽比惩罚。
 *
 * 当 length 或 width 超过 height 时施加惩罚——锥桶应近似"高瘦"而非"矮胖"。
 */
double ScoreAspectPenalty(double length, double width, double height, const ScoringParams& p);

/**
 * @brief 计算尺寸惩罚。
 *
 * 根据 road_type 使用不同的惩罚策略：road_type==1（加速赛道）使用固定值，
 * 其他类型按超出/不足阈值的比例惩罚。
 */
double ScoreSizePenalty(double height, double area, const ScoringParams& p);

/**
 * @brief 计算倾斜惩罚。
 *
 * 对点云做 PCA，主轴线与垂直方向偏差越大惩罚越重。
 */
double ScoreTiltPenalty(const pcl::PointCloud<PointType>::Ptr& cloud, const ScoringParams& p);

/**
 * @brief 计算锥桶聚类的置信度分数。
 *
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
