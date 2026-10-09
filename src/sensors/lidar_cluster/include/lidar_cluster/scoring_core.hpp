#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace lidar_cluster {

/**
 * @brief 锥桶置信度评分的聚合参数。
 *
 * 从 LidarCluster 成员变量中提取，使评分函数不依赖类状态，
 * 从而可以独立进行单元测试。
 *
 * #59：随评分几何核一起下沉为纯 std core（原定义在 lidar_cluster_scoring.h，
 * 该头现经 using 声明转发至此，ROS 侧行为不变）。
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
inline double ScoreAspectPenalty(double length, double width, double height, const ScoringParams& p) {
    double penalty = 0.0;
    if (length > height)
        penalty += p.conf_penalty_aspect * (length - height);
    if (width > height)
        penalty += p.conf_penalty_aspect * (width - height);
    return penalty;
}

/**
 * @brief 计算尺寸惩罚。
 *
 * 根据 road_type 使用不同的惩罚策略：road_type==1（加速赛道）使用固定值，
 * 其他类型按超出/不足阈值的比例惩罚。
 */
inline double ScoreSizePenalty(double height, double area, const ScoringParams& p) {
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

/**
 * @brief 计算倾斜惩罚。
 *
 * 对点集做 PCA，主轴线（协方差矩阵最大特征值对应的特征向量）与垂直方向偏差
 * 越大惩罚越重。
 *
 * #59：原实现经 pcl::computeMeanAndCovarianceMatrix 取协方差；此处改为手写
 * 两遍法协方差（均值→去均值外积累加→按点数归一），仍以
 * Eigen::SelfAdjointEigenSolver 取主轴线。数学等价：协方差整体缩放不改变
 * 特征向量方向；非有限点按 PCL 既有语义跳过（点数不足时协方差保持零矩阵，
 * 与旧实现同输入同输出）。点集少于 3 个时不构成可估主轴，直接返回 0。
 */
inline double ScoreTiltPenalty(std::span<const Eigen::Vector3f> points, const ScoringParams& p) {
    if (points.size() < 3)
        return 0.0;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    std::size_t valid = 0;
    for (const auto& pt : points) {
        if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z()))
            continue;  // 与 pcl::computeMeanAndCovarianceMatrix 的非有限点语义一致
        mean += pt;
        ++valid;
    }
    if (valid > 0) {
        mean /= static_cast<float>(valid);
        for (const auto& pt : points) {
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z()))
                continue;
            const Eigen::Vector3f d = pt - mean;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(valid);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success)
        return 0.0;
    float cos_theta = std::abs(solver.eigenvectors().col(2).dot(Eigen::Vector3f::UnitZ()));
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
    float tilt_deg = std::acos(cos_theta) * 180.0f / std::numbers::pi_v<float>;
    return tilt_deg > p.max_tilt_angle ? p.conf_penalty_tilt * (tilt_deg - p.max_tilt_angle) / p.max_tilt_angle : 0.0;
}

/**
 * @brief 计算锥桶聚类的置信度分数。
 *
 * 分数 = 1.0 - 长宽惩罚 - 尺寸惩罚 - 倾斜惩罚。
 * 返回范围 [-1.0, 1.0]，其中 <0 表示不可信。
 *
 * #59：点云 AABB（length/width/height）由 ROS 侧适配层算好后传入；
 * 本函数只负责合成。points 为去均值前的原始点集（见 ScoreTiltPenalty）。
 */
inline double ComputeConfidence(double length, double width, double height, std::span<const Eigen::Vector3f> points,
                                const ScoringParams& p) {
    double score = 1.0 - ScoreAspectPenalty(length, width, height, p) - ScoreSizePenalty(height, length * width, p) -
                   ScoreTiltPenalty(points, p);
    return score < 0 ? -1.0 : score;
}

}  // namespace lidar_cluster
