#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "lidar_cluster/scoring_core.hpp"

using lidar_cluster::ComputeConfidence;
using lidar_cluster::ScoreAspectPenalty;
using lidar_cluster::ScoreSizePenalty;
using lidar_cluster::ScoreTiltPenalty;
using lidar_cluster::ScoringParams;

namespace {

// z 轴竖直点列（主轴线 = UnitZ，倾斜 0°）
std::vector<Eigen::Vector3f> VerticalLine() {
    return {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.05f}, {0.0f, 0.0f, 0.10f}, {0.0f, 0.0f, 0.15f}, {0.0f, 0.0f, 0.20f}};
}

// x-z 平面 45° 点列（主轴线与 UnitZ 夹角恰为 45°）
std::vector<Eigen::Vector3f> Line45Deg() {
    return {{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.1f}, {0.2f, 0.0f, 0.2f}, {0.3f, 0.0f, 0.3f}, {0.4f, 0.0f, 0.4f}};
}

// 任意共线点列：方向 (1,2,3)/|(1,2,3)|，与 UnitZ 夹角 acos(3/sqrt(14)) ≈ 36.8699°
std::vector<Eigen::Vector3f> Collinear123() {
    return {{0.0f, 0.0f, 0.0f}, {0.1f, 0.2f, 0.3f}, {0.2f, 0.4f, 0.6f}, {0.3f, 0.6f, 0.9f}};
}

}  // namespace

// ── ScoreAspectPenalty ─────────────────────────────────────────────────────

TEST(ScoreAspectPenaltyTest, NoPenaltyWhenCompact) {
    ScoringParams p;
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.3, 0.3, p), 0.0, 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenLengthExceedsHeight) {
    ScoringParams p;
    EXPECT_NEAR(ScoreAspectPenalty(0.5, 0.3, 0.3, p), p.conf_penalty_aspect * (0.5 - 0.3), 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenWidthExceedsHeight) {
    ScoringParams p;
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.6, 0.3, p), p.conf_penalty_aspect * (0.6 - 0.3), 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenBothExceed) {
    ScoringParams p;
    double expected = p.conf_penalty_aspect * (0.5 - 0.3) + p.conf_penalty_aspect * (0.6 - 0.3);
    EXPECT_NEAR(ScoreAspectPenalty(0.5, 0.6, 0.3, p), expected, 1e-12);
}

TEST(ScoreAspectPenaltyTest, EqualToHeightIsNotPenalized) {
    ScoringParams p;
    // 严格大于才罚：恰好相等不罚
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.3, 0.3, p), 0.0, 1e-12);
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.4, 0.4, p), 0.0, 1e-12);
}

// ── ScoreSizePenalty ───────────────────────────────────────────────────────

TEST(ScoreSizePenaltyTest, NoPenaltyWithinThresholds) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    EXPECT_NEAR(ScoreSizePenalty(0.3, 0.1, p), 0.0, 1e-12);
}

TEST(ScoreSizePenaltyTest, PenaltyHeightOverThreshold) {
    ScoringParams p;
    p.min_height = 0.0;
    p.max_height = 0.3;
    p.min_area = 0.0;
    p.max_area = 1.0;
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.1, p), p.conf_penalty_height_over * (0.5 - 0.3), 1e-12);
}

TEST(ScoreSizePenaltyTest, PenaltyAreaOverThreshold) {
    ScoringParams p;
    p.min_height = 0.0;
    p.max_height = 1.0;
    p.min_area = 0.0;
    p.max_area = 0.3;
    EXPECT_NEAR(ScoreSizePenalty(0.3, 0.5, p), p.conf_penalty_area_over * (0.5 - 0.3), 1e-12);
}

TEST(ScoreSizePenaltyTest, PenaltyHeightUnderThreshold) {
    ScoringParams p;
    p.min_height = 0.2;
    p.max_height = 1.0;
    p.min_area = 0.0;
    p.max_area = 1.0;
    EXPECT_NEAR(ScoreSizePenalty(0.1, 0.1, p), p.conf_penalty_height_under * (0.2 - 0.1), 1e-12);
}

TEST(ScoreSizePenaltyTest, PenaltyAreaUnderThreshold) {
    ScoringParams p;
    p.min_height = 0.0;
    p.max_height = 1.0;
    p.min_area = 0.05;
    p.max_area = 1.0;
    EXPECT_NEAR(ScoreSizePenalty(0.3, 0.02, p), p.conf_penalty_area_under * (0.05 - 0.02), 1e-12);
}

TEST(ScoreSizePenaltyTest, OverAndUnderStack) {
    ScoringParams p;
    p.min_height = 0.2;
    p.max_height = 0.3;
    p.min_area = 0.05;
    p.max_area = 0.1;
    // height=0.5 超上限 + area=0.02 低于下限 → 两项叠加
    double expected = p.conf_penalty_height_over * (0.5 - 0.3) + p.conf_penalty_area_under * (0.05 - 0.02);
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.02, p), expected, 1e-12);
}

TEST(ScoreSizePenaltyTest, AccelRoadTypeFixedPenalty) {
    ScoringParams p;
    p.road_type = 1;
    p.min_height = 0.0;
    p.max_height = 0.3;
    p.min_area = 0.0;
    p.max_area = 0.1;
    // 加速赛道（road_type==1）超上限为固定惩罚，与超出量无关
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.2, p), 2 * p.conf_penalty_over_max_accel, 1e-12);
}

TEST(ScoreSizePenaltyTest, AccelRoadTypeUnderStillProportional) {
    ScoringParams p;
    p.road_type = 1;
    p.min_height = 0.2;
    p.max_height = 1.0;
    p.min_area = 0.05;
    p.max_area = 1.0;
    // road_type==1 只改变"超上限"分支；低于下限仍按比例罚
    double expected = p.conf_penalty_height_under * (0.2 - 0.1) + p.conf_penalty_area_under * (0.05 - 0.02);
    EXPECT_NEAR(ScoreSizePenalty(0.1, 0.02, p), expected, 1e-12);
}

TEST(ScoreSizePenaltyTest, AccelRoadTypeAtThresholdNoPenalty) {
    ScoringParams p;
    p.road_type = 1;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.3, p), 0.0, 1e-12);
}

// ── ScoreTiltPenalty（PCA 几何核） ──────────────────────────────────────────

TEST(ScoreTiltPenaltyTest, FewerThanThreePointsReturnsZero) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    const std::vector<Eigen::Vector3f> none;
    const std::vector<Eigen::Vector3f> one = {{0.0f, 0.0f, 0.15f}};
    const std::vector<Eigen::Vector3f> two = {{0.0f, 0.0f, 0.1f}, {0.0f, 0.0f, 0.2f}};
    EXPECT_NEAR(ScoreTiltPenalty(none, p), 0.0, 1e-12);
    EXPECT_NEAR(ScoreTiltPenalty(one, p), 0.0, 1e-12);
    EXPECT_NEAR(ScoreTiltPenalty(two, p), 0.0, 1e-12);
}

TEST(ScoreTiltPenaltyTest, VerticalLineNoPenalty) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    EXPECT_NEAR(ScoreTiltPenalty(VerticalLine(), p), 0.0, 1e-9);
}

TEST(ScoreTiltPenaltyTest, GoldenSample45DegLine) {
    ScoringParams p;
    p.max_tilt_angle = 10.0;
    p.conf_penalty_tilt = 0.5;
    // 主轴线与 UnitZ 夹角恰 45°：penalty = 0.5 * (45 - 10) / 10 = 1.75
    // （该黄金值与下沉前 pcl::computeMeanAndCovarianceMatrix 实现逐位一致：
    //   见 colcon 侧未改动的 test_lidar_cluster_scoring 与等价性比对记录）
    EXPECT_NEAR(ScoreTiltPenalty(Line45Deg(), p), 1.75, 1e-4);
}

TEST(ScoreTiltPenaltyTest, GoldenSampleCollinear123) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    // 方向 (1,2,3)/|(1,2,3)|：cos(tilt) = 3/sqrt(14) ≈ 0.8017837 → tilt ≈ 36.6992°
    // penalty = 0.5 * (36.6992 - 25) / 25 ≈ 0.2339845
    // （黄金值与下沉前 pcl::computeMeanAndCovarianceMatrix 实现一致到 1e-6° 以内：
    //   见 colcon 侧未改动的 test_lidar_cluster_scoring 与等价性比对记录）
    EXPECT_NEAR(ScoreTiltPenalty(Collinear123(), p), 0.2339845, 1e-4);
}

TEST(ScoreTiltPenaltyTest, BelowThresholdNoPenalty) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    // 缓倾斜：方向 (0.01,0,0.05)，与 UnitZ 夹角 ≈ 11.3° < 25°
    std::vector<Eigen::Vector3f> pts;
    for (int i = 0; i < 5; ++i)
        pts.emplace_back(0.01f * i, 0.0f, 0.05f * i);
    EXPECT_NEAR(ScoreTiltPenalty(pts, p), 0.0, 1e-9);
}

TEST(ScoreTiltPenaltyTest, ExactlyThreePointsAccepted) {
    ScoringParams p;
    p.max_tilt_angle = 10.0;
    p.conf_penalty_tilt = 0.5;
    // size == 3 是下限：取 45° 线前三点，与黄金样本同值
    std::vector<Eigen::Vector3f> pts = {{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.1f}, {0.2f, 0.0f, 0.2f}};
    EXPECT_NEAR(ScoreTiltPenalty(pts, p), 1.75, 1e-4);
}

TEST(ScoreTiltPenaltyTest, DuplicatePointsDoNotCrash) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    // 病态输入：全重复点 → 协方差为零矩阵。旧实现同样落在零矩阵特征求解上，
    // 行为（有限值、不崩溃）保持一致
    std::vector<Eigen::Vector3f> pts(5, Eigen::Vector3f(0.1f, 0.1f, 0.1f));
    double penalty = ScoreTiltPenalty(pts, p);
    EXPECT_TRUE(std::isfinite(penalty));
}

TEST(ScoreTiltPenaltyTest, NonFinitePointsFilteredLikePcl) {
    ScoringParams p;
    p.max_tilt_angle = 10.0;
    p.conf_penalty_tilt = 0.5;
    // pcl::computeMeanAndCovarianceMatrix 跳过非有限点：3 个有效 45° 点 + 2 个 NaN
    // 应与纯 45° 线（黄金样本）同值
    auto pts = Line45Deg();
    pts.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
    pts.emplace_back(0.0f, std::numeric_limits<float>::quiet_NaN(), 1.0f);
    EXPECT_NEAR(ScoreTiltPenalty(pts, p), 1.75, 1e-4);
}

TEST(ScoreTiltPenaltyTest, AllNonFinitePointsFiniteResult) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<Eigen::Vector3f> pts = {{nan, nan, nan}, {nan, 1.0f, 2.0f}, {1.0f, nan, nan}, {nan, nan, nan}};
    EXPECT_TRUE(std::isfinite(ScoreTiltPenalty(pts, p)));
}

TEST(ScoreTiltPenaltyTest, PlanarSquareFiniteAndDeterministic) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    // 平面点集（非共线）：主轴线为平面内方差最大方向，结果有限且可复现
    std::vector<Eigen::Vector3f> pts = {
        {0.3f, 0.3f, 0.0f}, {-0.3f, 0.3f, 0.0f}, {0.3f, -0.3f, 0.5f}, {-0.3f, -0.3f, 0.5f}};
    double first = ScoreTiltPenalty(pts, p);
    EXPECT_TRUE(std::isfinite(first));
    EXPECT_DOUBLE_EQ(ScoreTiltPenalty(pts, p), first);
}

// ── ComputeConfidence（置信度合成） ─────────────────────────────────────────

TEST(ComputeConfidenceTest, PerfectConeScoresHigh) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    // length=width=height=0.3 → 无长宽惩罚；area=0.09 在阈值内 → 无尺寸惩罚；
    // 竖直点列 → 无倾斜惩罚
    EXPECT_NEAR(ComputeConfidence(0.3, 0.3, 0.3, VerticalLine(), p), 1.0, 1e-9);
}

TEST(ComputeConfidenceTest, WideFlatObjectScoresLow) {
    ScoringParams p;
    // length=2.0 >> height=0.1 → 重长宽惩罚；score < 0 → 钳位 -1
    EXPECT_NEAR(ComputeConfidence(2.0, 2.0, 0.1, VerticalLine(), p), -1.0, 1e-12);
}

TEST(ComputeConfidenceTest, ScoreClampedToNegativeOne) {
    ScoringParams p;
    EXPECT_NEAR(ComputeConfidence(10.0, 10.0, 0.05, VerticalLine(), p), -1.0, 1e-12);
}

TEST(ComputeConfidenceTest, ZeroScoreNotClamped) {
    ScoringParams p;
    p.conf_penalty_aspect = 4.0;
    p.min_height = 0.0;
    p.max_height = 1.0;
    p.min_area = 0.0;
    p.max_area = 1.0;
    // 惩罚恰为 1.0 → score 恰为 0；钳位只映射严格负值，0 保持 0。
    // 取 2 的幂尺寸使 (0.5-0.25)*4.0 在二进制浮点下精确等于 1.0
    EXPECT_NEAR(ComputeConfidence(0.5, 0.25, 0.25, VerticalLine(), p), 0.0, 1e-12);
}

TEST(ComputeConfidenceTest, ScoreStaysWithinUnitRange) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    p.max_tilt_angle = 10.0;
    const double dims[][3] = {
        {0.3, 0.3, 0.3},   {0.5, 0.3, 0.3}, {0.3, 0.6, 0.3}, {2.0, 2.0, 0.1},
        {0.05, 0.05, 0.4}, {1.0, 0.8, 0.2}, {0.0, 0.0, 0.3}, {0.4, 0.4, 0.05},
    };
    for (const auto& d : dims) {
        for (const auto& pts : {VerticalLine(), Line45Deg(), Collinear123()}) {
            double score = ComputeConfidence(d[0], d[1], d[2], pts, p);
            EXPECT_TRUE(std::isfinite(score));
            EXPECT_GE(score, -1.0);
            EXPECT_LE(score, 1.0);
        }
    }
}

TEST(ComputeConfidenceTest, SizePenaltyCanDragScoreNegative) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.2;
    p.min_area = 0.0;
    p.max_area = 0.05;
    p.max_tilt_angle = 25.0;
    // height=0.5 超上限 7.0*(0.5-0.2)=2.1；area=0.3 超上限 2.0*(0.3-0.05)=0.5 → 合计 2.6 > 1
    EXPECT_NEAR(ComputeConfidence(0.5, 0.6, 0.5, VerticalLine(), p), -1.0, 1e-12);
}

TEST(ComputeConfidenceTest, NullCloudEquivalentToFewPoints) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    // 空点集（ROS 侧 null cloud 适配后同此路径）：无倾斜惩罚
    EXPECT_NEAR(ComputeConfidence(0.3, 0.3, 0.3, std::vector<Eigen::Vector3f>{}, p), 1.0, 1e-9);
}
