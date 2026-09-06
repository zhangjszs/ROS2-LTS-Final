#include <gtest/gtest.h>

#include <cmath>

#include "lidar_cluster_scoring.h"
#include "profiler/frame_profiler.h"
#include "string_utils.h"

// ── ScoreAspectPenalty ─────────────────────────────────────────────────────

TEST(ScoreAspectPenaltyTest, NoPenaltyWhenCompact) {
    ScoringParams p;
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.3, 0.3, p), 0.0, 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenLengthExceedsHeight) {
    ScoringParams p;
    double expected = p.conf_penalty_aspect * (0.5 - 0.3);
    EXPECT_NEAR(ScoreAspectPenalty(0.5, 0.3, 0.3, p), expected, 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenWidthExceedsHeight) {
    ScoringParams p;
    double expected = p.conf_penalty_aspect * (0.6 - 0.3);
    EXPECT_NEAR(ScoreAspectPenalty(0.3, 0.6, 0.3, p), expected, 1e-12);
}

TEST(ScoreAspectPenaltyTest, PenaltyWhenBothExceed) {
    ScoringParams p;
    double expected = p.conf_penalty_aspect * (0.5 - 0.3) + p.conf_penalty_aspect * (0.6 - 0.3);
    EXPECT_NEAR(ScoreAspectPenalty(0.5, 0.6, 0.3, p), expected, 1e-12);
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
    // height=0.5 > max → penalty = 7.0 * (0.5 - 0.3) = 1.4
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.1, p), p.conf_penalty_height_over * (0.5 - 0.3), 1e-12);
}

TEST(ScoreSizePenaltyTest, PenaltyHeightUnderThreshold) {
    ScoringParams p;
    p.min_height = 0.2;
    p.max_height = 1.0;
    p.min_area = 0.0;
    p.max_area = 1.0;
    // height=0.1 < min → penalty = 1.5 * (0.2 - 0.1) = 0.15
    EXPECT_NEAR(ScoreSizePenalty(0.1, 0.1, p), p.conf_penalty_height_under * (0.2 - 0.1), 1e-12);
}

TEST(ScoreSizePenaltyTest, AccelRoadTypeFixedPenalty) {
    ScoringParams p;
    p.road_type = 1;
    p.min_height = 0.0;
    p.max_height = 0.3;
    p.min_area = 0.0;
    p.max_area = 0.1;
    // height=0.5 > max → fixed penalty 0.05, area=0.2 > max → fixed penalty 0.05
    EXPECT_NEAR(ScoreSizePenalty(0.5, 0.2, p), 2 * p.conf_penalty_over_max_accel, 1e-12);
}

// ── ComputeConfidence ──────────────────────────────────────────────────────

TEST(ComputeConfidenceTest, PerfectConeScoresHigh) {
    ScoringParams p;
    p.min_height = 0.1;
    p.max_height = 0.5;
    p.min_area = 0.05;
    p.max_area = 0.3;
    PointType max_pt, min_pt;
    max_pt.x = 0.15;
    max_pt.y = 0.15;
    max_pt.z = 0.3;
    min_pt.x = -0.15;
    min_pt.y = -0.15;
    min_pt.z = 0.0;
    Eigen::Vector4f centroid(0, 0, 0.15, 0);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    // Single point → PCA needs >= 3 points → tilt penalty = 0
    PointType pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = 0.15;
    cloud->push_back(pt);
    double score = ComputeConfidence(max_pt, min_pt, centroid, cloud, p);
    // length=0.3, width=0.3, height=0.3 → no aspect penalty
    // size within thresholds → no size penalty
    // cloud size < 3 → no tilt penalty
    EXPECT_NEAR(score, 1.0, 1e-12);
}

TEST(ComputeConfidenceTest, WideFlatObjectScoresLow) {
    ScoringParams p;
    PointType max_pt, min_pt;
    max_pt.x = 1.0;
    max_pt.y = 1.0;
    max_pt.z = 0.1;
    min_pt.x = -1.0;
    min_pt.y = -1.0;
    min_pt.z = 0.0;
    Eigen::Vector4f centroid(0, 0, 0.05, 0);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    PointType pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = 0.05;
    cloud->push_back(pt);
    double score = ComputeConfidence(max_pt, min_pt, centroid, cloud, p);
    // length=2.0 >> height=0.1 → heavy aspect penalty
    EXPECT_LT(score, 0.0);
}

TEST(ComputeConfidenceTest, ScoreClampedToNegativeOne) {
    ScoringParams p;
    PointType max_pt, min_pt;
    max_pt.x = 10.0;
    max_pt.y = 10.0;
    max_pt.z = 0.05;
    min_pt.x = -10.0;
    min_pt.y = -10.0;
    min_pt.z = 0.0;
    Eigen::Vector4f centroid(0, 0, 0.025, 0);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    PointType pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = 0.025;
    cloud->push_back(pt);
    double score = ComputeConfidence(max_pt, min_pt, centroid, cloud, p);
    EXPECT_NEAR(score, -1.0, 1e-12);
}

// ── ScoreTiltPenalty ─────────────────────────────────────────────────────

TEST(ScoreTiltPenaltyTest, TooFewPointsReturnsZero) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    PointType pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = 0.15;
    cloud->push_back(pt);
    cloud->push_back(pt);  // 2 points < 3 → returns 0
    EXPECT_NEAR(ScoreTiltPenalty(cloud, p), 0.0, 1e-12);
}

TEST(ScoreTiltPenaltyTest, VerticalConeNoPenalty) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    // Points aligned along z-axis → vertical → tilt ≈ 0°
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    for (int i = 0; i < 5; ++i) {
        PointType pt;
        pt.x = 0.0;
        pt.y = 0.0;
        pt.z = i * 0.05f;
        cloud->push_back(pt);
    }
    EXPECT_NEAR(ScoreTiltPenalty(cloud, p), 0.0, 1e-6);
}

TEST(ScoreTiltPenaltyTest, TiltedBeyondThresholdPenalized) {
    ScoringParams p;
    p.max_tilt_angle = 10.0;  // 小阈值确保被触发
    p.conf_penalty_tilt = 0.5;
    // 主方向偏离 Z 轴约 45°（x 和 z 等量分布）
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    for (int i = 0; i < 5; ++i) {
        PointType pt;
        pt.x = i * 0.1f;
        pt.y = 0.0f;
        pt.z = i * 0.1f;
        cloud->push_back(pt);
    }
    double penalty = ScoreTiltPenalty(cloud, p);
    EXPECT_GT(penalty, 0.0);  // 45° > 10° threshold → positive penalty
}

TEST(ScoreTiltPenaltyTest, NullCloudReturnsZero) {
    ScoringParams p;
    p.max_tilt_angle = 25.0;
    p.conf_penalty_tilt = 0.5;
    pcl::PointCloud<PointType>::Ptr cloud;  // null
    EXPECT_NEAR(ScoreTiltPenalty(cloud, p), 0.0, 1e-12);
}

// ── SingleFrameDedup ──────────────────────────────────────────────────────

static common_msgs::msg::HuatConeCluster MakeConeCluster(const std::vector<std::tuple<float, float, float>>& cones) {
    common_msgs::msg::HuatConeCluster msg;
    for (const auto& c : cones) {
        geometry_msgs::msg::Point32 p;
        p.x = std::get<0>(c);
        p.y = std::get<1>(c);
        p.z = std::get<2>(c);
        msg.points.push_back(p);
        msg.confidence.push_back(0.8f);
        msg.obj_dist.push_back(0.0f);
        msg.max_points.push_back(p);
        msg.min_points.push_back(p);
    }
    return msg;
}

TEST(SingleFrameDedupTest, NoDedupWhenFarApart) {
    auto msg = MakeConeCluster({{0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}});
    int removed = SingleFrameDedup(msg, 0.5);
    EXPECT_EQ(removed, 0);
    EXPECT_EQ(msg.points.size(), 2u);
}

TEST(SingleFrameDedupTest, RemovesNearDuplicate) {
    auto msg = MakeConeCluster({{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}});
    msg.confidence[0] = 0.9f;
    msg.confidence[1] = 0.5f;
    int removed = SingleFrameDedup(msg, 0.5);
    EXPECT_EQ(removed, 1);
    EXPECT_EQ(msg.points.size(), 1u);
    EXPECT_NEAR(msg.confidence[0], 0.9f, 1e-6);
}

TEST(SingleFrameDedupTest, KeepsHigherConfidence) {
    auto msg = MakeConeCluster({{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}});
    msg.confidence[0] = 0.3f;
    msg.confidence[1] = 0.8f;
    int removed = SingleFrameDedup(msg, 0.5);
    EXPECT_EQ(removed, 1);
    EXPECT_EQ(msg.points.size(), 1u);
    EXPECT_NEAR(msg.confidence[0], 0.8f, 1e-6);
}

TEST(SingleFrameDedupTest, EmptyInput) {
    common_msgs::msg::HuatConeCluster msg;
    int removed = SingleFrameDedup(msg, 0.5);
    EXPECT_EQ(removed, 0);
}

TEST(SingleFrameDedupTest, SinglePoint) {
    auto msg = MakeConeCluster({{1.0f, 2.0f, 0.0f}});
    int removed = SingleFrameDedup(msg, 0.5);
    EXPECT_EQ(removed, 0);
    EXPECT_EQ(msg.points.size(), 1u);
}

// ── FrameProfiler Format Tests (C++20 std::format) ─────────────────────────

TEST(FrameProfilerFormatTest, SummaryFormat) {
    std::string summary = FrameProfiler::formatSummary(1.234, 2.345, 3.456, 7.035, 0);
    EXPECT_EQ(summary,
              "[lidar_cluster] Profiling (ms) | PassThrough=1.23 GroundSeg=2.35 Cluster=3.46 Total=7.04 NoData=0");
}

TEST(FrameProfilerFormatTest, WarnMessageWithNoData) {
    std::string warn_msg = FrameProfiler::formatWarnMessage(45.5, 30.0, 3, 2);
    EXPECT_EQ(warn_msg,
              "[lidar_cluster] Frame time 45.50 ms exceeds WARN threshold 30.00 ms (consecutive=3, no_data=2)");
}

TEST(FrameProfilerFormatTest, WarnMessageWithoutNoData) {
    std::string warn_msg = FrameProfiler::formatWarnMessage(35.2, 30.0, 1, -1);
    EXPECT_EQ(warn_msg,
              "[lidar_cluster] Frame time 35.20 ms exceeds WARN threshold 30.00 ms (consecutive=1)");
}

TEST(FrameProfilerFormatTest, ErrorMessageFormat) {
    std::string err_msg = FrameProfiler::formatErrorMessage(55.25, 50.0);
    EXPECT_EQ(err_msg, "[lidar_cluster] Frame time 55.25 ms exceeds ERROR threshold 50.00 ms");
}

TEST(FrameProfilerFormatTest, RecoveryMessageFormat) {
    std::string rec_msg = FrameProfiler::formatRecoveryMessage(22.10, 4);
    EXPECT_EQ(rec_msg, "[lidar_cluster] Frame time recovered to 22.10 ms (was consecutive=4)");
}

// ── StringUtils Tests (C++20 std::string_view & std::format) ───────────────

TEST(StringUtilsTest, ParseCsvDoublesNormal) {
    std::vector<double> vals;
    lidar_cluster::parseCsvDoubles("1.5, 2.25, 3.75", vals);
    ASSERT_EQ(vals.size(), 3u);
    EXPECT_DOUBLE_EQ(vals[0], 1.5);
    EXPECT_DOUBLE_EQ(vals[1], 2.25);
    EXPECT_DOUBLE_EQ(vals[2], 3.75);
}

TEST(StringUtilsTest, ParseCsvDoublesWithWhitespaceAndEmpty) {
    std::vector<double> vals;
    lidar_cluster::parseCsvDoubles(" 10.0,  , 20.5 , ", vals);
    ASSERT_EQ(vals.size(), 2u);
    EXPECT_DOUBLE_EQ(vals[0], 10.0);
    EXPECT_DOUBLE_EQ(vals[1], 20.5);
}

TEST(StringUtilsTest, FormatCsvDoubles) {
    std::vector<double> vals = {1.2, 3.456, 7.891};
    std::string formatted = lidar_cluster::formatCsvDoubles(vals);
    EXPECT_EQ(formatted, "1.20, 3.46, 7.89");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
