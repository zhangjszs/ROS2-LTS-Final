#include <gtest/gtest.h>

#include <vector>

#include "centerline_core.hpp"

namespace {

using skidpad::CenterlineParams;
using skidpad::ComputeCenterlineCore;
using skidpad::Point2D;

TEST(CenterlineCore, EmptyInputsReturnEmpty) {
    CenterlineParams params;
    std::vector<Point2D> empty;
    std::vector<Point2D> pts = {{1.0, -1.0}};

    EXPECT_TRUE(ComputeCenterlineCore(empty, empty, params).empty());
    EXPECT_TRUE(ComputeCenterlineCore(empty, pts, params).empty());
    EXPECT_TRUE(ComputeCenterlineCore(pts, empty, params).empty());
}

TEST(CenterlineCore, SymmetricConesGoldenSample) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    std::vector<Point2D> left = {
        {1.0, -1.0},
        {3.0, -1.0},
        {5.0, -1.0},
    };
    std::vector<Point2D> right = {
        {1.0, 1.0},
        {3.0, 1.0},
        {5.0, 1.0},
    };

    auto result = ComputeCenterlineCore(left, right, params);
    ASSERT_EQ(result.size(), 3U);

    EXPECT_DOUBLE_EQ(result[0].x, 1.0);
    EXPECT_DOUBLE_EQ(result[0].y, 0.0);
    EXPECT_DOUBLE_EQ(result[1].x, 3.0);
    EXPECT_DOUBLE_EQ(result[1].y, 0.0);
    EXPECT_DOUBLE_EQ(result[2].x, 5.0);
    EXPECT_DOUBLE_EQ(result[2].y, 0.0);
}

TEST(CenterlineCore, AllExceedMaxPairDistReturnsEmpty) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    // Distance between left and right is 14.0 > 6.0
    std::vector<Point2D> left = {{1.0, -7.0}, {3.0, -7.0}};
    std::vector<Point2D> right = {{1.0, 7.0}, {3.0, 7.0}};

    auto result = ComputeCenterlineCore(left, right, params);
    EXPECT_TRUE(result.empty());
}

TEST(CenterlineCore, BoundaryPairDistance) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    // Exactly at distance 6.0: (0, -3.0) and (0, 3.0) -> dist = 6.0
    std::vector<Point2D> left_exact = {{0.0, -3.0}};
    std::vector<Point2D> right_exact = {{0.0, 3.0}};
    auto res_exact = ComputeCenterlineCore(left_exact, right_exact, params);
    ASSERT_EQ(res_exact.size(), 1U);
    EXPECT_DOUBLE_EQ(res_exact[0].x, 0.0);
    EXPECT_DOUBLE_EQ(res_exact[0].y, 0.0);

    // Slightly exceeding 6.0: (0, -3.001) and (0, 3.001) -> dist = 6.002
    std::vector<Point2D> left_over = {{0.0, -3.001}};
    std::vector<Point2D> right_over = {{0.0, 3.001}};
    auto res_over = ComputeCenterlineCore(left_over, right_over, params);
    EXPECT_TRUE(res_over.empty());
}

TEST(CenterlineCore, DeduplicationWithinThreshold) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    // path_spacing * 0.5 = 0.25 threshold.
    // Consecutive points closer than 0.25 must be deduplicated.
    std::vector<Point2D> left = {
        {2.0, -1.0},
        {2.1, -1.0},  // Midpoint with right[0] will be (2.05, 0.0), dist 0.05 < 0.25
        {4.0, -1.0},
    };
    std::vector<Point2D> right = {
        {2.0, 1.0},
        {2.1, 1.0},
        {4.0, 1.0},
    };

    auto result = ComputeCenterlineCore(left, right, params);
    // (2.0, 0.0) will be taken, (2.05, 0.0) or (2.1, 0.0) deduplicated, then (4.0, 0.0)
    ASSERT_EQ(result.size(), 2U);
    EXPECT_DOUBLE_EQ(result[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result[0].y, 0.0);
    EXPECT_DOUBLE_EQ(result[1].x, 4.0);
    EXPECT_DOUBLE_EQ(result[1].y, 0.0);
}

TEST(CenterlineCore, LookaheadFiltering) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 10.0,
    };

    // Midpoint (15.0, 0.0) has dist 15.0 > lookahead 10.0, so it is filtered out.
    std::vector<Point2D> left = {
        {2.0, -1.0},
        {8.0, -1.0},
        {15.0, -1.0},
    };
    std::vector<Point2D> right = {
        {2.0, 1.0},
        {8.0, 1.0},
        {15.0, 1.0},
    };

    auto result = ComputeCenterlineCore(left, right, params);
    ASSERT_EQ(result.size(), 2U);
    EXPECT_DOUBLE_EQ(result[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result[1].x, 8.0);
}

TEST(CenterlineCore, ScrambledOrderChainedFromOrigin) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    // Provide cones in scrambled order along x
    std::vector<Point2D> left = {
        {5.0, -1.0},
        {1.0, -1.0},
        {3.0, -1.0},
    };
    std::vector<Point2D> right = {
        {5.0, 1.0},
        {1.0, 1.0},
        {3.0, 1.0},
    };

    auto result = ComputeCenterlineCore(left, right, params);
    ASSERT_EQ(result.size(), 3U);
    // Nearest neighbor chain from (0,0) starts at (1,0), then (3,0), then (5,0)
    EXPECT_DOUBLE_EQ(result[0].x, 1.0);
    EXPECT_DOUBLE_EQ(result[1].x, 3.0);
    EXPECT_DOUBLE_EQ(result[2].x, 5.0);
}

TEST(CenterlineCore, DuplicateConesDoNotCrash) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    std::vector<Point2D> left = {
        {2.0, -1.0},
        {2.0, -1.0},
        {2.0, -1.0},
    };
    std::vector<Point2D> right = {
        {2.0, 1.0},
        {2.0, 1.0},
    };

    auto result = ComputeCenterlineCore(left, right, params);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_DOUBLE_EQ(result[0].x, 2.0);
    EXPECT_DOUBLE_EQ(result[0].y, 0.0);
}

// —— #89：核心边界参数清洗与防御测试 ——

TEST(CenterlineCore, SanitizeConfigFallsBackToDefaults) {
    skidpad::IcpApfConfig cfg;
    cfg.center_margin = std::numeric_limits<double>::quiet_NaN();
    cfg.k_a = -1.0;
    cfg.k_r = std::numeric_limits<double>::quiet_NaN();
    cfg.d_0 = -1.0;
    cfg.path_spacing = -0.5;
    cfg.path_lookahead = std::numeric_limits<double>::quiet_NaN();
    cfg.apf_step_limit = -0.1;
    cfg.icp_max_iter = -1;

    const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
    ASSERT_EQ(sanitized.fixes.size(), 8u);

    EXPECT_STREQ(sanitized.fixes[0].name, "center_margin");
    EXPECT_DOUBLE_EQ(sanitized.cfg.center_margin, 0.3);
    EXPECT_TRUE(std::isnan(sanitized.fixes[0].bad_value));

    EXPECT_STREQ(sanitized.fixes[1].name, "k_a");
    EXPECT_DOUBLE_EQ(sanitized.cfg.k_a, 0.5);
    EXPECT_DOUBLE_EQ(sanitized.fixes[1].bad_value, -1.0);

    EXPECT_STREQ(sanitized.fixes[2].name, "k_r");
    EXPECT_DOUBLE_EQ(sanitized.cfg.k_r, 2.0);
    EXPECT_TRUE(std::isnan(sanitized.fixes[2].bad_value));

    EXPECT_STREQ(sanitized.fixes[3].name, "d_0");
    EXPECT_DOUBLE_EQ(sanitized.cfg.d_0, 2.0);
    EXPECT_DOUBLE_EQ(sanitized.fixes[3].bad_value, -1.0);

    EXPECT_STREQ(sanitized.fixes[4].name, "path_station_spacing");
    EXPECT_DOUBLE_EQ(sanitized.cfg.path_spacing, 0.5);
    EXPECT_DOUBLE_EQ(sanitized.fixes[4].bad_value, -0.5);

    EXPECT_STREQ(sanitized.fixes[5].name, "path_lookahead_distance");
    EXPECT_DOUBLE_EQ(sanitized.cfg.path_lookahead, 20.0);
    EXPECT_TRUE(std::isnan(sanitized.fixes[5].bad_value));

    EXPECT_STREQ(sanitized.fixes[6].name, "apf_step_limit");
    EXPECT_DOUBLE_EQ(sanitized.cfg.apf_step_limit, 0.2);
    EXPECT_DOUBLE_EQ(sanitized.fixes[6].bad_value, -0.1);

    EXPECT_STREQ(sanitized.fixes[7].name, "icp_max_iterations");
    EXPECT_EQ(sanitized.cfg.icp_max_iter, 20);
    EXPECT_DOUBLE_EQ(sanitized.fixes[7].bad_value, -1.0);
}

TEST(CenterlineCore, SanitizeConfigKeepsLegalConfigUntouched) {
    skidpad::IcpApfConfig cfg;
    cfg.center_margin = 0.0;
    cfg.k_a = 0.0;
    cfg.k_r = 0.0;
    cfg.d_0 = 0.01;
    cfg.path_spacing = 0.1;
    cfg.path_lookahead = 1.0;
    cfg.apf_step_limit = 0.01;
    cfg.icp_max_iter = 1;

    const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
    EXPECT_TRUE(sanitized.fixes.empty());
    EXPECT_DOUBLE_EQ(sanitized.cfg.center_margin, 0.0);
    EXPECT_DOUBLE_EQ(sanitized.cfg.k_a, 0.0);
    EXPECT_DOUBLE_EQ(sanitized.cfg.k_r, 0.0);
    EXPECT_DOUBLE_EQ(sanitized.cfg.d_0, 0.01);
    EXPECT_DOUBLE_EQ(sanitized.cfg.path_spacing, 0.1);
    EXPECT_DOUBLE_EQ(sanitized.cfg.path_lookahead, 1.0);
    EXPECT_DOUBLE_EQ(sanitized.cfg.apf_step_limit, 0.01);
    EXPECT_EQ(sanitized.cfg.icp_max_iter, 1);
}

TEST(CenterlineCore, SanitizeConfigIllegalCenterMarginCombinations) {
    for (double bad_margin : {std::numeric_limits<double>::quiet_NaN(), -0.3, std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
        skidpad::IcpApfConfig cfg;
        cfg.center_margin = bad_margin;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "center_margin");
        EXPECT_DOUBLE_EQ(sanitized.cfg.center_margin, 0.3);
    }
}

TEST(CenterlineCore, SanitizeConfigIllegalD0AndStepLimit) {
    for (double bad_d0 : {0.0, -1.0, -100.0, std::numeric_limits<double>::quiet_NaN()}) {
        skidpad::IcpApfConfig cfg;
        cfg.d_0 = bad_d0;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "d_0");
        EXPECT_DOUBLE_EQ(sanitized.cfg.d_0, 2.0);
    }

    for (double bad_step :
         {0.0, -0.2, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        skidpad::IcpApfConfig cfg;
        cfg.apf_step_limit = bad_step;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "apf_step_limit");
        EXPECT_DOUBLE_EQ(sanitized.cfg.apf_step_limit, 0.2);
    }
}

TEST(CenterlineCore, SanitizeConfigIllegalSpacingAndLookahead) {
    for (double bad_spacing : {0.0, -0.5, -1e-6, std::numeric_limits<double>::quiet_NaN()}) {
        skidpad::IcpApfConfig cfg;
        cfg.path_spacing = bad_spacing;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "path_station_spacing");
        EXPECT_DOUBLE_EQ(sanitized.cfg.path_spacing, 0.5);
    }

    for (double bad_lookahead :
         {0.0, -20.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        skidpad::IcpApfConfig cfg;
        cfg.path_lookahead = bad_lookahead;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "path_lookahead_distance");
        EXPECT_DOUBLE_EQ(sanitized.cfg.path_lookahead, 20.0);
    }
}

TEST(CenterlineCore, SanitizeConfigIllegalIcpIterAndGains) {
    for (int bad_iter : {0, -1, -20}) {
        skidpad::IcpApfConfig cfg;
        cfg.icp_max_iter = bad_iter;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 1u);
        EXPECT_STREQ(sanitized.fixes[0].name, "icp_max_iterations");
        EXPECT_EQ(sanitized.cfg.icp_max_iter, 20);
    }

    for (double bad_gain : {-0.5, -10.0, std::numeric_limits<double>::quiet_NaN()}) {
        skidpad::IcpApfConfig cfg;
        cfg.k_a = bad_gain;
        cfg.k_r = bad_gain;
        const auto sanitized = skidpad::SanitizeSkidpadConfig(cfg);
        EXPECT_EQ(sanitized.fixes.size(), 2u);
        EXPECT_DOUBLE_EQ(sanitized.cfg.k_a, 0.5);
        EXPECT_DOUBLE_EQ(sanitized.cfg.k_r, 2.0);
    }
}

TEST(CenterlineCore, SanitizedConfigProducesFiniteCenterline) {
    skidpad::IcpApfConfig bad_cfg;
    bad_cfg.center_margin = std::numeric_limits<double>::quiet_NaN();
    bad_cfg.d_0 = -1.0;
    bad_cfg.apf_step_limit = std::numeric_limits<double>::quiet_NaN();
    bad_cfg.path_spacing = -0.5;
    bad_cfg.path_lookahead = 0.0;

    const auto sanitized = skidpad::SanitizeSkidpadConfig(bad_cfg);
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = sanitized.cfg.path_spacing,
        .path_lookahead = sanitized.cfg.path_lookahead,
    };

    std::vector<Point2D> left = {{1.0, -1.0}, {3.0, -1.0}, {5.0, -1.0}};
    std::vector<Point2D> right = {{1.0, 1.0}, {3.0, 1.0}, {5.0, 1.0}};
    const auto path = ComputeCenterlineCore(left, right, params);
    ASSERT_FALSE(path.empty());
    for (const auto& pt : path) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
    }
}

}  // namespace
