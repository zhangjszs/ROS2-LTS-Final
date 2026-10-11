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

    std::vector<Point2D> left = {
        {2.0, -1.0},
        {8.0, -1.0},
        {15.0, -1.0},  // Midpoint (15.0, 0.0) has dist 15.0 > lookahead 10.0
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

}  // namespace
