#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "cone_dedup_algo.h"

using cone_dedup_algo::ComputeDynamicAlpha;
using cone_dedup_algo::FilterIndicesByRadiusSq;
using cone_dedup_algo::HungarianAssign;

static constexpr double kInfCost = 1e9;

// ── HungarianAssign ────────────────────────────────────────────────────────

TEST(HungarianAssignTest, EmptyCost) {
    std::vector<std::vector<double>> cost;
    auto result = HungarianAssign(cost, kInfCost);
    EXPECT_TRUE(result.empty());
}

TEST(HungarianAssignTest, SingleElement) {
    std::vector<std::vector<double>> cost = {{5.0}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0], 0);
}

TEST(HungarianAssignTest, SquareMatrixOptimal) {
    // 2x2: optimal is row0->col1 (1), row1->col0 (2), total = 3
    std::vector<std::vector<double>> cost = {{4, 1}, {2, 3}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 0);
}

TEST(HungarianAssignTest, RectangularMoreRows) {
    // 3x2: only 2 rows can be assigned
    std::vector<std::vector<double>> cost = {{1, 10}, {10, 1}, {5, 5}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 3u);
    // Row 0 -> col 0 (cost 1), Row 1 -> col 1 (cost 1), Row 2 -> unassigned
    EXPECT_EQ(result[0], 0);
    EXPECT_EQ(result[1], 1);
    EXPECT_EQ(result[2], -1);
}

TEST(HungarianAssignTest, RectangularMoreCols) {
    // 2x3: all rows assigned, one col unused
    // Optimal: row0->col1(2), row1->col0(2) = total 4
    std::vector<std::vector<double>> cost = {{1, 2, 3}, {2, 4, 6}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 0);
}

TEST(HungarianAssignTest, AllForbidden) {
    // All costs >= inf_cost → no assignments
    std::vector<std::vector<double>> cost = {{kInfCost, kInfCost}, {kInfCost, kInfCost}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0], -1);
    EXPECT_EQ(result[1], -1);
}

TEST(HungarianAssignTest, ZeroCost) {
    std::vector<std::vector<double>> cost = {{0, 0}, {0, 0}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 2u);
    // Any valid permutation is acceptable
    EXPECT_NE(result[0], result[1]);
    EXPECT_GE(result[0], 0);
    EXPECT_GE(result[1], 0);
}

TEST(HungarianAssignTest, ClassicAssignment) {
    // Classic 3x3 assignment problem
    // Optimal: 0->1(1), 1->2(1), 2->0(1) = total 3
    std::vector<std::vector<double>> cost = {{9, 1, 9}, {9, 9, 1}, {1, 9, 9}};
    auto result = HungarianAssign(cost, kInfCost);
    ASSERT_EQ(result.size(), 3u);
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 2);
    EXPECT_EQ(result[2], 0);
}

// ── ComputeDynamicAlpha ────────────────────────────────────────────────────

TEST(ComputeDynamicAlphaTest, DisabledReturnsFallback) {
    // When speed_ref <= 0, returns fallback
    EXPECT_NEAR(ComputeDynamicAlpha(10.0, 0.0, 0.1, 0.5, 0.3), 0.3, 1e-12);
    EXPECT_NEAR(ComputeDynamicAlpha(10.0, -1.0, 0.1, 0.5, 0.3), 0.3, 1e-12);
}

TEST(ComputeDynamicAlphaTest, ZeroSpeedReturnsAlphaMin) {
    // speed=0 → ratio=0 → alpha=alpha_min
    EXPECT_NEAR(ComputeDynamicAlpha(0.0, 15.0, 0.1, 0.5, 0.3), 0.1, 1e-12);
}

TEST(ComputeDynamicAlphaTest, AtRefSpeedReturnsAlphaMax) {
    // speed=speed_ref → ratio=1 → alpha=alpha_max
    EXPECT_NEAR(ComputeDynamicAlpha(15.0, 15.0, 0.1, 0.5, 0.3), 0.5, 1e-12);
}

TEST(ComputeDynamicAlphaTest, HalfRefSpeedInterpolates) {
    // speed=speed_ref/2 → ratio=0.5 → alpha=midpoint
    EXPECT_NEAR(ComputeDynamicAlpha(7.5, 15.0, 0.1, 0.5, 0.3), 0.3, 1e-12);
}

TEST(ComputeDynamicAlphaTest, ExceedsRefSpeedClamps) {
    // speed > speed_ref → ratio clamped to 1.0 → alpha=alpha_max
    EXPECT_NEAR(ComputeDynamicAlpha(30.0, 15.0, 0.1, 0.5, 0.3), 0.5, 1e-12);
}

TEST(ComputeDynamicAlphaTest, NegativeSpeedClamps) {
    // negative speed → ratio clamped to 0.0 → alpha=alpha_min
    EXPECT_NEAR(ComputeDynamicAlpha(-5.0, 15.0, 0.1, 0.5, 0.3), 0.1, 1e-12);
}

TEST(FilterIndicesByRadiusSq, KeepsOnlyNearbyPoints) {
    std::vector<double> xs = {0.0, 10.0, 1.0};
    std::vector<double> ys = {0.0, 0.0, 0.0};
    auto idx = FilterIndicesByRadiusSq(xs, ys, 0.0, 0.0, 4.0);  // r=2
    ASSERT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx[0], 0u);
    EXPECT_EQ(idx[1], 2u);
}

TEST(FilterIndicesByRadiusSq, EmptyInput) {
    std::vector<double> xs, ys;
    auto idx = FilterIndicesByRadiusSq(xs, ys, 0.0, 0.0, 1.0);
    EXPECT_TRUE(idx.empty());
}

TEST(FilterIndicesByRadiusSq, CulledIndicesAreSkippedByUnmatchedLoop) {
    // 契约：后续插入只遍历 valid_idx，被裁掉的下标不会变成新轨迹。
    std::vector<double> xs = {0.0, 100.0, 0.5};
    std::vector<double> ys = {0.0, 0.0, 0.0};
    auto valid = FilterIndicesByRadiusSq(xs, ys, 0.0, 0.0, 4.0);
    std::vector<bool> visited(xs.size(), false);
    for (size_t i : valid)
        visited[i] = true;
    EXPECT_TRUE(visited[0]);
    EXPECT_FALSE(visited[1]);
    EXPECT_TRUE(visited[2]);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
