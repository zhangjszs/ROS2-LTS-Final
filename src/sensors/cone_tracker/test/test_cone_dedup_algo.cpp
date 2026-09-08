#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <span>
#include <vector>

#include "cone_dedup_algo.h"

using cone_dedup_algo::ComputeDynamicAlpha;
using cone_dedup_algo::FilterIndicesByRadiusSq;
using cone_dedup_algo::HungarianAssign;
using cone_dedup_algo::Point2D;

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

TEST(HungarianAssignTest, FlatMatrixEquivalentTo2D) {
    // 验证展平的一维连续内存版本与二维 vector 版本结果严格一致
    std::vector<std::vector<double>> cost_2d = {{4, 1}, {2, 3}};
    std::vector<double> cost_flat = {4, 1, 2, 3};
    auto res_2d = HungarianAssign(cost_2d, kInfCost);
    auto res_flat = HungarianAssign(cost_flat, 2, 2, kInfCost);
    ASSERT_EQ(res_flat.size(), res_2d.size());
    for (size_t i = 0; i < res_2d.size(); ++i) {
        EXPECT_EQ(res_flat[i], res_2d[i]);
    }

    // 矩形矩阵测试
    std::vector<std::vector<double>> cost_rect_2d = {{1, 10}, {10, 1}, {5, 5}};
    std::vector<double> cost_rect_flat = {1, 10, 10, 1, 5, 5};
    auto res_rect_2d = HungarianAssign(cost_rect_2d, kInfCost);
    auto res_rect_flat = HungarianAssign(cost_rect_flat, 3, 2, kInfCost);
    ASSERT_EQ(res_rect_flat.size(), res_rect_2d.size());
    for (size_t i = 0; i < res_rect_2d.size(); ++i) {
        EXPECT_EQ(res_rect_flat[i], res_rect_2d[i]);
    }
}

TEST(MatrixViewTest, BasicOperations) {
    cone_dedup_algo::MatrixView<double> empty_view;
    EXPECT_TRUE(empty_view.empty());
    EXPECT_EQ(empty_view.rows(), 0u);
    EXPECT_EQ(empty_view.cols(), 0u);
    EXPECT_EQ(empty_view.size(), 0u);

    std::array<double, 6> raw_buf = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    cone_dedup_algo::MatrixView<double> mat(raw_buf, 2, 3);
    EXPECT_FALSE(mat.empty());
    EXPECT_EQ(mat.rows(), 2u);
    EXPECT_EQ(mat.cols(), 3u);
    EXPECT_EQ(mat.size(), 6u);

    EXPECT_DOUBLE_EQ(mat(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(mat(0, 2), 3.0);
    EXPECT_DOUBLE_EQ(mat(1, 0), 4.0);
    EXPECT_DOUBLE_EQ(mat(1, 2), 6.0);

    // 修改验证
    mat(1, 1) = 99.0;
    EXPECT_DOUBLE_EQ(raw_buf[4], 99.0);

    // Row span 验证
    auto row0 = mat.row(0);
    ASSERT_EQ(row0.size(), 3u);
    EXPECT_DOUBLE_EQ(row0[0], 1.0);
    EXPECT_DOUBLE_EQ(row0[1], 2.0);
    EXPECT_DOUBLE_EQ(row0[2], 3.0);
}

TEST(FlatMatrixTest, StorageAndViews) {
    cone_dedup_algo::FlatMatrix<double> mat(2, 3, 10.0);
    EXPECT_EQ(mat.rows(), 2u);
    EXPECT_EQ(mat.cols(), 3u);
    EXPECT_EQ(mat.size(), 6u);
    EXPECT_DOUBLE_EQ(mat(0, 1), 10.0);

    mat(0, 1) = 42.0;
    EXPECT_DOUBLE_EQ(mat(0, 1), 42.0);

    // 视图获取与传递
    cone_dedup_algo::MatrixView<const double> view = mat.view();
    EXPECT_EQ(view.rows(), 2u);
    EXPECT_EQ(view.cols(), 3u);
    EXPECT_DOUBLE_EQ(view(0, 1), 42.0);

    // 隐式转换支持
    cone_dedup_algo::MatrixView<double> mut_view = mat;
    mut_view(1, 2) = 100.0;
    EXPECT_DOUBLE_EQ(mat(1, 2), 100.0);

    // Resize 与 Clear
    mat.resize(3, 3, -1.0);
    EXPECT_EQ(mat.rows(), 3u);
    EXPECT_EQ(mat.cols(), 3u);
    EXPECT_EQ(mat.size(), 9u);
    EXPECT_DOUBLE_EQ(mat(2, 2), -1.0);

    mat.clear();
    EXPECT_TRUE(mat.empty());
    EXPECT_EQ(mat.size(), 0u);
}

TEST(HungarianAssignTest, MatrixViewOptimal) {
    cone_dedup_algo::FlatMatrix<double> cost(3, 3);
    cost(0, 0) = 9.0; cost(0, 1) = 1.0; cost(0, 2) = 9.0;
    cost(1, 0) = 9.0; cost(1, 1) = 9.0; cost(1, 2) = 1.0;
    cost(2, 0) = 1.0; cost(2, 1) = 9.0; cost(2, 2) = 9.0;

    auto result = HungarianAssign(cost.view(), kInfCost);
    ASSERT_EQ(result.size(), 3u);
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 2);
    EXPECT_EQ(result[2], 0);

    // 矩形矩阵非方阵测试：3 行 2 列
    cone_dedup_algo::FlatMatrix<double> cost_rect(3, 2);
    cost_rect(0, 0) = 1.0;  cost_rect(0, 1) = 10.0;
    cost_rect(1, 0) = 10.0; cost_rect(1, 1) = 1.0;
    cost_rect(2, 0) = 5.0;  cost_rect(2, 1) = 5.0;

    auto result_rect = HungarianAssign(cost_rect.view(), kInfCost);
    ASSERT_EQ(result_rect.size(), 3u);
    EXPECT_EQ(result_rect[0], 0);
    EXPECT_EQ(result_rect[1], 1);
    EXPECT_EQ(result_rect[2], -1);
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

TEST(FilterIndicesByRadiusSq, StdArraySupport) {
    // C++20 std::span 零开销兼容固定大小栈容器 std::array
    std::array<double, 3> xs = {0.0, 10.0, 1.0};
    std::array<double, 3> ys = {0.0, 0.0, 0.0};
    auto idx = FilterIndicesByRadiusSq(xs, ys, 0.0, 0.0, 4.0);
    ASSERT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx[0], 0u);
    EXPECT_EQ(idx[1], 2u);
}

TEST(FilterIndicesByRadiusSq, RawArraySupport) {
    // C++20 std::span 自动退化推导 C 语言原始数组，安全传递大小
    double xs[] = {0.0, 10.0, 1.0};
    double ys[] = {0.0, 0.0, 0.0};
    auto idx = FilterIndicesByRadiusSq(xs, ys, 0.0, 0.0, 4.0);
    ASSERT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx[0], 0u);
    EXPECT_EQ(idx[1], 2u);
}

TEST(FilterIndicesByRadiusSq, SubspanSupport) {
    // 验证利用 std::span::subspan 进行零拷贝切片过滤
    std::vector<double> xs = {999.0, 0.0, 10.0, 1.0, 999.0};
    std::vector<double> ys = {999.0, 0.0, 0.0, 0.0, 999.0};
    std::span<const double> xs_sub(xs.data() + 1, 3);
    std::span<const double> ys_sub(ys.data() + 1, 3);
    auto idx = FilterIndicesByRadiusSq(xs_sub, ys_sub, 0.0, 0.0, 4.0);
    ASSERT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx[0], 0u);
    EXPECT_EQ(idx[1], 2u);
}

TEST(FilterIndicesByRadiusSq, Point2DSupport) {
    // 验证 Point2D 结构体视图接口
    std::vector<Point2D> points = {{0.0, 0.0}, {10.0, 0.0}, {1.0, 0.0}};
    auto idx = FilterIndicesByRadiusSq(points, 0.0, 0.0, 4.0);
    ASSERT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx[0], 0u);
    EXPECT_EQ(idx[1], 2u);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
