#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

#include "cone_boundary.h"

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 非 ROS 测试桩：仅满足 core 的鸭子类型约束（编译期由下方 static_assert 校验）。
struct TestCone {
    struct BaseLinkPoint {
        double x{0.0};
        double y{0.0};
    };
    BaseLinkPoint position_base_link;
};

struct TestSide {
    double slope{0.0};
    double intercept{0.0};
    bool valid{false};
};

static_assert(slp_core::BaseLinkConeLike<TestCone>, "TestCone 必须满足锥桶视图约束");
static_assert(!slp_core::BaseLinkConeLike<double>, "非锥桶类型不得满足锥桶视图约束");
static_assert(slp_core::LineParamsLike<TestSide>, "TestSide 必须满足边界视图约束");
static_assert(!slp_core::LineParamsLike<double>, "非边界类型不得满足边界视图约束");

TestCone MakeCone(double y) {
    TestCone c;
    c.position_base_link.y = y;
    return c;
}

slp_core::ConeSideSplit Split(const std::vector<TestCone>& cones, double margin) {
    return slp_core::SplitConesBySide(std::span<const TestCone>{cones}, margin);
}

TestSide ValidSide(double slope, double intercept) {
    TestSide s;
    s.slope = slope;
    s.intercept = intercept;
    s.valid = true;
    return s;
}

constexpr slp_core::PlausibilityThresholds kDefaults{};

}  // namespace

// ---------------------------------------------------------------------------
// 锥桶左右分离
// ---------------------------------------------------------------------------

TEST(ConeBoundarySplit, SeparatesSidesAndIgnoresCenterBand) {
    const std::vector<TestCone> cones = {MakeCone(-1.8), MakeCone(1.2), MakeCone(0.0), MakeCone(-1.5), MakeCone(1.6)};
    const slp_core::ConeSideSplit split = Split(cones, 0.3);
    EXPECT_EQ(split.left, (std::vector<std::size_t>{0, 3}));
    EXPECT_EQ(split.right, (std::vector<std::size_t>{1, 4}));
}

TEST(ConeBoundarySplit, EmptyInputYieldsEmptySides) {
    const std::vector<TestCone> cones;
    const slp_core::ConeSideSplit split = Split(cones, 0.3);
    EXPECT_TRUE(split.left.empty());
    EXPECT_TRUE(split.right.empty());
}

TEST(ConeBoundarySplit, SparseSingleConeYieldsSingleSide) {
    const std::vector<TestCone> left_only = {MakeCone(-1.0)};
    const slp_core::ConeSideSplit left_split = Split(left_only, 0.3);
    EXPECT_EQ(left_split.left, (std::vector<std::size_t>{0}));
    EXPECT_TRUE(left_split.right.empty());

    const std::vector<TestCone> right_only = {MakeCone(2.0)};
    const slp_core::ConeSideSplit right_split = Split(right_only, 0.3);
    EXPECT_TRUE(right_split.left.empty());
    EXPECT_EQ(right_split.right, (std::vector<std::size_t>{0}));
}

TEST(ConeBoundarySplit, ExactMarginValuesBelongToNoSide) {
    // 原谓词为严格不等：y < -margin 归左、y > margin 归右，恰好等于 margin 不归任何一侧。
    const std::vector<TestCone> cones = {MakeCone(-0.3), MakeCone(0.3)};
    const slp_core::ConeSideSplit split = Split(cones, 0.3);
    EXPECT_TRUE(split.left.empty());
    EXPECT_TRUE(split.right.empty());

    const std::vector<TestCone> just_outside = {MakeCone(-0.3000001), MakeCone(0.3000001)};
    const slp_core::ConeSideSplit outside_split = Split(just_outside, 0.3);
    EXPECT_EQ(outside_split.left, (std::vector<std::size_t>{0}));
    EXPECT_EQ(outside_split.right, (std::vector<std::size_t>{1}));
}

TEST(ConeBoundarySplit, NaNOrdinateJoinsNoSide) {
    // 与原 copy_if 谓词一致：NaN 与任何数比较均为 false，两侧都不进入（记录现状，不收紧）。
    const std::vector<TestCone> cones = {MakeCone(kNaN), MakeCone(-1.0)};
    const slp_core::ConeSideSplit split = Split(cones, 0.3);
    EXPECT_EQ(split.left, (std::vector<std::size_t>{1}));
    EXPECT_TRUE(split.right.empty());
}

TEST(ConeBoundarySplit, SplitIndicesPreserveInputOrder) {
    const std::vector<TestCone> cones = {MakeCone(1.6), MakeCone(-1.8), MakeCone(1.2), MakeCone(-1.5)};
    const slp_core::ConeSideSplit split = Split(cones, 0.3);
    EXPECT_EQ(split.left, (std::vector<std::size_t>{1, 3}));
    EXPECT_EQ(split.right, (std::vector<std::size_t>{0, 2}));
}

// ---------------------------------------------------------------------------
// 边界可接受性判定（宽度阈值）
// ---------------------------------------------------------------------------

TEST(BoundaryPlausibility, PlausibleWhenWithinAllThresholds) {
    const auto verdict = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.1, -1.5), ValidSide(-0.1, 1.5), kDefaults);
    EXPECT_TRUE(verdict.plausible);
    EXPECT_EQ(verdict.issue, slp_core::PlausibilityIssue::None);
}

TEST(BoundaryPlausibility, NotBothValidIsRejected) {
    const auto left_invalid = slp_core::EvaluateBoundaryPlausibility(TestSide{}, ValidSide(0.0, 1.5), kDefaults);
    EXPECT_FALSE(left_invalid.plausible);
    EXPECT_EQ(left_invalid.issue, slp_core::PlausibilityIssue::NotBothValid);

    const auto right_invalid = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -1.5), TestSide{}, kDefaults);
    EXPECT_FALSE(right_invalid.plausible);
    EXPECT_EQ(right_invalid.issue, slp_core::PlausibilityIssue::NotBothValid);
}

TEST(BoundaryPlausibility, SlopeLimitsEnforcedPerSide) {
    const auto left_over =
        slp_core::EvaluateBoundaryPlausibility(ValidSide(0.31, -1.5), ValidSide(0.0, 1.5), kDefaults);
    EXPECT_FALSE(left_over.plausible);
    EXPECT_EQ(left_over.issue, slp_core::PlausibilityIssue::LeftSlopeExceeds);

    const auto right_over =
        slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -1.5), ValidSide(-0.31, 1.5), kDefaults);
    EXPECT_FALSE(right_over.plausible);
    EXPECT_EQ(right_over.issue, slp_core::PlausibilityIssue::RightSlopeExceeds);
}

TEST(BoundaryPlausibility, SlopeAtLimitAccepted) {
    // 原实现为严格 “> max”：恰好等于上限视为通过。
    const auto verdict = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.3, -1.5), ValidSide(-0.3, 1.5), kDefaults);
    EXPECT_TRUE(verdict.plausible);
    EXPECT_EQ(verdict.issue, slp_core::PlausibilityIssue::None);
}

TEST(BoundaryPlausibility, InterceptOrderViolationRejected) {
    // right 必须严格大于 left：相等与反转都被拒（!(right > left)）。
    const auto equal = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, 1.0), ValidSide(0.0, 1.0), kDefaults);
    EXPECT_FALSE(equal.plausible);
    EXPECT_EQ(equal.issue, slp_core::PlausibilityIssue::InterceptOrderViolated);

    const auto inverted = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, 1.0), ValidSide(0.0, -1.0), kDefaults);
    EXPECT_FALSE(inverted.plausible);
    EXPECT_EQ(inverted.issue, slp_core::PlausibilityIssue::InterceptOrderViolated);
}

TEST(BoundaryPlausibility, WidthOutOfRangeIsRejected) {
    // 宽度 0.4 < 0.5 下限
    const auto too_narrow =
        slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -0.2), ValidSide(0.0, 0.2), kDefaults);
    EXPECT_FALSE(too_narrow.plausible);
    EXPECT_EQ(too_narrow.issue, slp_core::PlausibilityIssue::WidthOutOfRange);

    // 宽度 6.1 > 2 * 3.0 上限
    const auto too_wide =
        slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -3.05), ValidSide(0.0, 3.05), kDefaults);
    EXPECT_FALSE(too_wide.plausible);
    EXPECT_EQ(too_wide.issue, slp_core::PlausibilityIssue::WidthOutOfRange);
}

TEST(BoundaryPlausibility, WidthAtBoundsAccepted) {
    // 恰好 0.5 / 6.0 边界均通过（严格不等号语义）。
    const auto at_min = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -0.25), ValidSide(0.0, 0.25), kDefaults);
    EXPECT_TRUE(at_min.plausible);

    const auto at_max = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -3.0), ValidSide(0.0, 3.0), kDefaults);
    EXPECT_TRUE(at_max.plausible);
}

TEST(BoundaryPlausibility, NaNInterceptFailsOrderCheck) {
    // 与原实现一致：!(right > left) 对 NaN 比较取非为 true → 顺序检查失败。
    const auto left_nan = slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, kNaN), ValidSide(0.0, 1.5), kDefaults);
    EXPECT_FALSE(left_nan.plausible);
    EXPECT_EQ(left_nan.issue, slp_core::PlausibilityIssue::InterceptOrderViolated);

    const auto right_nan =
        slp_core::EvaluateBoundaryPlausibility(ValidSide(0.0, -1.5), ValidSide(0.0, kNaN), kDefaults);
    EXPECT_FALSE(right_nan.plausible);
    EXPECT_EQ(right_nan.issue, slp_core::PlausibilityIssue::InterceptOrderViolated);
}

TEST(BoundaryPlausibility, NaNSlopeKeepsLegacyEvaluation) {
    // 与原实现一致：NaN 斜率在 “> max” 比较中为 false，不触发斜率拒绝（记录现状，不收紧）。
    const auto verdict = slp_core::EvaluateBoundaryPlausibility(ValidSide(kNaN, -1.5), ValidSide(0.0, 1.5), kDefaults);
    EXPECT_TRUE(verdict.plausible);
    EXPECT_EQ(verdict.issue, slp_core::PlausibilityIssue::None);
}

// ---------------------------------------------------------------------------
// 单侧可用性判定
// ---------------------------------------------------------------------------

TEST(SingleSideUsability, RequiresExactlyOneValid) {
    EXPECT_FALSE(slp_core::IsSingleSideUsable(ValidSide(0.0, -1.5), ValidSide(0.0, 1.5), 0.3));
    EXPECT_FALSE(slp_core::IsSingleSideUsable(TestSide{}, TestSide{}, 0.3));
    EXPECT_TRUE(slp_core::IsSingleSideUsable(ValidSide(0.2, -1.5), TestSide{}, 0.3));
    EXPECT_TRUE(slp_core::IsSingleSideUsable(TestSide{}, ValidSide(-0.2, 1.5), 0.3));
}

TEST(SingleSideUsability, SlopeLimitEnforced) {
    EXPECT_FALSE(slp_core::IsSingleSideUsable(ValidSide(0.5, -1.5), TestSide{}, 0.3));
    EXPECT_FALSE(slp_core::IsSingleSideUsable(TestSide{}, ValidSide(-0.5, 1.5), 0.3));
    // 恰好等于上限通过（<= 语义）
    EXPECT_TRUE(slp_core::IsSingleSideUsable(ValidSide(0.3, -1.5), TestSide{}, 0.3));
    // NaN 斜率 “<= max” 为 false → 不可用（与原实现一致）
    EXPECT_FALSE(slp_core::IsSingleSideUsable(ValidSide(kNaN, -1.5), TestSide{}, 0.3));
}

// ---------------------------------------------------------------------------
// 常量与默认阈值
// ---------------------------------------------------------------------------

TEST(CoreConstants, DefaultsMatchNodeParameterDefaults) {
    constexpr slp_core::PlausibilityThresholds defaults{};
    EXPECT_DOUBLE_EQ(defaults.max_abs_slope, 0.3);
    EXPECT_DOUBLE_EQ(defaults.max_intercept_diff, 3.0);
    EXPECT_DOUBLE_EQ(slp_core::kMinWidthAtOrigin, 0.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
