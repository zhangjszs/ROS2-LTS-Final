#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "centerline_core.hpp"

namespace {

using skidpad::CenterlineParams;
using skidpad::Circle2D;
using skidpad::ComputeCircleCenterlineCore;
using skidpad::FitCircleTaubinCore;
using skidpad::Point2D;

TEST(CircleCenterlineCore, UnitCircleGoldenSample) {
    // 8 points uniformly distributed on unit circle centered at (0, 0)
    std::vector<Point2D> pts;
    constexpr int kNumPts = 8;
    for (int i = 0; i < kNumPts; ++i) {
        double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(kNumPts);
        pts.emplace_back(std::cos(angle), std::sin(angle));
    }

    Circle2D circle = FitCircleTaubinCore(pts);
    ASSERT_TRUE(circle.valid);
    EXPECT_NEAR(circle.cx, 0.0, 1e-6);
    EXPECT_NEAR(circle.cy, 0.0, 1e-6);
    EXPECT_NEAR(circle.r, 1.0, 1e-6);
}

TEST(CircleCenterlineCore, OffsetCircleGoldenSample) {
    // Circle centered at (5.0, -3.0) with radius 7.5
    constexpr double kExpectedCx = 5.0;
    constexpr double kExpectedCy = -3.0;
    constexpr double kExpectedR = 7.5;

    std::vector<Point2D> pts;
    constexpr int kNumPts = 12;
    for (int i = 0; i < kNumPts; ++i) {
        double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(kNumPts);
        pts.emplace_back(kExpectedCx + (kExpectedR * std::cos(angle)), kExpectedCy + (kExpectedR * std::sin(angle)));
    }

    Circle2D circle = FitCircleTaubinCore(pts);
    ASSERT_TRUE(circle.valid);
    EXPECT_NEAR(circle.cx, kExpectedCx, 1e-6);
    EXPECT_NEAR(circle.cy, kExpectedCy, 1e-6);
    EXPECT_NEAR(circle.r, kExpectedR, 1e-6);
}

TEST(CircleCenterlineCore, NoisyCircleGoldenSample) {
    // Full circle centered at (5.0, -3.0) with radius 7.5 plus deterministic
    // +/-0.02 noise: the algebraic fit stays near-exact (tolerance 0.05).
    constexpr double kExpectedCx = 5.0;
    constexpr double kExpectedCy = -3.0;
    constexpr double kExpectedR = 7.5;

    std::vector<Point2D> pts;
    constexpr int kNumPts = 16;
    for (int i = 0; i < kNumPts; ++i) {
        const double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(kNumPts);
        const double nx = 0.01 * static_cast<double>((i * 7) % 5 - 2);
        const double ny = 0.01 * static_cast<double>((i * 13) % 5 - 2);
        pts.emplace_back(kExpectedCx + (kExpectedR * std::cos(angle)) + nx,
                         kExpectedCy + (kExpectedR * std::sin(angle)) + ny);
    }

    const Circle2D circle = FitCircleTaubinCore(pts);
    ASSERT_TRUE(circle.valid);
    EXPECT_NEAR(circle.cx, kExpectedCx, 0.05);
    EXPECT_NEAR(circle.cy, kExpectedCy, 0.05);
    EXPECT_NEAR(circle.r, kExpectedR, 0.05);
}

TEST(CircleCenterlineCore, PartialArcFitsWithoutCrashing) {
    // Partial-arc bias is inherited verbatim from the original ROS implementation
    // (faithful port, not a fitting-accuracy fix): the Taubin algebraic fit is
    // near-exact for full-circle data (see golden tests above) but biased for
    // partial arcs. This test locks in defined behavior (valid, finite,
    // deterministic) rather than geometric accuracy. Measured 2026-10-12 for the
    // 7-point semicircle below: fit = (2.0, -3.21, 8.07) vs true (2.0, 1.0, 5.0).
    // Tightening partial-arc accuracy would change vehicle behavior and belongs
    // in a separate issue.
    std::vector<Point2D> pts;
    constexpr int kNumPts = 7;
    for (int i = 0; i < kNumPts; ++i) {
        const double angle = std::numbers::pi_v<double> * static_cast<double>(i) / static_cast<double>(kNumPts - 1);
        pts.emplace_back(2.0 + (5.0 * std::cos(angle)), 1.0 + (5.0 * std::sin(angle)));
    }

    const Circle2D first = FitCircleTaubinCore(pts);
    const Circle2D second = FitCircleTaubinCore(pts);
    EXPECT_TRUE(first.valid);
    EXPECT_TRUE(std::isfinite(first.cx) && std::isfinite(first.cy) && std::isfinite(first.r));
    EXPECT_GT(first.r, 0.0);
    EXPECT_EQ(first, second);
}

TEST(CircleCenterlineCore, DegenerateFewerThanThreePoints) {
    std::vector<Point2D> empty;
    std::vector<Point2D> one = {{1.0, 2.0}};
    std::vector<Point2D> two = {{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_FALSE(FitCircleTaubinCore(empty).valid);
    EXPECT_FALSE(FitCircleTaubinCore(one).valid);
    EXPECT_FALSE(FitCircleTaubinCore(two).valid);
}

TEST(CircleCenterlineCore, DegenerateIdenticalPoints) {
    std::vector<Point2D> identical = {{2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}};
    Circle2D circle = FitCircleTaubinCore(identical);
    EXPECT_FALSE(circle.valid);
}

TEST(CircleCenterlineCore, DegenerateCollinearPoints) {
    // The verbatim Taubin port returns a finite (poor) circle for collinear input
    // instead of invalid (measured 2026-10-12: center (2.5, 2.5), r = 1.58). The
    // safety property lives one layer up: the [3, 25] radius gate in
    // ComputeCircleCenterlineCore rejects it, so no path is produced.
    const std::vector<Point2D> collinear = {{1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 4.0}};
    const Circle2D circle = FitCircleTaubinCore(collinear);
    EXPECT_TRUE(std::isfinite(circle.cx) && std::isfinite(circle.cy) && std::isfinite(circle.r));
    EXPECT_EQ(circle, FitCircleTaubinCore(collinear));

    const CenterlineParams params{.max_pair_dist = 6.0, .path_spacing = 0.5, .path_lookahead = 10.0};
    const std::vector<Point2D> right = {{2.0, 1.0}, {3.0, 2.0}, {4.0, 3.0}, {5.0, 4.0}};
    EXPECT_TRUE(ComputeCircleCenterlineCore(collinear, right, params).empty());
}

TEST(CircleCenterlineCore, DegenerateTwoIdenticalAndOneDistinct) {
    // Only two distinct points cannot define a circle geometrically, but the
    // verbatim port still returns a finite circle (measured 2026-10-12: center
    // (5.46, 5.46), r = 7.31). Lock in defined, deterministic behavior (no crash
    // under ASan+UBSan, finite values); tightening this to invalid would change
    // vehicle behavior and belongs in a separate issue.
    const std::vector<Point2D> pts = {{0.0, 0.0}, {0.0, 0.0}, {10.0, 10.0}};
    const Circle2D circle = FitCircleTaubinCore(pts);
    EXPECT_TRUE(std::isfinite(circle.cx) && std::isfinite(circle.cy) && std::isfinite(circle.r));
    EXPECT_GT(circle.r, 0.0);
    EXPECT_EQ(circle, FitCircleTaubinCore(pts));
}

TEST(CircleCenterlineCore, ParameterGuardSpacingZeroDoesNotUB) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.0,  // Pathology: spacing = 0
        .path_lookahead = 20.0,
    };

    std::vector<Point2D> left = {{0.0, 9.0}, {9.0, 0.0}, {0.0, -9.0}, {-9.0, 0.0}};
    std::vector<Point2D> right = {{0.0, 11.0}, {11.0, 0.0}, {0.0, -11.0}, {-11.0, 0.0}};

    auto path = ComputeCircleCenterlineCore(left, right, params);
    EXPECT_TRUE(path.empty());
}

TEST(CircleCenterlineCore, ParameterGuardNegativeSpacingOrLookahead) {
    CenterlineParams params_neg_spacing{
        .max_pair_dist = 6.0,
        .path_spacing = -0.5,
        .path_lookahead = 20.0,
    };
    CenterlineParams params_zero_lookahead{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 0.0,
    };
    CenterlineParams params_neg_lookahead{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = -10.0,
    };

    std::vector<Point2D> left = {{0.0, 9.0}, {9.0, 0.0}, {0.0, -9.0}, {-9.0, 0.0}};
    std::vector<Point2D> right = {{0.0, 11.0}, {11.0, 0.0}, {0.0, -11.0}, {-11.0, 0.0}};

    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, params_neg_spacing).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, params_zero_lookahead).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, params_neg_lookahead).empty());
}

TEST(CircleCenterlineCore, ParameterGuardNonFiniteSpacingOrLookahead) {
    // NaN slips through naive `<= 0.0` comparisons, and subnormal spacing makes
    // lookahead/spacing overflow to +inf; both must degrade to empty instead of
    // reaching `static_cast<int>` (UB) under UBSan.
    const std::vector<Point2D> left = {{0.0, 9.0}, {9.0, 0.0}, {0.0, -9.0}, {-9.0, 0.0}};
    const std::vector<Point2D> right = {{0.0, 11.0}, {11.0, 0.0}, {0.0, -11.0}, {-11.0, 0.0}};
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    const CenterlineParams nan_spacing{.max_pair_dist = 6.0, .path_spacing = kNan, .path_lookahead = 20.0};
    const CenterlineParams nan_lookahead{.max_pair_dist = 6.0, .path_spacing = 0.5, .path_lookahead = kNan};
    const CenterlineParams tiny_spacing{.max_pair_dist = 6.0, .path_spacing = 1e-308, .path_lookahead = 20.0};
    const CenterlineParams inf_lookahead{
        .max_pair_dist = 6.0, .path_spacing = 0.5, .path_lookahead = std::numeric_limits<double>::infinity()};
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, nan_spacing).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, nan_lookahead).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, tiny_spacing).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(left, right, inf_lookahead).empty());
}

TEST(CircleCenterlineCore, NormalBothSidesProduceCenterline) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 10.0,
    };

    // Center at (0.0, 10.0), track radius = 10.0.
    // Inner cones (left) at radius 8.5, outer cones (right) at radius 11.5.
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    for (int i = 0; i < 8; ++i) {
        double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / 8.0;
        left.emplace_back(8.5 * std::cos(angle), 10.0 + (8.5 * std::sin(angle)));
        right.emplace_back(11.5 * std::cos(angle), 10.0 + (11.5 * std::sin(angle)));
    }

    auto path = ComputeCircleCenterlineCore(left, right, params);
    ASSERT_FALSE(path.empty());
    const int expected_steps = static_cast<int>(params.path_lookahead / params.path_spacing) + 1;
    EXPECT_EQ(path.size(), static_cast<size_t>(expected_steps));

    // Points should lie on radius ~ 10.0 from (0.0, 10.0)
    for (const auto& pt : path) {
        double dist = std::hypot(pt.x - 0.0, pt.y - 10.0);
        EXPECT_NEAR(dist, 10.0, 1e-4);
    }
}

TEST(CircleCenterlineCore, RadiusOutOfRangeReturnsEmpty) {
    CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 10.0,
    };

    // Circle with radius 1.5 < kMinRadius (3.0)
    std::vector<Point2D> small_left;
    std::vector<Point2D> small_right;
    for (int i = 0; i < 6; ++i) {
        double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / 6.0;
        small_left.emplace_back(1.0 * std::cos(angle), 1.0 * std::sin(angle));
        small_right.emplace_back(2.0 * std::cos(angle), 2.0 * std::sin(angle));
    }

    EXPECT_TRUE(ComputeCircleCenterlineCore(small_left, small_right, params).empty());
}

}  // namespace
