#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "line_detector.hpp"
#include "straight_line_geom.h"

static common_msgs::msg::HuatCone MakeCone(double x, double y) {
    common_msgs::msg::HuatCone c;
    c.position_base_link.x = x;
    c.position_base_link.y = y;
    return c;
}

TEST(GlobalDeltaToBaseLink, IdentityAtZeroHeading) {
    double bx = 0, by = 0;
    GlobalDeltaToBaseLink(3.0, 4.0, 0.0, &bx, &by);
    EXPECT_NEAR(bx, 3.0, 1e-12);
    EXPECT_NEAR(by, 4.0, 1e-12);
}

TEST(GlobalDeltaToBaseLink, RotatesWithVehicleHeading) {
    double bx = 0, by = 0;
    GlobalDeltaToBaseLink(1.0, 0.0, M_PI / 2.0, &bx, &by);
    EXPECT_NEAR(bx, 0.0, 1e-12);
    EXPECT_NEAR(by, -1.0, 1e-12);
}

TEST(LineDetector, EmptyInputFails) {
    LineDetectorConfig cfg;
    cfg.enable_temporal_filter = false;
    cfg.enable_hough = false;
    LineDetector detector(cfg);
    DetectedBoundaries b = detector.Detect({});
    EXPECT_FALSE(b.success);
    EXPECT_FALSE(b.left.valid);
    EXPECT_FALSE(b.right.valid);
}

TEST(LineDetector, ParallelConesFitBothSides) {
    LineDetectorConfig cfg;
    cfg.enable_temporal_filter = false;
    cfg.enable_hough = false;
    LineDetector detector(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 1; i <= 6; ++i) {
        cones.push_back(MakeCone(static_cast<double>(i), -1.5));
        cones.push_back(MakeCone(static_cast<double>(i), 1.5));
    }
    DetectedBoundaries b = detector.Detect(cones);
    EXPECT_TRUE(b.success);
    EXPECT_TRUE(b.left.valid);
    EXPECT_TRUE(b.right.valid);
    EXPECT_NEAR(b.left.slope, 0.0, 0.05);
    EXPECT_NEAR(b.right.slope, 0.0, 0.05);
    EXPECT_LT(b.left.intercept, 0.0);
    EXPECT_GT(b.right.intercept, 0.0);
}

TEST(LineDetector, OneSidedDoesNotReportSuccess) {
    LineDetectorConfig cfg;
    cfg.enable_temporal_filter = false;
    cfg.enable_hough = false;
    LineDetector detector(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 1; i <= 6; ++i)
        cones.push_back(MakeCone(static_cast<double>(i), -1.5));
    DetectedBoundaries b = detector.Detect(cones);
    EXPECT_FALSE(b.success);
    EXPECT_TRUE(b.left.valid);
    EXPECT_FALSE(b.right.valid);
}

TEST(LineDetector, ClusterConesWithRanges) {
    LineDetectorConfig cfg;
    cfg.center_margin = 0.3;
    LineDetector detector(cfg);

    std::vector<common_msgs::msg::HuatCone> cones = {
        MakeCone(2.0, -1.8),
        MakeCone(3.0, 1.2),
        MakeCone(1.0, 0.0),   // near center
        MakeCone(4.0, -1.5),
        MakeCone(5.0, 1.6)
    };

    std::vector<common_msgs::msg::HuatCone> left, right;
    detector.ClusterCones(cones, left, right);

    EXPECT_EQ(left.size(), 2u);
    EXPECT_EQ(right.size(), 2u);
    EXPECT_NEAR(left[0].position_base_link.y, -1.8, 1e-5);
    EXPECT_NEAR(left[1].position_base_link.y, -1.5, 1e-5);
    EXPECT_NEAR(right[0].position_base_link.y, 1.2, 1e-5);
    EXPECT_NEAR(right[1].position_base_link.y, 1.6, 1e-5);
}

TEST(LineDetector, HoughFitWithRangesMaxElement) {
    LineDetectorConfig cfg;
    cfg.enable_hough = true;
    cfg.hough_min_inlier_ratio = 0.5;
    LineDetector detector(cfg);

    // Cones strictly along y = 0.1 * x + 1.5
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 1; i <= 8; ++i) {
        double x = static_cast<double>(i) * 2.0;
        double y = 0.1 * x + 1.5;
        cones.push_back(MakeCone(x, y));
    }

    LineParams params = detector.HoughFit(cones);
    EXPECT_TRUE(params.valid);
    EXPECT_NEAR(params.slope, 0.1, 0.1);
    EXPECT_NEAR(params.intercept, 1.5, 0.5);
}

TEST(LineDetector, RansacFitWithRangesSorting) {
    LineDetectorConfig cfg;
    LineDetector detector(cfg);

    // Cones along y = -2.0 with slight jitter and out-of-order x values
    std::vector<common_msgs::msg::HuatCone> cones = {
        MakeCone(8.0, -2.01),
        MakeCone(2.0, -1.99),
        MakeCone(6.0, -2.0),
        MakeCone(4.0, -2.0),
        MakeCone(10.0, -2.0)
    };

    LineParams params = detector.RansacFit(cones);
    EXPECT_TRUE(params.valid);
    EXPECT_NEAR(params.slope, 0.0, 0.05);
    EXPECT_NEAR(params.intercept, -2.0, 0.1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
