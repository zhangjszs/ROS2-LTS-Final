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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
