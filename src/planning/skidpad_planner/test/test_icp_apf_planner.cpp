#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "icp_apf_planner.hpp"

static common_msgs::msg::HuatCone MakeCone(double x, double y) {
    common_msgs::msg::HuatCone c;
    c.position_base_link.x = x;
    c.position_base_link.y = y;
    return c;
}

TEST(IcpApfPlanner, EmptyConesReturnsPreviousPath) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<skidpad::Point2D> prev = {skidpad::Point2D(1.0, 0.0), skidpad::Point2D(2.0, 0.0)};
    auto path = planner.GeneratePath({}, prev);
    ASSERT_EQ(path.size(), 2u);
    EXPECT_NEAR(path[0].x, 1.0, 1e-12);
    EXPECT_NEAR(path[1].x, 2.0, 1e-12);
}

TEST(IcpApfPlanner, OneSidedConesReturnsPreviousPath) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 0; i < 6; ++i)
        cones.push_back(MakeCone(static_cast<double>(i), 1.5));
    auto path = planner.GeneratePath(cones, {});
    EXPECT_TRUE(path.empty());
}

TEST(IcpApfPlanner, OppositeSidesProduceForwardPath) {
    skidpad::IcpApfConfig cfg;
    cfg.path_lookahead = 20.0;
    cfg.path_spacing = 0.5;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 1; i <= 8; ++i) {
        cones.push_back(MakeCone(static_cast<double>(i), -1.5));
        cones.push_back(MakeCone(static_cast<double>(i), 1.5));
    }
    auto path = planner.GeneratePath(cones, {});
    ASSERT_FALSE(path.empty());
    EXPECT_GT(path.front().x, 0.0);
}

TEST(IcpApfPlanner, ClusterConesRangesPartitioning) {
    skidpad::IcpApfConfig cfg;
    cfg.center_margin = 0.3;
    skidpad::IcpApfPlanner planner(cfg);

    std::vector<common_msgs::msg::HuatCone> cones = {
        MakeCone(1.0, -1.5),  // left
        MakeCone(2.0, 1.5),   // right
        MakeCone(3.0, -0.2),  // within margin, should be skipped
        MakeCone(4.0, 0.1),   // within margin, should be skipped
        MakeCone(5.0, -2.0),  // left
        MakeCone(6.0, 2.5)    // right
    };

    std::vector<common_msgs::msg::HuatCone> left, right;
    planner.ClusterCones(cones, left, right);

    EXPECT_EQ(left.size(), 2u);
    EXPECT_EQ(right.size(), 2u);
    EXPECT_DOUBLE_EQ(left[0].position_base_link.y, -1.5);
    EXPECT_DOUBLE_EQ(left[1].position_base_link.y, -2.0);
    EXPECT_DOUBLE_EQ(right[0].position_base_link.y, 1.5);
    EXPECT_DOUBLE_EQ(right[1].position_base_link.y, 2.5);
}

TEST(IcpApfPlanner, TransformPointsRanges) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);

    std::vector<skidpad::Point2D> pts = {
        {1.0, 0.0},
        {0.0, 1.0}
    };

    // Rotate 90 deg counter-clockwise, translate dx=2.0, dy=3.0
    double rot = M_PI / 2.0;
    auto transformed = planner.TransformPoints(pts, rot, 2.0, 3.0);

    ASSERT_EQ(transformed.size(), 2u);
    EXPECT_NEAR(transformed[0].x, 2.0, 1e-6);
    EXPECT_NEAR(transformed[0].y, 4.0, 1e-6);
    EXPECT_NEAR(transformed[1].x, 1.0, 1e-6);
    EXPECT_NEAR(transformed[1].y, 3.0, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
