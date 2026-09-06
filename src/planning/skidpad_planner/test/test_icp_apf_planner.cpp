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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
