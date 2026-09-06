#include <gtest/gtest.h>

#include "structures/Way.hpp"

TEST(WayEmpty, DefaultIsEmpty) {
    Way way;
    EXPECT_TRUE(way.empty());
    EXPECT_EQ(way.size(), 0u);
}

TEST(WayEmpty, InterpolationDoesNotCrash) {
    Way way;
    EXPECT_TRUE(way.getPathInterpolation(0.0, 0.0).empty());
    EXPECT_TRUE(way.getPathInterpolationLocal(0.0, 0.0).empty());
    EXPECT_TRUE(way.getPathFullInterpolation().empty());
    EXPECT_TRUE(way.getPathFullInterpolationLocal().empty());
}

TEST(WayEmpty, DeletePassedIsNoOp) {
    Way way;
    way.deleteWayPassed();
    EXPECT_TRUE(way.empty());
}

TEST(WayEmpty, NextPathPointIsOrigin) {
    Way way;
    Point p = way.getNextPathPoint();
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
}

TEST(WayEmpty, QuinEhOnTwoEmptyWaysIsFalse) {
    Way a, b;
    EXPECT_FALSE(a.quinEhLobjetiuDeLaSevaDiresio(b));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
