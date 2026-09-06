#include <gtest/gtest.h>

#include "race/race_director.h"

TEST(RaceDirector, StartingAtOriginDoesNotCountALap) {
    RaceDirector rd(1, 0.0, 0.0, 2.0, 6.0);
    EXPECT_FALSE(rd.update(0.0, 0.0));
    EXPECT_EQ(rd.lapCount(), 0);
}

TEST(RaceDirector, LeavingStartDoesNotCount) {
    RaceDirector rd(1, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    EXPECT_FALSE(rd.update(10.0, 0.0));
    EXPECT_EQ(rd.lapCount(), 0);
}

TEST(RaceDirector, ReturningAfterLeavingCountsOneLap) {
    RaceDirector rd(1, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    rd.update(10.0, 0.0);
    EXPECT_FALSE(rd.update(0.2, 0.1));
    EXPECT_EQ(rd.lapCount(), 1);
}

TEST(RaceDirector, SecondReturnEndsRaceWhenLimitIsOne) {
    RaceDirector rd(1, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    rd.update(10.0, 0.0);
    rd.update(0.0, 0.0);
    rd.update(10.0, 0.0);
    EXPECT_TRUE(rd.update(0.0, 0.0));
    EXPECT_EQ(rd.lapCount(), 2);
}

TEST(RaceDirector, OscillatingInsideExitRadiusDoesNotCountExtraLap) {
    RaceDirector rd(2, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    rd.update(10.0, 0.0);
    rd.update(0.0, 0.0);  // lap 1
    EXPECT_EQ(rd.lapCount(), 1);
    EXPECT_FALSE(rd.update(4.0, 0.0));  // still inside exit radius
    EXPECT_FALSE(rd.update(0.2, 0.0));  // re-enter without leaving
    EXPECT_EQ(rd.lapCount(), 1);
}

TEST(RaceDirector, MustLeaveBeyondExitRadiusBeforeNextLap) {
    RaceDirector rd(2, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    rd.update(10.0, 0.0);
    rd.update(0.0, 0.0);
    rd.update(4.0, 0.0);
    EXPECT_EQ(rd.lapCount(), 1);
    rd.update(10.0, 0.0);
    EXPECT_FALSE(rd.update(0.0, 0.0));
    EXPECT_EQ(rd.lapCount(), 2);
}

TEST(RaceDirector, CrossingBesideOriginStillCountsWithWideGate) {
    RaceDirector rd(0, 0.0, 0.0, 2.0, 6.0);
    rd.update(0.0, 0.0);
    rd.update(8.0, 4.0);
    EXPECT_TRUE(rd.update(0.5, 1.2));
    EXPECT_EQ(rd.lapCount(), 1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
