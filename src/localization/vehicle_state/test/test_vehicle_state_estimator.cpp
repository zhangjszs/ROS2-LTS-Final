// issue #12：设备时间重复帧判定单元测试（纯静态函数，无需 ROS context）
#include <gtest/gtest.h>

#include "vehicle_state_estimator.h"

constexpr auto ShouldRejectDuplicateDeviceTime = &VehicleStateEstimator::ShouldRejectDuplicateDeviceTime;

TEST(ShouldRejectDuplicateDeviceTimeTest, DisabledNeverRejects) {
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(false, 2300, 100.5, 2300, 100.5));
}

TEST(ShouldRejectDuplicateDeviceTimeTest, IdenticalDeviceTimeRejectedWhenEnabled) {
    EXPECT_TRUE(ShouldRejectDuplicateDeviceTime(true, 2300, 100.5, 2300, 100.5));
}

TEST(ShouldRejectDuplicateDeviceTimeTest, AdvancedDeviceTimeAccepted) {
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(true, 2300, 100.5, 2300, 100.6));
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(true, 2300, 100.5, 2301, 0.1));
}

TEST(ShouldRejectDuplicateDeviceTimeTest, UnknownSentinelFieldsNeverReject) {
    // 未填/未知（-1）字段不得参与判定，避免设备时间语义标定前误丢合法帧
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(true, -1.0, -1.0, -1.0, -1.0));
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(true, 2300, 100.5, 2300, -1.0));
    EXPECT_FALSE(ShouldRejectDuplicateDeviceTime(true, -1.0, 100.5, 2300, 100.5));
}
