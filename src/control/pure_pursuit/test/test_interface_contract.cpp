// issue #14：统一接口契约校验器单元测试（纯 std，无 ROS context）。
#include <gtest/gtest.h>

#include <limits>

#include "interface_contract.h"  // 仓库约定：common_msgs 手写头不带前缀（同 cone_types.h / steering_calibration.h）

using namespace common_msgs::contract;

TEST(InterfaceContract, FrameAllowList) {
    EXPECT_TRUE(isFrameSupported("map"));
    EXPECT_TRUE(isFrameSupported("base_link"));
    EXPECT_FALSE(isFrameSupported("odom"));
    EXPECT_FALSE(isFrameSupported(""));
}

TEST(InterfaceContract, GeometryValidRejectsTooFewAndNonFinite) {
    const double xs2[2] = {0.0, 1.0};
    const double ys2[2] = {0.0, 0.0};
    EXPECT_TRUE(geometryValid({xs2, 2}, {ys2, 2}));

    const double one[1] = {0.0};
    EXPECT_FALSE(geometryValid({one, 1}, {one, 1}));  // 少于 2 点

    const double nan[2] = {0.0, std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(geometryValid({xs2, 2}, {nan, 2}));  // 坐标非有限

    const double ys3[3] = {0, 0, 0};
    EXPECT_FALSE(geometryValid({xs2, 2}, {ys3, 3}));  // 长度不一致
}

TEST(InterfaceContract, ZeroSpeedIsLegalButMissingIsNot) {
    // 允许合法 0 m/s（停车），与“缺失”严格区分
    const double stop[2] = {0.0, 0.0};
    EXPECT_TRUE(targetSpeedsEffective({stop, 2}, 2));
    EXPECT_FALSE(std::isnan(stop[0]));       // 合法 0 不是 NaN 哨兵
    EXPECT_TRUE(std::isnan(kUnknownSpeed));  // 缺失用 NaN 哨兵
    EXPECT_NE(kUnknownSpeed, 0.0);
}

TEST(InterfaceContract, TargetSpeedsRequiresEqualLengthFiniteNonNegative) {
    const double ok[3] = {5.0, 0.0, 3.0};
    EXPECT_TRUE(targetSpeedsEffective({ok, 3}, 3));
    EXPECT_FALSE(targetSpeedsEffective({ok, 2}, 3));  // 短于路径 => 缺失
    const double neg[2] = {5.0, -0.1};
    EXPECT_FALSE(targetSpeedsEffective({neg, 2}, 2));  // 含负值 => 不可信
    const double nan[2] = {5.0, std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(targetSpeedsEffective({nan, 2}, 2));  // 含 NaN => 不可信
    EXPECT_FALSE(targetSpeedsEffective({}, 0));        // 空路径
}

TEST(InterfaceContract, SourceAgeGate) {
    EXPECT_TRUE(sourceAgeAcceptable(0.05, 0.5));   // 新鲜
    EXPECT_FALSE(sourceAgeAcceptable(0.9, 0.5));   // 超龄
    EXPECT_FALSE(sourceAgeAcceptable(-0.1, 0.5));  // 非法未来/负年龄
    EXPECT_FALSE(sourceAgeAcceptable(std::numeric_limits<double>::quiet_NaN(), 0.5));
    EXPECT_TRUE(sourceAgeAcceptable(999.0, -1.0));  // tolerance<0 => 检查禁用
}

TEST(InterfaceContract, QosDescriptorsEncodeLatchContract) {
    EXPECT_TRUE(kQosStop.transient_local);    // 停车话题锁存（#8）
    EXPECT_FALSE(kQosState.transient_local);  // 状态话题不锁存
    EXPECT_TRUE(kQosStop.reliable);
    EXPECT_EQ(kQosStop.depth, 1u);
}
