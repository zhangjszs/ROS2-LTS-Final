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

// #14 项②：速度载体唯一为 target_speeds；Point.z 从不参与速度选择。
TEST(InterfaceContract, ReferenceSpeedNeverReadsPointZ) {
    // 旧约定把速度藏在 Point.z=5.0；新契约下，当 target_speeds 缺失（如空）时，
    // selectReferenceSpeed 只能返回“配置的默认参考速度”，绝不会因 z=5.0 而返回 5.0。
    const double legacy_point_z = 5.0;  // 仅作几何占位，任何路径都不得当速度读取
    const double configured_default = 2.5;

    // 1) 缺失显式速度：走默认参考速度分支（valid 当 >0.1）
    const auto missing = selectReferenceSpeed(/*has_explicit=*/false, /*target_speed=*/legacy_point_z,
                                              /*default_ref=*/configured_default);
    EXPECT_DOUBLE_EQ(missing.speed, configured_default);
    EXPECT_TRUE(missing.valid);
    EXPECT_NE(missing.speed, legacy_point_z);  // 关键：未采纳 z

    // 2) 有显式 target_speeds：逐点用该值（允许 0=停车），即使与 z 不同
    const double stop_speed = 0.0;
    const auto explicit_stop = selectReferenceSpeed(/*has_explicit=*/true, /*target_speed=*/stop_speed,
                                                    /*default_ref=*/configured_default);
    EXPECT_DOUBLE_EQ(explicit_stop.speed, 0.0);
    EXPECT_TRUE(explicit_stop.valid);  // 0 是合法停车目标，不等于缺失

    // 3) 默认参考速度过小（<=0.1）时 valid=false，触发降级/停车，而非静默行驶
    const auto weak_default = selectReferenceSpeed(/*has_explicit=*/false, /*target_speed=*/0.0,
                                                   /*default_ref=*/0.05);
    EXPECT_FALSE(weak_default.valid);

    // 4) 缺失且默认值为 NaN 哨兵时不可信
    const auto nan_default = selectReferenceSpeed(/*has_explicit=*/false, /*target_speed=*/0.0, kUnknownSpeed);
    EXPECT_FALSE(nan_default.valid);
}
