// #54：本文件只校验 SensorSimulator 的**适配层**——纯 std 感知几何核（FOV/距离/坐标/噪声）
// 的断言已移至 test_sensor_model_core.cpp（进 #40 的 ASan+UBSan 门）。本文件依赖 ROS 生成
// 消息（HuatMap/HuatCone），故仅由 colcon 轨运行。
#include <gtest/gtest.h>

#include <vector>

#include "vehicle_simulator/sensor_simulator.hpp"

using namespace simulation;

// 只验证"中性核结果 → HuatMap 消息"的组装是否正确（过滤/几何由 test_sensor_model_core 覆盖）。
TEST(SensorSimulatorTest, AssemblesDetectedConesIntoHuatMap) {
    SensorSimulator sim;
    sim.SetTrackCones({
        TrackCone{.x = 5.0, .y = 0.0, .type = 0, .id = 1},   // 应可见
        TrackCone{.x = 25.0, .y = 0.0, .type = 1, .id = 4},  // 超距，应被过滤
    });

    const VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    const auto detected = sim.GeneratePerceivedCones(car, 120.0, 15.0, 0.0);

    ASSERT_EQ(detected.cone.size(), 1u);
    EXPECT_EQ(detected.cone[0].id, 1u);
    EXPECT_EQ(detected.cone[0].type, 0u);
    EXPECT_EQ(detected.cone[0].confidence, 95u);
    EXPECT_NEAR(detected.cone[0].position_base_link.x, 5.0f, 1e-3);
    EXPECT_NEAR(detected.cone[0].position_base_link.y, 0.0f, 1e-3);
    EXPECT_FLOAT_EQ(detected.cone[0].position_base_link.z, 0.0f);
    EXPECT_NEAR(detected.cone[0].position_global.x, 5.0f, 1e-3);
    EXPECT_FLOAT_EQ(detected.cone[0].position_global.z, 0.0f);
}
