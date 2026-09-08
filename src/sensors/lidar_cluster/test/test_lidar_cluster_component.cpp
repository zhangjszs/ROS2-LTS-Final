#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "lidar_cluster.h"
#include "lidar_cluster_component.hpp"

class LidarClusterComponentTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(LidarClusterComponentTest, ConstructWithoutBadWeakPtr) {
    // 验证以 std::make_shared 方式构造 LidarClusterComponent（模拟 ROS 2 ComponentManager 动态加载流程）
    // 在修复前，构造函数内部直接调用 shared_from_this() 会百分之百抛出 std::bad_weak_ptr 导致崩溃
    // C++20 现代架构改造后，通过传递非拥有型 node 指针 (this) 彻底消除了该问题
    rclcpp::NodeOptions options;
    std::shared_ptr<lidar_cluster::LidarClusterComponent> component;

    EXPECT_NO_THROW({
        component = std::make_shared<lidar_cluster::LidarClusterComponent>(options);
    });

    ASSERT_NE(component, nullptr);
    EXPECT_TRUE(component->isRunning());
    EXPECT_STREQ(component->get_name(), "lidar_cluster_node");
}

TEST_F(LidarClusterComponentTest, DirectLidarClusterInstantiation) {
    // 验证 LidarCluster 直接支持原生指针或引用构造，解除循环引用与强依赖
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("test_host_node", options);

    EXPECT_NO_THROW({
        LidarCluster lc(node.get());
    });
}
