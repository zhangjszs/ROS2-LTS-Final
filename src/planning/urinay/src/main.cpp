/**
 * @file main.cpp
 * @author yzh
 * @brief Urinay 主文件，创建所有模块、订阅者和发布者。
 * @version 1.0
 * @date 2022-10-31
 */

#include <chrono>
#include <common_msgs/msg/huat_cone.hpp>
#include <common_msgs/msg/huat_map.hpp>
#include <common_msgs/msg/huat_path_limits.hpp>
#include <common_msgs/msg/huat_tracklimits.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

#include "common_msgs/msg/huat_stop.hpp"
#include "modules/WayComputer.hpp"
#include "modules/delaunay_triangulator.hpp"
#include "modules/urinay_visualizer.hpp"
#include "race/race_director.h"
#include "utils/Time.hpp"

// wayComputer / params 在 main() 中创建，通过引用捕获传递给 lambda。
std::unique_ptr<WayComputer> g_wayComputer;
std::unique_ptr<UrinayParams> g_params;
std::mutex g_planningMutex;

/**
 * @brief 地图回调 — 处理锥桶地图、运行路径规划、发布结果。
 */
void OnConeMapMessage(const rclcpp::Node::SharedPtr& node, const common_msgs::msg::HuatMap::ConstSharedPtr& data,
                      rclcpp::Publisher<common_msgs::msg::HuatPathLimits>::SharedPtr& pubPartial,
                      rclcpp::Publisher<common_msgs::msg::HuatStop>::SharedPtr& stopPub,
                      const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& profPub,
                      std::unique_ptr<RaceDirector>& raceDirector) {
    auto t_start = std::chrono::steady_clock::now();

    std::unique_lock<std::mutex> planning_lock(g_planningMutex, std::try_to_lock);
    if (!planning_lock.owns_lock()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "[urinay] Planner is still processing previous cone map, dropping stale frame");
        return;
    }

    if (not g_wayComputer->isLocalTfValid()) {
        RCLCPP_WARN(node->get_logger(), "[urinay] Vehicle state not received");
        return;
    }
    if (data->cone.empty()) {
        RCLCPP_WARN(node->get_logger(), "[urinay] Cone set is empty");
        return;
    }

    // 赛事总监：更新圈数计数并处理停止信号
    const auto car_state = g_wayComputer->getCarState().car_state;
    if (raceDirector->update(car_state.x, car_state.y)) {
        if (raceDirector->shouldPublishStop()) {
            common_msgs::msg::HuatStop msg;
            msg.stop = true;
            stopPub->publish(msg);
            raceDirector->markStopPublished();
            if (raceDirector->lapCount() == g_params->main.number_of_stopped_turns + 1) {
                RCLCPP_INFO(node->get_logger(),
                            "[urinay] Race finished, stop requested; keeping planner alive to avoid path dropout");
            } else {
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                     "[urinay] Race finished, re-publishing stop request (heartbeat)");
            }
        }
    }

    // 锥筒坐标 → 节点
    std::vector<Node> nodes;
    nodes.reserve(data->cone.size());
    for (const common_msgs::msg::HuatCone& c : data->cone) {
        nodes.emplace_back(c);
    }

    // 计算局部坐标
    const Eigen::Affine3d local_tf = g_wayComputer->getLocalTf();
    for (const Node& n : nodes) {
        n.updateLocal(local_tf);
    }

    // 三角剖分
    auto t_tri0 = std::chrono::steady_clock::now();
    TriangleSet triangles = DelaunayTriangulator::compute(nodes);
    auto t_tri1 = std::chrono::steady_clock::now();
    double tri_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_tri1 - t_tri0).count() / 1000.0;

    // 路径规划
    auto t_way0 = std::chrono::steady_clock::now();
    g_wayComputer->update(triangles, data->header.stamp);
    auto t_way1 = std::chrono::steady_clock::now();
    double way_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_way1 - t_way0).count() / 1000.0;

    // 发布路径
    auto t_pub0 = std::chrono::steady_clock::now();
    pubPartial->publish(g_wayComputer->getPathLimitsGlobal(static_cast<PathMode>(g_params->main.the_mode_of_path)));

    if (g_wayComputer->isLoopClosed() && g_params->main.shutdown_on_loop_closure) {
        RCLCPP_INFO(node->get_logger(), "[urinay] Loop closure detected, shutting down as configured");
        rclcpp::shutdown();
    }
    auto t_pub1 = std::chrono::steady_clock::now();
    double pub_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_pub1 - t_pub0).count() / 1000.0;

    auto t_end = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;

    // 性能分析发布
    if (g_params->main.enable_profiling && profPub && profPub->get_subscription_count() > 0) {
        std_msgs::msg::Float64MultiArray arr;
        arr.data = {tri_ms, way_ms, pub_ms, total_ms};
        profPub->publish(arr);
    }
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                         "[urinay] Profiling (ms) | Triangulation=%.2f WayCompute=%.2f PathGen=%.2f Total=%.2f", tri_ms,
                         way_ms, pub_ms, total_ms);
}

// 主函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("urinay");
    g_params = std::make_unique<UrinayParams>(node.get());
    g_wayComputer = std::make_unique<WayComputer>(g_params->wayComputer);
    auto raceDirector = std::make_unique<RaceDirector>(g_params->main.number_of_stopped_turns, g_params->main.start_x,
                                                       g_params->main.start_y);
    RCLCPP_INFO(node->get_logger(), "[urinay] RaceDirector start point: (%.2f, %.2f)", g_params->main.start_x,
                g_params->main.start_y);
    UrinayVisualizer::getInstance().init(node, g_params->visualization);

    // 发布者（main 局部，通过引用传递给回调）
    rclcpp::Publisher<common_msgs::msg::HuatPathLimits>::SharedPtr pubPartial =
        node->create_publisher<common_msgs::msg::HuatPathLimits>(g_params->main.output_topic, 1);
    rclcpp::Publisher<common_msgs::msg::HuatStop>::SharedPtr stopPub =
        node->create_publisher<common_msgs::msg::HuatStop>(g_params->main.stop_topic, 1);
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr profPub;
    if (g_params->main.enable_profiling) {
        profPub = node->create_publisher<std_msgs::msg::Float64MultiArray>(g_params->main.profiling_topic, 10);
        RCLCPP_INFO(node->get_logger(), "[urinay] Profiling enabled (topic=%s)",
                    g_params->main.profiling_topic.c_str());
    }

    // 锥桶地图订阅
    auto subCones = node->create_subscription<common_msgs::msg::HuatMap>(
        g_params->main.input_cones_topic, 1, [&, node](const common_msgs::msg::HuatMap::ConstSharedPtr& data) {
            OnConeMapMessage(node, data, pubPartial, stopPub, profPub, raceDirector);
        });

    // 车辆位姿订阅
    auto subPose = node->create_subscription<common_msgs::msg::HuatCarstate>(
        g_params->main.input_pose_topic, 1,
        [](const common_msgs::msg::HuatCarstate::ConstSharedPtr& data) { g_wayComputer->stateCallback(data); });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    return 0;
}
