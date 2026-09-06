/**
 * @file urinay_visualizer.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 UrinayVisualizer 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "structures/Way.hpp"
#include "utils/definitions.hpp"
#include "utils/urinay_params.hpp"

/**
 * @brief 实现所有必要函数以可视化程序所有结果的类。
 */
class UrinayVisualizer {
   private:
    /**
     * @brief 所有 Marker 的发布器。
     */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trianglesPub, midpointsPub, wayPub;

    /**
     * @brief 所有与可视化类相关的参数。
     */
    UrinayParams::Visualization params_;

    /**
     * @brief 所有 Marker 将使用此时间戳发布。
     */
    builtin_interfaces::msg::Time stamp_;

   public:
    /**
     * @brief 构造一个新的可视化对象。
     */
    UrinayVisualizer() = default;

    // 单例模式
    static UrinayVisualizer &getInstance();
    UrinayVisualizer(UrinayVisualizer const &) = delete;
    void operator=(UrinayVisualizer const &) = delete;

    /**
     * @brief 初始化单例的方法。
     *
     * @param[in] node
     * @param[in] params
     */
    void init(rclcpp::Node::SharedPtr node, const UrinayParams::Visualization &params);

    /**
     * @brief 设置 stamp_ 属性，所有 Marker 将使用此时间戳发布。
     *
     * @param[in] stamp
     */
    void setTimestamp(const rclcpp::Time &stamp);

    /**
     * @brief 可视化 TriangleSet 的方法。
     *
     * @param[in] triSet
     */
    void visualize(const TriangleSet &triSet) const;

    /**
     * @brief 可视化 EdgeSet 的方法。
     *
     * @param[in] edgeSet
     */
    void visualize(const EdgeSet &edgeSet) const;

    /**
     * @brief 可视化 Way 的方法。
     *
     * @param[in] way
     */
    void visualize(const Way &way) const;
};
