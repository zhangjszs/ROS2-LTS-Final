/**
 * @file Node.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Node 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <compare>
#include <iostream>

#include <common_msgs/msg/huat_cone.hpp>
#include <cone_types.h>
#include "structures/Point.hpp"
#include "structures/Vector.hpp"
#include "utils/constants.hpp"

/**
 * @brief 表示一个节点，即 as_msgs::Cone 中有用的部分
 *（位置和 ID）。
 */
class Node {
   public:
    /**
     * @brief 节点的 ID，原则上由锥桶追踪器给出。
     */
    const uint32_t id;

   private:
    /**
     * @brief 超级三角形的节点必须有一个 ID，这个 ID 不能干扰
     * 现有节点的 ID，为了解决这个问题，所有属于超级三角形的
     * 节点都将被分配一个巨大的 ID。
     * 参见 utils/constants.hpp/HASH_SHIFT_NUM 了解为何是这个值。
     */
    static const uint32_t SUPERTRIANGLE_BASEID = (1 << HASH_SHIFT_NUM) - 3;

    /**
     * @brief 当前超级三角形节点编号。
     */
    static uint32_t superTriangleNodeNum;

    /**
     * @brief 局部坐标系下的锥桶点。
     * **注意** 它是 mutable（可变的）。
     * 表示该变量是可变的（mutable）。即使在一个常量成员函数中，这个可变变量也可以被修改。
     */
    mutable Point point_;

    /**
     * @brief 全局坐标系下的锥桶点。
     */
    const Point pointGlobal_;

    /**
     * @brief 该节点是否属于超级三角形。
     */
    const bool belongsToSuperTriangle_;

    /**
     * @brief 构造一个新的 Node 对象，该对象将作为超级三角形。
     *
     * @param[in] x
     * @param[in] y
     */
    Node(const double &x, const double &y);

   public:

    /**
     * @brief 构造一个新的 Node 对象。
     *
     * @param[in] x
     * @param[in] y
     * @param[in] xGlobal
     * @param[in] yGlobal
     * @param[in] id
     */
    Node(const double &x, const double &y, const double &xGlobal, const double &yGlobal, const uint32_t &id);

    /**
     * @brief 从 as_msgs::Cone 构造一个新的 Node 对象。
     *
     * @param[in] c
     */
    Node(const common_msgs::msg::HuatCone &c);

    /**
     * @brief 返回节点的局部 x 坐标。
     */
    const double &x() const;

    /**
     * @brief 返回节点的局部 y 坐标。
     */
    const double &y() const;

    /**
     * @brief C++20 宇宙飞船操作符 <=>。
     * 当且仅当两个节点的 ID 相同时判定相同/顺序，返回 std::strong_ordering 全序。
     * 自动合成 <, <=, >, >=。
     */
    [[nodiscard]] constexpr std::strong_ordering operator<=>(const Node &n) const noexcept {
        return this->id <=> n.id;
    }

    /**
     * @brief 比较运算符。当且仅当两个节点的 ID 相同时，它们相等。
     * C++20 自动合成 != 运算符。
     */
    [[nodiscard]] constexpr bool operator==(const Node &n) const noexcept {
        return this->id == n.id;
    }

    /**
     * @brief 返回一个局部坐标为 (x, y) 的超级三角形节点。
     *
     * @param[in] x
     * @param[in] y
     */
    static Node superTriangleNode(const double &x, const double &y);

    /**
     * @brief 检查该节点是否属于超级三角形。
     */
    const bool &belongsToSuperTriangle() const;

    /**
     * @brief 更新节点的局部坐标。
     *
     * @param[in] tf
     */
    void updateLocal(const Eigen::Affine3d &tf) const;

    /**
     * @brief 返回局部坐标系下的点。
     */
    const Point &point() const;

    /**
     * @brief 返回全局坐标系下的点。
     */
    const Point &pointGlobal() const;

    /**
     * @brief 返回从节点局部点到 p 的距离平方。
     *
     * @param[in] p
     */
    double distSq(const Point &p) const;

    /**
     * @brief 返回该节点与节点 n0 和 n1 形成的角度。
     *
     * @param[in] n0
     * @param[in] n1
     */
    double angleWith(const Node &n0, const Node &n1) const;

    /**
     * @brief 将节点转换为 as_msgs::Cone 并返回。
     */
    common_msgs::msg::HuatCone cone() const;

    /**
     * @brief 输出流运算符。
     *
     * @param[in,out] os
     * @param[in] n
     */
    friend std::ostream &operator<<(std::ostream &os, const Node &n);
};
