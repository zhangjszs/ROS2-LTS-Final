/**
 * @file Triangle.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Triangle 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <compare>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "structures/Circle.hpp"
#include "structures/Edge.hpp"
#include "structures/Node.hpp"

/**
 * @brief 表示一个三角形，包含所有便于 Delaunay
 * 三角剖分计算的元素。
 */
class Triangle {
   public:
    /**
     * @brief 定义一个三角形的 3 个节点。
     */
    const std::array<Node, 3> nodes;

    /**
     * @brief 由 3 个节点定义的边。这个对象基本上包含
     * 与 nodes 相同的信息，但它便于访问三角形的边
     * 并且通过少量额外的内存使程序运行更快。
     * 这是对变量用途和优化的说明。它指出，这个对象基本上包含了与 `nodes` 相同的信息，
     * 但它方便了对三角形（`Triangle`）的边进行访问，并且通过少量额外的内存消耗使程序运行更快
     */
    const std::array<Edge, 3> edges;

   private:
    /**
     * @brief 三角形的外接圆。
     */
    const Circle circumCircle_;

    /**
     * @brief 三角形的唯一哈希值。哈希值使用节点的 ID 计算得出。
     */
    const uint64_t hash_;

    /**
     * @brief 返回以 n0、n1 和 n2 为节点的三角形所具有的哈希值。
     *
     * @param[in] n0
     * @param[in] n1
     * @param[in] n2
     */
    static uint64_t computeHash(const Node& n0, const Node& n1, const Node& n2);
    friend class std::hash<Triangle>;

   public:
    /**
     * @brief 从 3 个节点构造一个新的 Triangle 对象。
     *
     * @param[in] n0
     * @param[in] n1
     * @param[in] n2
     */
    Triangle(const Node& n0, const Node& n1, const Node& n2);

    /**
     * @brief 从一条边和一个节点构造一个新的 Triangle 对象。
     *
     * @param[in] e
     * @param[in] n
     */
    Triangle(const Edge& e, const Node& n);

    /**
     * @brief C++20 宇宙飞船操作符 <=>。
     * 根据三角形唯一哈希值 hash_ 进行全序三路比较，返回 std::strong_ordering。
     * 自动合成 <, <=, >, >=，使 Triangle 可直接用于排序与有序容器。
     */
    [[nodiscard]] constexpr std::strong_ordering operator<=>(const Triangle& t) const noexcept {
        return this->hash_ <=> t.hash_;
    }

    /**
     * @brief 比较运算符，如果两个三角形的哈希值相同，则它们相等。
     * C++20 自动合成 != 运算符。
     */
    [[nodiscard]] constexpr bool operator==(const Triangle& t) const noexcept { return this->hash_ == t.hash_; }

    /**
     * @brief 检查三角形是否包含节点 n。
     *
     * @param[in] n
     */
    bool containsNode(const Node& n) const;

    /**
     * @brief 检查边 e 是否属于该三角形。
     *
     * @param[in] e
     */
    bool containsEdge(const Edge& e) const;

    /**
     * @brief 检查一个节点是否在三角形的外接圆内部。
     *
     * @param[in] n
     */
    bool circleContainsNode(const Node& n) const;

    /**
     * @brief 检查三角形的任何节点是否属于超级三角形。
     */
    bool anyNodeInSuperTriangle() const;

    /**
     * @brief 返回包含三角形 3 个角度的数组。
     */
    std::array<double, 3> angles() const;

    /**
     * @brief 返回局部坐标系下的外心。
     */
    const Point& circumCenter() const;

    /**
     * @brief 返回全局坐标系下的外心。
     */
    const Point& circumCenterGlobal() const;

    /**
     * @brief 输出流运算符。
     *
     * @param[in,out] os
     * @param[in] t
     */
    friend std::ostream& operator<<(std::ostream& os, const Triangle& t);

    int getHash() const { return hash_; }
};

template <>
struct std::hash<Triangle> {
    uint64_t operator()(const Triangle& t) const { return t.hash_; }
};
