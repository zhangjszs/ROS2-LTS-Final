/**
 * @file Circle.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Circle 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cmath>
#include <compare>
#include <iostream>

#include "structures/Node.hpp"

/**
 * @brief 表示二维坐标系中的一个圆。
 */
class Circle {
   private:
    /**
     * @brief 圆心，包含局部坐标和全局坐标。
     */
    Point center_, centerGlobal_;

    /**
     * @brief 圆的半径的平方。
     */
    double radSq_{0.0};

   public:
    constexpr Circle() noexcept = default;

    /**
     * @brief C++20 宇宙飞船操作符 <=> 默认合成。
     * 由于包含 Point 与 double 成员，返回类型为 std::partial_ordering。
     * 自动合成 <, <=, >, >=。
     */
    [[nodiscard]] constexpr auto operator<=>(const Circle&) const noexcept = default;

    /**
     * @brief C++20 默认相等比较操作符，自动合成 == 与 !=。
     */
    [[nodiscard]] constexpr bool operator==(const Circle&) const noexcept = default;

    /**
     * @brief 构造一个新的 Circle 对象。
     * n0, n1, n2 是位于圆周上的 3 个节点。
     *
     * @param[in] n0
     * @param[in] n1
     * @param[in] n2
     */
    Circle(const Node& n0, const Node& n1, const Node& n2);

    /**
     * @brief 检查节点 n 是否在圆周内部。
     *
     * @param n
     */
    [[nodiscard]] bool containsNode(const Node& n) const;

    /**
     * @brief 返回局部坐标系下的圆心。
     */
    [[nodiscard]] const Point& center() const noexcept;

    /**
     * @brief 返回全局坐标系下的圆心。
     */
    [[nodiscard]] const Point& centerGlobal() const noexcept;

    /**
     * @brief 返回圆半径的平方。
     */
    [[nodiscard]] constexpr double radSq() const noexcept { return radSq_; }
};
