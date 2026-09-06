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
    double radSq_;

   public:
    /**
     * @brief 构造一个新的 Circle 对象。
     * n0, n1, n2 是位于圆周上的 3 个节点。
     *
     * @param[in] n0
     * @param[in] n1
     * @param[in] n2
     */
    Circle(const Node &n0, const Node &n1, const Node &n2);

    /**
     * @brief 检查节点 n 是否在圆周内部。
     *
     * @param n
     */
    bool containsNode(const Node &n) const;

    /**
     * @brief 返回局部坐标系下的圆心。
     */
    const Point &center() const;

    /**
     * @brief 返回全局坐标系下的圆心。
     */
    const Point &centerGlobal() const;
};
