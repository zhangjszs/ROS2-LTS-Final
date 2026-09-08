/**
 * @file delaunay_triangulator.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 DelaunayTriangulator 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cmath>
#include <iostream>
#include <span>
#include <vector>

#include "structures/Triangle.hpp"
#include "utils/definitions.hpp"

/**
 * @brief 静态类，用于计算给定一组节点（Node，即锥桶）的 Delaunay 三角网集合。
 */
class DelaunayTriangulator {
   private:
    /**
     * @brief 构建并返回一个包含 nodes 中所有节点的三角形。
     *
     * @param[in] nodes C++20 非拥有型节点切片视图
     */
    static Triangle superTriangle(std::span<const Node> nodes);

   public:
    /**
     * @brief 使用 Bowyer-Watson 算法实现计算 Delaunay 三角网集合，
     * 以在给定点集的情况下找到 Delaunay 三角剖分。时间复杂度 O(nlogn)。
     *
     * @param nodes C++20 非拥有型节点切片视图
     */
    static TriangleSet compute(std::span<const Node> nodes);
};
