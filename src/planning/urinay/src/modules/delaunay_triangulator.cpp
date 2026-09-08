/**
 * @file DelaunayTri.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief DelaunayTriangulator 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/delaunay_triangulator.hpp"

#include <algorithm>
#include <cassert>
#include <ranges>
#include <unordered_map>

/* ----------------------------- 私有方法 ---------------------------- */
// 超级三角形是用来包围一组节点的初始三角形，在进行 Delaunay 三角剖分后，属于超级三角形的三角形会被去除。
Triangle DelaunayTriangulator::superTriangle(std::span<const Node> nodes) {
    assert(!nodes.empty());
    // C++20 std::ranges::minmax_element 配合投影函数，一行代码求出 x/y 轴极值范围
    auto [min_x_it, max_x_it] = std::ranges::minmax_element(nodes, {}, [](const Node& n) { return n.x(); });
    auto [min_y_it, max_y_it] = std::ranges::minmax_element(nodes, {}, [](const Node& n) { return n.y(); });

    const double xmin = min_x_it->x();
    const double xmax = max_x_it->x();
    const double ymin = min_y_it->y();
    const double ymax = max_y_it->y();

    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dmax = std::max(dx, dy);
    const double midx = (xmin + xmax) / 2.0;
    const double midy = (ymin + ymax) / 2.0;

    const Node n0 = Node::superTriangleNode(midx - 20 * dmax, midy - dmax);
    const Node n1 = Node::superTriangleNode(midx, midy + 20 * dmax);
    const Node n2 = Node::superTriangleNode(midx + 20 * dmax, midy - dmax);

    return {n0, n1, n2};
}

/* ----------------------------- 公有方法 ----------------------------- */
// 目的为了构建一个超级大三角形，完成包含所有点的工作，做这个工作的原因保证后续三角剖分的正确性与完整性

TriangleSet DelaunayTriangulator::compute(std::span<const Node> nodes) {
    if (nodes.size() < 3)
        return {};
    // 创建一个TriangleSet对象用于存储三角剖分结果
    TriangleSet triangulation;
    // 添加一个足够大以包含所有点的超级三角形到三角剖分中
    // 可以确保算法在进行后续的三角剖分操作时能够准确地考虑到所有的点，并生成一个正确的Delaunay三角剖分结果。
    triangulation.insert(superTriangle(nodes));

    // 将每个节点逐个添加到三角剖分中
    for (const Node& n : nodes) {
        TriangleSet badTriangles;

        // 首先找到由于插入节点而不再有效的所有三角形
        for (const Triangle& t : triangulation) {
            if (t.circleContainsNode(n)) {
                badTriangles.insert(t);
            }
        }

        // 使用边计数法查找边界边（O(b) 替代 O(b²)）
        // 边界边恰好只出现在一个坏三角形中
        std::unordered_map<Edge, int> edgeCount;
        for (const Triangle& t : badTriangles) {
            for (const Edge& e : t.edges) {
                edgeCount[e]++;
            }
        }

        EdgeSet polygon;
        for (const auto& entry : edgeCount) {
            if (entry.second == 1) {
                polygon.insert(entry.first);
            }
        }

        // 从三角剖分数据结构中移除所有坏三角形
        for (const Triangle& t : badTriangles) {
            triangulation.erase(t);
        }

        // 对多边形空洞重新进行三角剖分
        for (const Edge& e : polygon) {
            triangulation.emplace(e, n);
        }
    }

    // C++20 统一容器擦除：移除所有属于超级三角形的三角形
    std::erase_if(triangulation, [](const Triangle& t) { return t.anyNodeInSuperTriangle(); });

    return triangulation;
}