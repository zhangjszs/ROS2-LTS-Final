/**
 * @file definitions.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含整个程序中需要的定义（别名）。
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <unordered_set>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Triangle.hpp"

using TriangleSet = std::unordered_set<Triangle>;
using EdgeSet = std::unordered_set<Edge>;

using HeurInd = std::pair<double, size_t>;

/**
 * @brief 赛道左右边界聚合结构体 (C++20 Aggregate)
 * 替代原有的 std::pair<std::vector<Node>, std::vector<Node>>，消除 .first / .second 的语义模糊。
 * 原生支持结构化绑定：auto [left_cones, right_cones] = tracklimits;
 */
struct Tracklimits {
    std::vector<Node> left{};
    std::vector<Node> right{};

    [[nodiscard]] constexpr bool empty() const noexcept { return left.empty() && right.empty(); }

    void clear() noexcept {
        left.clear();
        right.clear();
    }
};
