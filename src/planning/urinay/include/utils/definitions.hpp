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
#include "structures/Triangle.hpp"

using TriangleSet = std::unordered_set<Triangle>;
using EdgeSet = std::unordered_set<Edge>;

using HeurInd = std::pair<double, size_t>;

using Tracklimits = std::pair<std::vector<Node>, std::vector<Node>>;
