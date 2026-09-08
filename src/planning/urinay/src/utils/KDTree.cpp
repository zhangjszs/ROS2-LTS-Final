/*
 * file: KDTree.cpp
 *
 * C++20 Concepts 泛型空间索引 KD-Tree 显式实例化文件。
 */

#include "utils/KDTree.hpp"

// 显式实例化标准配置，导出至 liburinay
template class BasicKDTree<Point, SquaredEuclideanMetric>;
template class BasicKDTree<Point, ManhattanMetric>;
