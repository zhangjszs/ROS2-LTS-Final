#pragma once

#include <algorithm>
#include <cmath>
#include <compare>
#include <limits>
#include <span>
#include <vector>

namespace cone_dedup_algo {

// 2D 空间点定义，用于感知与追踪算法轻量级视图传参
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    constexpr auto operator<=>(const Point2D&) const = default;
};

// 匈牙利算法（Kuhn-Munkres O(n³)）
// cost[i][j]: 输入 i 分配给轨迹 j 的代价；>= inf_cost 表示禁止分配
// 返回 assignment[i] = j（轨迹下标），-1 表示未分配
// C++20 重构：使用 std::span<const std::vector<double>> 替代 const std::vector<std::vector<double>>&，解耦外层容器
std::vector<int> HungarianAssign(std::span<const std::vector<double>> cost, double inf_cost);

// 高性能展平一维连续内存重载：避免 vector<vector<double>> 的 N+1 次堆内存分配与碎片
// cost_flat 为 rows * cols 的一维连续缓冲区，按行优先 (row-major) 存储
std::vector<int> HungarianAssign(std::span<const double> cost_flat, int rows, int cols, double inf_cost);

// 自适应 EMA alpha 计算
// speed_ref <= 0 或未启用时返回 fallback_alpha
double ComputeDynamicAlpha(double current_speed, double speed_ref, double alpha_min, double alpha_max,
                           double fallback_alpha);

// 保留距 (origin_x, origin_y) 平方距离 ≤ radius_sq 的下标。xs/ys 长度取较短者。
// C++20 重构：使用 std::span<const double> 统一非拥有式视图，解耦 std::vector，兼容 array/裸指针/切片
std::vector<size_t> FilterIndicesByRadiusSq(std::span<const double> xs, std::span<const double> ys,
                                            double origin_x, double origin_y, double radius_sq);

// Point2D 结构体视图重载：直接处理坐标点连续集合，提升感知接口通用性
std::vector<size_t> FilterIndicesByRadiusSq(std::span<const Point2D> points,
                                            double origin_x, double origin_y, double radius_sq);

}  // namespace cone_dedup_algo
