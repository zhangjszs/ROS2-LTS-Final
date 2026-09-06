#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cone_dedup_algo {

// 匈牙利算法（Kuhn-Munkres O(n³)）
// cost[i][j]: 输入 i 分配给轨迹 j 的代价；>= inf_cost 表示禁止分配
// 返回 assignment[i] = j（轨迹下标），-1 表示未分配
std::vector<int> HungarianAssign(const std::vector<std::vector<double>>& cost, double inf_cost);

// 自适应 EMA alpha 计算
// speed_ref <= 0 或未启用时返回 fallback_alpha
double ComputeDynamicAlpha(double current_speed, double speed_ref, double alpha_min, double alpha_max,
                           double fallback_alpha);

// 保留距 (origin_x, origin_y) 平方距离 ≤ radius_sq 的下标。xs/ys 长度取较短者。
std::vector<size_t> FilterIndicesByRadiusSq(const std::vector<double>& xs, const std::vector<double>& ys,
                                            double origin_x, double origin_y, double radius_sq);

}  // namespace cone_dedup_algo
