#include "cone_dedup_algo.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace cone_dedup_algo {

namespace {

// 内部通用的 Kuhn-Munkres 匈牙利算法实现，通过 Lambda 访问代价矩阵，消除多重重载的代码重复
template <typename CostAccessor>
std::vector<int> HungarianAssignInternal(int rows, int cols, double inf_cost, CostAccessor&& get_cost) {
    if (rows == 0)
        return {};
    if (cols == 0)
        return std::vector<int>(rows, -1);
    int sz = std::max(rows, cols);

    // 扩展为方阵，无效格填 inf_cost
    std::vector<std::vector<double>> c(sz, std::vector<double>(sz, inf_cost));
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            c[i][j] = get_cost(i, j);

    std::vector<double> u(sz + 1, 0.0), v(sz + 1, 0.0);
    std::vector<int> p(sz + 1, 0), way(sz + 1, 0);

    for (int i = 1; i <= sz; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(sz + 1, std::numeric_limits<double>::max());
        std::vector<bool> used(sz + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = -1;
            double delta = std::numeric_limits<double>::max();
            for (int j = 1; j <= sz; ++j) {
                if (!used[j]) {
                    double val = c[i0 - 1][j - 1] - u[i0] - v[j];
                    if (val < minv[j]) {
                        minv[j] = val;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= sz; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // p[j] = 行（1-indexed）被分配到列 j
    std::vector<int> result(rows, -1);
    for (int j = 1; j <= cols; ++j) {
        int row = p[j] - 1;
        if (row >= 0 && row < rows && get_cost(row, j - 1) < inf_cost)
            result[row] = j - 1;
    }
    return result;
}

}  // namespace

std::vector<int> HungarianAssign(std::span<const std::vector<double>> cost, double inf_cost) {
    int rows = static_cast<int>(cost.size());
    if (rows == 0)
        return {};
    int cols = static_cast<int>(cost[0].size());
    return HungarianAssignInternal(rows, cols, inf_cost, [&](int r, int c) { return cost[r][c]; });
}

std::vector<int> HungarianAssign(std::span<const double> cost_flat, int rows, int cols, double inf_cost) {
    if (rows < 0 || cols < 0 || cost_flat.size() < static_cast<size_t>(rows * cols))
        return {};
    return HungarianAssignInternal(rows, cols, inf_cost, [&](int r, int c) { return cost_flat[r * cols + c]; });
}

double ComputeDynamicAlpha(double current_speed, double speed_ref, double alpha_min, double alpha_max,
                           double fallback_alpha) {
    if (speed_ref <= 0.0) {
        return fallback_alpha;
    }
    double ratio = current_speed / speed_ref;
    if (ratio > 1.0)
        ratio = 1.0;
    if (ratio < 0.0)
        ratio = 0.0;
    double alpha = alpha_min + (alpha_max - alpha_min) * ratio;
    return alpha;
}

std::vector<size_t> FilterIndicesByRadiusSq(std::span<const double> xs, std::span<const double> ys,
                                            double origin_x, double origin_y, double radius_sq) {
    std::vector<size_t> valid;
    const size_t n = std::min(xs.size(), ys.size());
    valid.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const double dx = xs[i] - origin_x;
        const double dy = ys[i] - origin_y;
        if (dx * dx + dy * dy <= radius_sq)
            valid.push_back(i);
    }
    return valid;
}

std::vector<size_t> FilterIndicesByRadiusSq(std::span<const Point2D> points,
                                            double origin_x, double origin_y, double radius_sq) {
    std::vector<size_t> valid;
    valid.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const double dx = points[i].x - origin_x;
        const double dy = points[i].y - origin_y;
        if (dx * dx + dy * dy <= radius_sq)
            valid.push_back(i);
    }
    return valid;
}

}  // namespace cone_dedup_algo
