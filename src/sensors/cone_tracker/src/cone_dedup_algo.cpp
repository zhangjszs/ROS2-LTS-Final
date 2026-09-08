#include "cone_dedup_algo.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace cone_dedup_algo {

namespace {

// 内部通用的 Kuhn-Munkres 匈牙利算法实现，通过 Lambda 访问代价矩阵，消除多重重载的代码重复
template <typename CostAccessor>
std::vector<int> HungarianAssignInternal(int rows, int cols, double inf_cost, CostAccessor&& get_cost) {
    if (rows <= 0)
        return {};
    if (cols <= 0)
        return std::vector<int>(rows, -1);
    const int sz = std::max(rows, cols);
    const size_t sz_st = static_cast<size_t>(sz);

    // 扩展为方阵，采用单块连续一维内存存储（Row-Major），彻底消除 vector<vector> 的 N+1 次堆分配
    std::vector<double> c(sz_st * sz_st, inf_cost);
    for (int i = 0; i < rows; ++i) {
        const size_t row_offset = static_cast<size_t>(i) * sz_st;
        for (int j = 0; j < cols; ++j) {
            c[row_offset + static_cast<size_t>(j)] = get_cost(i, j);
        }
    }

    std::vector<double> u(sz_st + 1, 0.0), v(sz_st + 1, 0.0);
    std::vector<int> p(sz_st + 1, 0), way(sz_st + 1, 0);

    // 将循环内部的临时变量提至外层，避免每轮迭代重复进行动态堆分配与释放
    std::vector<double> minv(sz_st + 1);
    std::vector<uint8_t> used(sz_st + 1);

    for (int i = 1; i <= sz; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), std::numeric_limits<double>::max());
        std::fill(used.begin(), used.end(), static_cast<uint8_t>(0));

        do {
            used[static_cast<size_t>(j0)] = 1;
            int i0 = p[static_cast<size_t>(j0)];
            int j1 = -1;
            double delta = std::numeric_limits<double>::max();
            const size_t c_row_offset = static_cast<size_t>(i0 - 1) * sz_st;
            for (int j = 1; j <= sz; ++j) {
                const size_t j_st = static_cast<size_t>(j);
                if (!used[j_st]) {
                    double val = c[c_row_offset + (j_st - 1)] - u[static_cast<size_t>(i0)] - v[j_st];
                    if (val < minv[j_st]) {
                        minv[j_st] = val;
                        way[j_st] = j0;
                    }
                    if (minv[j_st] < delta) {
                        delta = minv[j_st];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= sz; ++j) {
                const size_t j_st = static_cast<size_t>(j);
                if (used[j_st]) {
                    u[static_cast<size_t>(p[j_st])] += delta;
                    v[j_st] -= delta;
                } else {
                    minv[j_st] -= delta;
                }
            }
            j0 = j1;
        } while (p[static_cast<size_t>(j0)] != 0);
        do {
            int j1 = way[static_cast<size_t>(j0)];
            p[static_cast<size_t>(j0)] = p[static_cast<size_t>(j1)];
            j0 = j1;
        } while (j0);
    }

    // p[j] = 行（1-indexed）被分配到列 j
    std::vector<int> result(static_cast<size_t>(rows), -1);
    for (int j = 1; j <= cols; ++j) {
        int row = p[static_cast<size_t>(j)] - 1;
        if (row >= 0 && row < rows && get_cost(row, j - 1) < inf_cost)
            result[static_cast<size_t>(row)] = j - 1;
    }
    return result;
}

}  // namespace

std::vector<int> HungarianAssign(MatrixView<const double> cost_view, double inf_cost) {
    return HungarianAssignInternal(static_cast<int>(cost_view.rows()),
                                   static_cast<int>(cost_view.cols()),
                                   inf_cost,
                                   [&](int r, int c) { return cost_view(r, c); });
}

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
