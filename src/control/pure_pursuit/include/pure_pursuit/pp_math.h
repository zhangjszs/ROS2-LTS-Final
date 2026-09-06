#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace pp_math {

// 将 idx 钳位到三点法有效范围 [1, n-2]
inline int clampKappaIdx(int idx, int n) {
    return std::max(1, std::min(idx, n - 2));
}

// 三点法曲率估算：κ = 2*area / (|ab|*|bc|*|ca|)
inline double estimateCurvature(const std::vector<double>& refx, const std::vector<double>& refy, int idx) {
    const int n = static_cast<int>(refx.size());
    if (n < 3 || idx < 1 || idx >= n - 1)
        return 0.0;

    double ax = refx[idx - 1], ay = refy[idx - 1];
    double bx = refx[idx], by = refy[idx];
    double cx = refx[idx + 1], cy = refy[idx + 1];

    double ab = std::hypot(bx - ax, by - ay);
    double bc = std::hypot(cx - bx, cy - by);
    double ca = std::hypot(ax - cx, ay - cy);

    // area2 = 叉积模 = 2×三角形面积；κ = 4×area / (|ab|×|bc|×|ca|) = 2×area2 / denom
    double area2 = std::abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay));
    if (area2 < 1e-9)
        return 0.0;

    return 2.0 * area2 / (ab * bc * ca);
}

struct NearestIndexResult {
    int idx = -1;
    double dist_sq = 0.0;
};

// 在 [search_start, search_end) 内找距 (cx, cy) 最近的路径点。
inline NearestIndexResult findNearestIndex(const std::vector<double>& refx, const std::vector<double>& refy, double cx,
                                           double cy, int search_start, int search_end) {
    NearestIndexResult result;
    const int n = static_cast<int>(refx.size());
    if (n == 0 || static_cast<int>(refy.size()) != n || search_start >= search_end || search_start < 0)
        return result;
    search_end = std::min(search_end, n);
    result.dist_sq = std::numeric_limits<double>::max();
    result.idx = search_start;
    for (int i = search_start; i < search_end; i++) {
        double dx = cx - refx[i];
        double dy = cy - refy[i];
        double d2 = dx * dx + dy * dy;
        if (d2 < result.dist_sq) {
            result.dist_sq = d2;
            result.idx = i;
        }
    }
    return result;
}

inline bool isBaseLinkFrame(const std::string& frame_id) {
    return frame_id == "base_link";
}

}  // namespace pp_math
