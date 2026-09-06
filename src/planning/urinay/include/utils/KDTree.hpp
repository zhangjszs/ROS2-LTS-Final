/*
 * file: KDTree.hpp
 *
 * 平坦数组 KD-tree（替代 shared_ptr 版本）。
 * 内部使用连续 std::vector<Node> 存储，消除每帧堆分配和 cache miss。
 * 公开 API 与原版完全兼容。
 */

#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <list>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "structures/Point.hpp"

using indexArr = std::vector<size_t>;
using pointIndex = std::pair<Point, size_t>;
using pointIndexArr = std::vector<pointIndex>;

// Optional wrapper（与原版 KDTData 接口一致）
template <class T>
class KDTData {
    bool valid_ = false;
    T val_;

   public:
    KDTData() = default;
    explicit KDTData(const T& v) : valid_(true), val_(v) {}
    KDTData& operator=(const T& v) {
        valid_ = true;
        val_ = v;
        return *this;
    }
    explicit operator bool() const {
        return valid_;
    }
    T& operator*() {
        return val_;
    }
    T* operator->() {
        return &val_;
    }
    const T& operator*() const {
        return val_;
    }
    const T* operator->() const {
        return &val_;
    }
};
using pointIndexV = KDTData<pointIndex>;

class KDTree {
    struct Node {
        double c[2];    // 坐标
        size_t idx;     // 原始索引
        int left = -1;  // nodes_ 中子节点下标，-1 表示叶
        int right = -1;
    };

    std::vector<Node> nodes_;  // 平坦存储，构建后不再增删
    std::vector<Point> pts_;   // 保留原始 Point（用于返回 Point 对象）

    using Entry = std::pair<std::array<double, 2>, size_t>;

    // 递归建树（nth_element 中值分裂，O(n log n)）
    int buildRec(std::vector<Entry>& tmp, int lo, int hi, int lv);

    // 最近邻搜索（分支界定）
    void searchNN(int ni, const double q[2], int lv, int& best, double& bestD2, const std::set<size_t>& excs) const;

    // 半径查询
    void searchRadius(int ni, const double q[2], double r2, int lv, std::vector<size_t>& out) const;

   public:
    KDTree() = default;
    explicit KDTree(const std::vector<Point>& pts);
    explicit KDTree(const std::list<Point>& pts);

    KDTData<size_t> nearest_index(const Point& pt, const std::set<size_t>& excs = std::set<size_t>()) const;

    KDTData<Point> nearest_point(const Point& pt, const std::set<size_t>& excs = std::set<size_t>()) const;

    pointIndexV nearest_pointIndex(const Point& pt, const std::set<size_t>& excs = std::set<size_t>()) const;

    pointIndexArr neighborhood(const Point& pt, const double& rad) const;
    std::vector<Point> neighborhood_points(const Point& pt, const double& rad) const;
    indexArr neighborhood_indices(const Point& pt, const double& rad) const;
    std::unordered_set<size_t> neighborhood_indices_set(const Point& pt, const double& rad) const;
};
