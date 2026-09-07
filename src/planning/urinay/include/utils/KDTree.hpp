/*
 * file: KDTree.hpp
 *
 * C++20 Concepts 泛型空间索引 KD-Tree（平坦连续数组结构）。
 * 支持任意符合 Point2DLike 概念的空间点与符合 DistanceMetric 概念的度量算子。
 * 默认别名 KDTree 与旧代码 100% 保持完全兼容。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <initializer_list>
#include <limits>
#include <list>
#include <ranges>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "structures/Point.hpp"

namespace urinay::concepts {

/**
 * @brief 距离度量概念（DistanceMetric）：约束点对间距离度量、单轴距离裁剪及半径转换函数
 */
template <typename Metric, typename PointT>
concept DistanceMetric = requires(const Metric& m, const PointT& a, const PointT& b, size_t dim, double r) {
    { m(a, b) } -> std::convertible_to<double>;
    { m.axis_distance(a, b, dim) } -> std::convertible_to<double>;
    { m.to_metric_radius(r) } -> std::convertible_to<double>;
};

/**
 * @brief 可空间索引的点概念：必须满足二维几何点特征
 */
template <typename T>
concept SpatialIndexablePoint = Point2DLike<T>;

}  // namespace urinay::concepts

/**
 * @brief 平方欧氏距离度量算子（默认欧氏距离度量，避免开方计算）
 */
struct SquaredEuclideanMetric {
    template <urinay::concepts::Point2DLike P1, urinay::concepts::Point2DLike P2>
    constexpr double operator()(const P1& a, const P2& b) const noexcept {
        const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
        const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
        return dx * dx + dy * dy;
    }

    template <urinay::concepts::Point2DLike P1, urinay::concepts::Point2DLike P2>
    constexpr double axis_distance(const P1& a, const P2& b, size_t dim) const noexcept {
        const double diff = (dim == 0)
            ? (static_cast<double>(a.x) - static_cast<double>(b.x))
            : (static_cast<double>(a.y) - static_cast<double>(b.y));
        return diff * diff;
    }

    constexpr double to_metric_radius(double rad) const noexcept {
        return rad * rad;
    }
};

/**
 * @brief 曼哈顿距离度量算子（L1 范数度量）
 */
struct ManhattanMetric {
    template <urinay::concepts::Point2DLike P1, urinay::concepts::Point2DLike P2>
    constexpr double operator()(const P1& a, const P2& b) const noexcept {
        const double dx = std::abs(static_cast<double>(a.x) - static_cast<double>(b.x));
        const double dy = std::abs(static_cast<double>(a.y) - static_cast<double>(b.y));
        return dx + dy;
    }

    template <urinay::concepts::Point2DLike P1, urinay::concepts::Point2DLike P2>
    constexpr double axis_distance(const P1& a, const P2& b, size_t dim) const noexcept {
        return (dim == 0)
            ? std::abs(static_cast<double>(a.x) - static_cast<double>(b.x))
            : std::abs(static_cast<double>(a.y) - static_cast<double>(b.y));
    }

    constexpr double to_metric_radius(double rad) const noexcept {
        return rad;
    }
};

// Optional wrapper（与原版 KDTData 接口完全一致）
template <class T>
class KDTData {
    bool valid_ = false;
    T val_{};

   public:
    KDTData() = default;
    explicit KDTData(const T& v) : valid_(true), val_(v) {}
    KDTData& operator=(const T& v) {
        valid_ = true;
        val_ = v;
        return *this;
    }
    explicit operator bool() const noexcept {
        return valid_;
    }
    T& operator*() noexcept {
        return val_;
    }
    const T& operator*() const noexcept {
        return val_;
    }
    T* operator->() noexcept {
        return &val_;
    }
    const T* operator->() const noexcept {
        return &val_;
    }
};

/**
 * @brief 泛型平坦连续数组 KD-Tree
 * @tparam PointT 存储与查询的点类型，受 Point2DLike 概念约束
 * @tparam MetricT 空间距离度量策略，受 DistanceMetric 概念约束
 */
template <urinay::concepts::Point2DLike PointT = Point,
          urinay::concepts::DistanceMetric<PointT> MetricT = SquaredEuclideanMetric>
class BasicKDTree {
   public:
    using point_type = PointT;
    using metric_type = MetricT;
    using point_index = std::pair<PointT, size_t>;
    using point_index_arr = std::vector<point_index>;

   private:
    struct Node {
        double c[2];    // 坐标
        size_t idx;     // 原始索引
        int left = -1;  // nodes_ 中子节点下标，-1 表示叶节点
        int right = -1;
    };

    std::vector<Node> nodes_;   // 平坦存储，构建后无堆指针寻址，内存局部性极佳
    std::vector<PointT> pts_;   // 原始 PointT 数据
    MetricT metric_{};

    using Entry = std::pair<std::array<double, 2>, size_t>;

    // 递归建树（std::ranges::nth_element 中值快速划分）
    int buildRec(std::vector<Entry>& tmp, int lo, int hi, int lv) {
        if (lo >= hi)
            return -1;

        int mid = lo + (hi - lo) / 2;
        std::nth_element(tmp.begin() + lo, tmp.begin() + mid, tmp.begin() + hi,
                         [lv](const Entry& a, const Entry& b) { return a.first[lv] < b.first[lv]; });

        int ni = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{});
        nodes_.back().c[0] = tmp[mid].first[0];
        nodes_.back().c[1] = tmp[mid].first[1];
        nodes_.back().idx = tmp[mid].second;

        // 注意：push_back 可能使 nodes_ 重分配，必须先记录 ni 索引再进入递归
        int lc = buildRec(tmp, lo, mid, 1 - lv);
        int rc = buildRec(tmp, mid + 1, hi, 1 - lv);
        nodes_[ni].left = lc;
        nodes_[ni].right = rc;
        return ni;
    }

    void searchNN(int ni, const double q[2], int lv, int& best, double& bestDist,
                  const std::set<size_t>& excs) const {
        if (ni < 0)
            return;
        const Node& n = nodes_[ni];

        struct QueryProxy { double x, y; };
        QueryProxy q_pt{q[0], q[1]};
        QueryProxy n_pt{n.c[0], n.c[1]};

        double d = metric_(q_pt, n_pt);

        if (excs.find(n.idx) == excs.end()) {
            if (best < 0 || d < bestDist) {
                bestDist = d;
                best = ni;
            }
        }

        double diff = q[lv] - n.c[lv];
        int near_child = diff <= 0 ? n.left : n.right;
        int far_child = diff <= 0 ? n.right : n.left;

        searchNN(near_child, q, 1 - lv, best, bestDist, excs);

        // 分支界定裁剪：依据度量算子的单轴距离判定是否需要遍历远端分支
        double axis_d = metric_.axis_distance(q_pt, n_pt, static_cast<size_t>(lv));
        if (axis_d < bestDist || best < 0) {
            searchNN(far_child, q, 1 - lv, best, bestDist, excs);
        }
    }

    void searchRadius(int ni, const double q[2], double r_metric, int lv, std::vector<size_t>& out) const {
        if (ni < 0)
            return;
        const Node& n = nodes_[ni];

        struct QueryProxy { double x, y; };
        QueryProxy q_pt{q[0], q[1]};
        QueryProxy n_pt{n.c[0], n.c[1]};

        if (metric_(q_pt, n_pt) <= r_metric) {
            out.push_back(n.idx);
        }

        double diff = q[lv] - n.c[lv];
        int near_child = diff <= 0 ? n.left : n.right;
        int far_child = diff <= 0 ? n.right : n.left;

        searchRadius(near_child, q, r_metric, 1 - lv, out);
        if (metric_.axis_distance(q_pt, n_pt, static_cast<size_t>(lv)) <= r_metric) {
            searchRadius(far_child, q, r_metric, 1 - lv, out);
        }
    }

    void initFromEntries(std::vector<Entry>& tmp) {
        if (tmp.empty())
            return;
        nodes_.reserve(tmp.size());
        buildRec(tmp, 0, static_cast<int>(tmp.size()), 0);
    }

   public:
    BasicKDTree() = default;
    explicit BasicKDTree(MetricT metric) : metric_(metric) {}

    /**
     * @brief 范围概念构造函数：支持从任意值类型满足 Point2DLike 的标准容器或惰性视图建树
     */
    template <std::ranges::input_range R>
        requires urinay::concepts::Point2DLike<std::ranges::range_value_t<R>>
    explicit BasicKDTree(R&& range, MetricT metric = MetricT{}) : metric_(metric) {
        std::vector<Entry> tmp;
        size_t idx = 0;
        for (const auto& p : range) {
            pts_.push_back(static_cast<PointT>(p));
            tmp.push_back({{static_cast<double>(p.x), static_cast<double>(p.y)}, idx++});
        }
        initFromEntries(tmp);
    }

    BasicKDTree(std::initializer_list<PointT> init, MetricT metric = MetricT{})
        : BasicKDTree(std::vector<PointT>(init), metric) {}

    template <urinay::concepts::Point2DLike QueryPointT>
    KDTData<size_t> nearest_index(const QueryPointT& pt, const std::set<size_t>& excs = {}) const {
        if (nodes_.empty())
            return KDTData<size_t>();
        int best = -1;
        double bestDist = std::numeric_limits<double>::max();
        double q[2] = {static_cast<double>(pt.x), static_cast<double>(pt.y)};
        searchNN(0, q, 0, best, bestDist, excs);
        if (best < 0)
            return KDTData<size_t>();
        return KDTData<size_t>(nodes_[best].idx);
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    KDTData<PointT> nearest_point(const QueryPointT& pt, const std::set<size_t>& excs = {}) const {
        auto idx = nearest_index(pt, excs);
        if (!idx)
            return KDTData<PointT>();
        return KDTData<PointT>(pts_[*idx]);
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    KDTData<point_index> nearest_pointIndex(const QueryPointT& pt, const std::set<size_t>& excs = {}) const {
        auto idx = nearest_index(pt, excs);
        if (!idx)
            return KDTData<point_index>();
        return KDTData<point_index>(point_index(pts_[*idx], *idx));
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    std::vector<size_t> neighborhood_indices(const QueryPointT& pt, double rad) const {
        if (nodes_.empty())
            return {};
        std::vector<size_t> idxs;
        double q[2] = {static_cast<double>(pt.x), static_cast<double>(pt.y)};
        double r_metric = metric_.to_metric_radius(rad);
        searchRadius(0, q, r_metric, 0, idxs);
        return idxs;
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    point_index_arr neighborhood(const QueryPointT& pt, double rad) const {
        auto idxs = neighborhood_indices(pt, rad);
        point_index_arr res;
        res.reserve(idxs.size());
        for (size_t i : idxs)
            res.emplace_back(pts_[i], i);
        return res;
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    std::vector<PointT> neighborhood_points(const QueryPointT& pt, double rad) const {
        auto idxs = neighborhood_indices(pt, rad);
        std::vector<PointT> res;
        res.reserve(idxs.size());
        for (size_t i : idxs)
            res.push_back(pts_[i]);
        return res;
    }

    template <urinay::concepts::Point2DLike QueryPointT>
    std::unordered_set<size_t> neighborhood_indices_set(const QueryPointT& pt, double rad) const {
        auto idxs = neighborhood_indices(pt, rad);
        return std::unordered_set<size_t>(idxs.begin(), idxs.end());
    }

    bool empty() const noexcept { return nodes_.empty(); }
    size_t size() const noexcept { return pts_.size(); }
    const std::vector<PointT>& pts() const noexcept { return pts_; }
};

// -------------------------------------------------------------
// 向下兼容别名 (Backward Compatibility Type Aliases)
// -------------------------------------------------------------
using KDTree = BasicKDTree<Point, SquaredEuclideanMetric>;
using indexArr = std::vector<size_t>;
using pointIndex = std::pair<Point, size_t>;
using pointIndexArr = std::vector<pointIndex>;
using pointIndexV = KDTData<pointIndex>;

