#pragma once

#include <algorithm>
#include <cmath>
#include <compare>
#include <concepts>
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

/**
 * @brief C++20 非拥有型二维连续矩阵视图（MatrixView）。
 * 对任意底层连续一维缓冲区（如 std::vector<T>, std::array<T>, std::span<T>）提供零拷贝行优先 (Row-Major) 二维索引。
 */
template <typename T>
class MatrixView {
   public:
    constexpr MatrixView() noexcept = default;
    constexpr MatrixView(std::span<T> data, size_t rows, size_t cols) noexcept
        : data_(data), rows_(rows), cols_(cols) {}

    // C++20 泛型转换构造函数：允许 MatrixView<T> 隐式转换为 MatrixView<const T>（匹配 std::span 转换语义）
    template <typename OtherT>
        requires std::convertible_to<OtherT*, T*>
    constexpr MatrixView(const MatrixView<OtherT>& other) noexcept
        : data_(other.data()), rows_(other.rows()), cols_(other.cols()) {}

    [[nodiscard]] constexpr size_t rows() const noexcept { return rows_; }
    [[nodiscard]] constexpr size_t cols() const noexcept { return cols_; }
    [[nodiscard]] constexpr size_t size() const noexcept { return rows_ * cols_; }
    [[nodiscard]] constexpr bool empty() const noexcept { return rows_ == 0 || cols_ == 0; }

    [[nodiscard]] constexpr T& operator()(size_t r, size_t c) noexcept { return data_[r * cols_ + c]; }
    [[nodiscard]] constexpr const T& operator()(size_t r, size_t c) const noexcept { return data_[r * cols_ + c]; }

    [[nodiscard]] constexpr std::span<T> row(size_t r) noexcept { return data_.subspan(r * cols_, cols_); }
    [[nodiscard]] constexpr std::span<const T> row(size_t r) const noexcept { return data_.subspan(r * cols_, cols_); }

    [[nodiscard]] constexpr std::span<T> data() noexcept { return data_; }
    [[nodiscard]] constexpr std::span<const T> data() const noexcept { return data_; }

   private:
    std::span<T> data_{};
    size_t rows_{0};
    size_t cols_{0};
};

/**
 * @brief 拥有型行优先一维扁平连续矩阵（FlatMatrix）。
 * 消除 std::vector<std::vector<T>> 导致的 N+1 次堆内存分配与碎片。
 */
template <typename T>
class FlatMatrix {
   public:
    FlatMatrix() = default;
    FlatMatrix(size_t rows, size_t cols, const T& init_val = T{})
        : rows_(rows), cols_(cols), data_(rows * cols, init_val) {}

    void resize(size_t rows, size_t cols, const T& init_val = T{}) {
        rows_ = rows;
        cols_ = cols;
        data_.assign(rows * cols, init_val);
    }

    void clear() noexcept {
        rows_ = 0;
        cols_ = 0;
        data_.clear();
    }

    [[nodiscard]] size_t rows() const noexcept { return rows_; }
    [[nodiscard]] size_t cols() const noexcept { return cols_; }
    [[nodiscard]] size_t size() const noexcept { return data_.size(); }
    [[nodiscard]] bool empty() const noexcept { return data_.empty(); }

    [[nodiscard]] T& operator()(size_t r, size_t c) noexcept { return data_[r * cols_ + c]; }
    [[nodiscard]] const T& operator()(size_t r, size_t c) const noexcept { return data_[r * cols_ + c]; }

    [[nodiscard]] std::span<T> row(size_t r) noexcept { return std::span<T>(data_).subspan(r * cols_, cols_); }
    [[nodiscard]] std::span<const T> row(size_t r) const noexcept {
        return std::span<const T>(data_).subspan(r * cols_, cols_);
    }

    [[nodiscard]] std::vector<T>& raw_vector() noexcept { return data_; }
    [[nodiscard]] const std::vector<T>& raw_vector() const noexcept { return data_; }

    [[nodiscard]] MatrixView<T> view() noexcept { return MatrixView<T>(data_, rows_, cols_); }
    [[nodiscard]] MatrixView<const T> view() const noexcept { return MatrixView<const T>(data_, rows_, cols_); }
    [[nodiscard]] MatrixView<const T> const_view() const noexcept { return MatrixView<const T>(data_, rows_, cols_); }

    operator MatrixView<const T>() const noexcept { return view(); }
    operator MatrixView<T>() noexcept { return view(); }

   private:
    size_t rows_{0};
    size_t cols_{0};
    std::vector<T> data_;
};

// 匈牙利算法（Kuhn-Munkres O(n³)）
// cost[i][j]: 输入 i 分配给轨迹 j 的代价；>= inf_cost 表示禁止分配
// 返回 assignment[i] = j（轨迹下标），-1 表示未分配

// C++20 MatrixView 矩阵视图重载（零拷贝、最推荐）
std::vector<int> HungarianAssign(MatrixView<const double> cost_view, double inf_cost);

template <typename T>
    requires(!std::same_as<T, const double> && std::convertible_to<T*, const double*>)
inline std::vector<int> HungarianAssign(MatrixView<T> cost_view, double inf_cost) {
    return HungarianAssign(MatrixView<const double>(cost_view), inf_cost);
}

// 高性能展平一维连续内存重载：避免 vector<vector<double>> 的 N+1 次堆内存分配与碎片
// cost_flat 为 rows * cols 的一维连续缓冲区，按行优先 (row-major) 存储
std::vector<int> HungarianAssign(std::span<const double> cost_flat, int rows, int cols, double inf_cost);

// C++20 重构：使用 std::span<const std::vector<double>> 替代 const std::vector<std::vector<double>>&，兼容原有容器
std::vector<int> HungarianAssign(std::span<const std::vector<double>> cost, double inf_cost);

// 自适应 EMA alpha 计算
// speed_ref <= 0 或未启用时返回 fallback_alpha
double ComputeDynamicAlpha(double current_speed, double speed_ref, double alpha_min, double alpha_max,
                           double fallback_alpha);

// 保留距 (origin_x, origin_y) 平方距离 ≤ radius_sq 的下标。xs/ys 长度取较短者。
// C++20 重构：使用 std::span<const double> 统一非拥有式视图，解耦 std::vector，兼容 array/裸指针/切片
std::vector<size_t> FilterIndicesByRadiusSq(std::span<const double> xs, std::span<const double> ys, double origin_x,
                                            double origin_y, double radius_sq);

// Point2D 结构体视图重载：直接处理坐标点连续集合，提升感知接口通用性
std::vector<size_t> FilterIndicesByRadiusSq(std::span<const Point2D> points, double origin_x, double origin_y,
                                            double radius_sq);

}  // namespace cone_dedup_algo
