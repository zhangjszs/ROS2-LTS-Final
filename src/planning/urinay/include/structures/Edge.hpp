/**
 * @file Edge.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Edge 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <compare>
#include "structures/Node.hpp"
#include "utils/constants.hpp"

/**
 * @brief 表示三角形的一条边，并包含所有相关信息。
 */
class Edge {
   public:
    /**
     * @brief 一条边由两个节点定义（其点构成一条边）。
     */
    const Node n0, n1;

    /**
     * @brief 边的长度。
     */
    const double len;

   private:
    /**
     * @brief 边的唯一哈希值。哈希值使用节点的 ID 计算得出。
     */
    const uint64_t hash_;

    /**
     * @brief 返回一条以 n0 和 n1 为节点的边所具有的哈希值。
     *
     * @param[in] n0
     * @param[in] n1
     */
    static uint64_t computeHash(const Node &n0, const Node &n1);

    /**
     * @brief 返回由 n0 和 n1 定义的边的长度。
     *
     * @param[in] n0
     * @param[in] n1
     */
    static double computeLen(const Node &n0, const Node &n1);
    friend class std::hash<Edge>;

   public:

    /**
     * @brief 构造一个新的 Edge 对象。
     *
     * @param[in] n0
     * @param[in] n1
     */
    Edge(const Node &n0, const Node &n1);

    /**
     * @brief C++20 宇宙飞船操作符 <=>。
     * 根据边的唯一哈希值 hash_ 进行全序三路比较，返回 std::strong_ordering。
     * 自动合成 <, <=, >, >=，使 Edge 可直接用于 std::ranges::sort、std::set 等有序容器。
     */
    [[nodiscard]] constexpr std::strong_ordering operator<=>(const Edge &e) const noexcept {
        return this->hash_ <=> e.hash_;
    }

    /**
     * @brief 比较运算符。如果两条边的哈希值相同，则它们相等。
     * C++20 自动合成 != 运算符。
     */
    [[nodiscard]] constexpr bool operator==(const Edge &e) const noexcept {
        return this->hash_ == e.hash_;
    }

    /**
     * @brief 更新两个节点的局部坐标。
     *
     * @param[in] tf
     */
    void updateLocal(const Eigen::Affine3d &tf) const;

    /**
     * @brief 返回局部坐标系下该边的中点。
     */
    Point midPoint() const;

    /**
     * @brief 返回全局坐标系下该边的中点。
     */
    Point midPointGlobal() const;

    /**
     * @brief 返回该边的一个法向量。
     */
    Vector normal() const;

    /**
     * @brief 输出流运算符。
     *
     * @param[in,out] os
     * @param[int] e
     */
    friend std::ostream &operator<<(std::ostream &os, const Edge &e);
};

template <>
struct std::hash<Edge> {
    uint64_t operator()(const Edge &e) const {
        return e.hash_;
    }
};
