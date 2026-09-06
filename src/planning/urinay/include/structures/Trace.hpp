/**
 * @file Trace.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Trace 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cassert>
#include <iostream>
#include <memory>

#include "structures/Edge.hpp"

/**
 * @brief 表示一条轨迹，即树搜索中的一条边路径。
 * 它使用 shared_ptr 实现，这样我们可以克隆一个对象并以 O(1) 时间复杂度追加一条边。
 * 表示轨迹的类，即树搜索中的边路径。它使用shared_ptr实现，这样我们可以克隆一个对象并以O(1)的时间复杂度添加一个Edge。
    轨迹"在这里可以理解为在搜索树中进行搜索时所经过的边的路径。"边"是树中两个节点之间的连接。
 */
class Trace {
   private:
    /**
     * @brief Trace 的内部类，包含关于最后一次追加的信息
     * 以及指向前一个连接的指针。
     */
    struct Connection {
        /**
         * @brief 指向它所代表的边的指针（索引）。
         */
        const size_t edgeInd;

        /**
         * @brief 指向此连接之前那个连接的共享指针。
         */
        const std::shared_ptr<Connection> before;

        /**
         * @brief 从此连接开始的 Trace 的大小
         */
        const size_t size;

        /**
         * @brief 此次追加的启发式值。
         */
        const double heur;

        /**
         * @brief 整条 Trace 的平均边长。
         */
        const double avgEdgeLen;

        /**
         * @brief Trace 是否已经闭合了循环
         *（因此它包含（或可能包含）重复的边）。
         */
        const bool loopClosed;

        /**
         * @brief 构造一个新的 Connection 对象。
         *
         * @param[in] edgeInd
         * @param[in] heur
         * @param[in] edgeLen
         * @param[in] loopClosed
         * @param[in] before
         */
        Connection(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed,
                   std::shared_ptr<Connection> before);

        /**
         * @brief 返回连接链中是否存在 _edgeInd。
         *
         * @param[in] _edgeInd
         */
        bool containsEdge(const size_t &_edgeInd) const;

        /**
         * @brief 输出流运算符。
         *
         * @param[in,out] os
         * @param[in] conn
         */
        friend std::ostream &operator<<(std::ostream &os, const Connection &conn) {
            if (conn.before != nullptr) {
                os << *(conn.before);
            }
            return os << conn.edgeInd << " -> ";
        }
    };

    /**
     * @brief 唯一属性，指向一个 Connection 的共享指针。
     */
    std::shared_ptr<Connection> p;

    /**
     * @brief 构造一个新的 Trace 对象，以 p 作为连接链的指针。
     *
     * @param[in] p
     */
    Trace(std::shared_ptr<Connection> p);

   public:
    /**
     * @brief 构造一个新的 Trace 对象。
     */
    Trace();

    /**
     * @brief 构造一个新的 Trace 对象和第一个 Connection 对象。
     *
     * @param[in] edgeInd
     * @param[in] heur
     * @param[in] edgeLen
     * @param[in] loopClosed
     */
    Trace(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed = false);

    /**
     * @brief 将一条边作为 Connection 追加，使用以下数据。
     *
     * @param[in] edgeInd
     * @param[in] heur
     * @param[in] edgeLen
     * @param[in] loopClosed
     */
    void addEdge(const size_t &edgeInd, const double &heur, const double &edgeLen, const bool &loopClosed = false);

    /**
     * @brief 查询 Trace 是否为空。
     */
    bool empty() const;

    /**
     * @brief 返回 Trace 的大小，即连接链的长度。
     */
    size_t size() const;

    /**
     * @brief 返回一个包含前一个连接的连接链的 Trace。
     */
    Trace before() const;

    /**
     * @brief 返回一个包含第一个（初始）连接的 Trace。
     */
    Trace first() const;

    /**
     * @brief 返回最后一条边的索引。
     */
    const size_t &edgeInd() const;

    /**
     * @brief 返回最后一个连接的启发式值。
     */
    const double &heur() const;

    /**
     * @brief 返回最后一个连接的平均边长。
     */
    double avgEdgeLen() const;

    /**
     * @brief 返回从最后一个连接或之前的连接开始循环是否已闭合。
     */
    bool isLoopClosed() const;

    /**
     * @brief 返回连接链的启发式值之和。
     */
    double sumHeur() const;

    /**
     * @brief 检查是否存在任何边索引为 edgeInd 的连接。
     *
     * @param[in] edgeInd
     */
    bool containsEdge(const size_t &edgeInd) const;

    /**
     * @brief 清空连接链。
     */
    void clear();

    /**
     * @brief 输出流运算符。
     *
     * @param[in,out] os
     * @param[in] trace
     */
    friend std::ostream &operator<<(std::ostream &os, const Trace &trace);
};
