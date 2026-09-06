/**
 * @file Way.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Way 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <common_msgs/msg/huat_path_limits.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <list>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/constants.hpp"
#include "utils/definitions.hpp"
#include "utils/urinay_params.hpp"

/**
 * @brief 表示一条路径，即中心线和赛道边界。
 * 它是 WayComputer 模块的计算结果。
 */
class Way {
   private:
    /**
     * @brief 所有与 Way 类相关的参数。
     */
    static UrinayParams::WayComputer::Way params_;

    /**
     * @brief 路径（Way）用边的列表表示，这些边直接
     * 从 Delaunay 三角剖分中获得。在列表中，边按照
     * 它们在中心线上的位置排序。赛道边界也隐含地
     * 在这里。
     * 是一个存储边（Edge）的列表，表示路径（Way）。
     * `path_` 是一个 `std::list<Edge>` 类型的变量，用于存储从Delaunay三角剖分中直接获取的边。
     * 在列表中，边按照它们在中心线上的位置排序。
     * 同时，路径限制（track limits）也内在地与其关联。
     * 通过这个列表，我们可以按顺序访问路径中的各个边。每个边对象都包含有关边的信息，例如起点、终点、长度等。
     * 路径限制也可以与每个边对象关联起来，以便在路径规划和执行过程中考虑到限制条件。
     * 通过访问 `path_` 列表中的边，我们可以遍历路径的每个点，并进行路径跟踪、路径规划和其他相关操作。
     * 。边与中心线起点的距离越短，它在列表中的位置就越靠前。这样可以确保列表中的边按照它们在中心线中的顺序排列。
     */
    std::list<Edge> path_;

    /**
     * @brief 存储在 path_ 中的所有边的平均长度。
     * 这个属性将被 WayComputer 用来查找下一条边。
     * `avgEdgeLen_` 表示存储在 `path_` 中的所有边的平均长度。它将被 `WayComputer` 类使用，
     * 用于查找下一个边。通过计算边的平均长度，`avgEdgeLen_` 属性提供了重要的信息，
     * 可以帮助 `WayComputer` 选择合适的下一个边以继续路径的导航或控制。
     */
    double avgEdgeLen_;

    /**
     * @brief 指向中点距离车辆位置最近的边。
     * 如果为空，则指向 path_.cend()。
     * `closestToCarElem_` 是一个指向 `std::list<Edge>` 类型中元素的常量迭代器。它被用于追踪距离汽车位置最近的边缘的位置
     * 如果 `closestToCarElem_` 为空，即没有找到最近的边缘，则它指向 `path_.cend()`，即指向路径列表 `path_` 的结束迭代器
     * 通过使用这个迭代器，我们可以确定距离汽车位置最近的边缘，
      std::list<Edge>` 是一个链表容器，它存储了 `Edge` 对象。
      const_iterator` 是 `std::list<Edge>` 类型定义的常量迭代器，
      用于遍历链表容器中的元素，并保证遍历过程中不会修改容器中的元素。
     */
    std::list<Edge>::const_iterator closestToCarElem_;

    /**
     * @brief 到达车辆之前的中点数量。在修剪后设置，
     * 对颜色 minimum_midpoints 很有用。
     * 例如，车辆可能已经行驶了一段距离，
     * 然后需要估计从路径起点到达当前车辆位置之前经过的路径部分的长度（用中点数量来衡量）
     */
    uint32_t sizeToCar_;

    /**
     * @brief 相应地更新 closestToCarElem_ 属性。
     */
    void updateClosestToCarElem();

    /**
     * @brief 检查由 AB 和 CD 定义的线段是否相交。
     *
     * @param[in] A
     * @param[in] B
     * @param[in] C
     * @param[in] D
     */
    static bool segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D);

   public:
    /**
     * @brief 初始化单例的方法。
     *
     * @param[in] params
     */
    static void init(const UrinayParams::WayComputer::Way &params);

    /**
     * @brief 构造一个新的 Way 对象。
     */
    Way();

    /**
     * @brief 查询 Way 是否为空。
     */
    bool empty() const;

    /**
     * @brief 查询 Way 的中点数量。
     */
    size_t size() const;

    /**
     * @brief 返回最后一条边。
     */
    const Edge &back() const;

    /**
     * @brief 返回倒数第二条边。
     */
    const Edge &beforeBack() const;

    /**
     * @brief 返回第一条边。
     */
    const Edge &front() const;

    /**
     * @brief 更新 Way 中所有边的局部位置。
     *
     * @param[in] tf
     */
    void updateLocal(const Eigen::Affine3d &tf);

    /**
     * @brief 在 Way 的末尾追加一条边。
     *
     * @param[in] edge
     */
    void addEdge(const Edge &edge);

    /**
     * @brief 修剪 Way，移除距离车辆最近的边之后的所有边。
     * 假设有一个Way对象，表示一条道路，由以下边构成：A -> B -> C -> D -> E -> F。车辆当前位置在边C上。
     * 调用`trimByLocal()`函数后，将会删除车边C之后的所有边。因此，最终的结果是保留了车辆当前位置及其之前的边，即A -> B
     * -> C
     */
    void trimByLocal();

    /**
     * @brief 检查当 e 被追加到 Way 时，Way 是否闭合循环。
     * 如果 lastPosInTrace 不为 NULL，则将其视为最后一个 Way 中点。
     *
     * @param[in] e
     * @param[in] lastPosInTrace
     */
    bool closesLoopWith(const Edge &e, const Point *lastPosInTrace = nullptr) const;

    /**
     * @brief 制作一个副本，确保：
     * - 移除可能的冗余点。（例如，当循环在第二个点处闭合时）。
     * - 第一条和最后一条边重合（完全相等）。
     */
    Way restructureClosure() const;

    /**
     * @brief 检查循环是否已闭合。这意味着：
     * - 长度超过一个阈值
     * - 第一个和最后一个中点之间的距离超过一个阈值
     */
    bool closesLoop() const;

    /**
     * @brief 检查边 e 是否在路径上创建交叉（一个循环）。
     * 时间复杂度 O(n)，n=this->size()。
     *
     * @param[in] e
     */
    bool intersectsWith(const Edge &e) const;

    /**
     * @brief 检查 Way 是否包含特定的边 e。
     *
     * @param[in] e
     */
    bool containsEdge(const Edge &e) const;

    /**
     * @brief 赋值运算符。
     *
     * @param[in] way
     */
    Way &operator=(const Way &way);

    /**
     * @brief 比较运算符。如果两个 Way 包含相同的边，则它们相等。
     * **注意** 中点（和赛道边界）的位置可能不相等。
     *
     * @param[in] way
     */
    bool operator==(const Way &way) const;

    /**
     * @brief 比较运算符的否定。
     *
     * @param[in] way
     */
    bool operator!=(const Way &way) const;

    /**
     * @brief 检查 vital_num_midpoints（车辆位置后的 n 个中点）
     * 在 *this 和 way 中是否相等。
     *
     * @param[in] way
     */
    bool quinEhLobjetiuDeLaSevaDiresio(const Way &way) const;

    /**
     * @brief 返回平均边长。
     */
    const double &getAvgEdgeLen() const;

    /**
     * @brief 返回路径最后两条边构成的前进方向向量。
     * 路径为空或仅有一条边时返回 (1,0)。
     */
    Vector trackDirection() const;

    /**
     * @brief 返回车辆前方到 Way 末端的中点数量。
     * 不考虑循环闭合。
     */
    uint32_t sizeAheadOfCar() const;

    /**
     * @brief 返回全局坐标系下所有中点的向量。
     */
    std::vector<Point> getPath() const;

    /**
     * @brief 返回赛道边界。第一个元素是左赛道边界，
     * 第二个是右赛道边界。
     */
    Tracklimits getTracklimits() const;

    /**
     * @brief 用于返回全局坐标下的路径
     */
    std::vector<Point> getPathLocal() const;

    void deleteWayPassed();
    Point getNextPathPoint();

    /**
     * @brief 使用去除走过的点进行插值（线性）
     * @param xy 车当前坐标
     */
    std::vector<geometry_msgs::msg::Point> getPathInterpolation(double x, double y);

    /**
     * @brief 使用去除走过的点进行插值（线性，局部坐标系）
     * @param xy 车当前坐标
     */
    std::vector<geometry_msgs::msg::Point> getPathInterpolationLocal(double x, double y);

    /**
     * @brief 使用所有路径点插值（线性）
     */
    std::vector<geometry_msgs::msg::Point> getPathFullInterpolation();

    /**
     * @brief 使用所有路径点插值（线性，局部坐标系）
     */
    std::vector<geometry_msgs::msg::Point> getPathFullInterpolationLocal();

    /**
     * @brief 输出流运算符。
     *
     * @param[in,out] os
     * @param[in] way
     */
    friend std::ostream &operator<<(std::ostream &os, const Way &way);
};
