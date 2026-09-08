/**
 * @file WayComputer.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 WayComputer 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <common_msgs/msg/huat_carstate.hpp>
#include <common_msgs/msg/huat_path_limits.hpp>
#include <common_msgs/msg/huat_tracklimits.hpp>
#include <fstream>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "modules/urinay_visualizer.hpp"
#include "structures/Trace.hpp"
#include "structures/Vector.hpp"
#include "structures/Way.hpp"
#include "utils/Failsafe.hpp"
#include "utils/KDTree.hpp"
#include "utils/constants.hpp"
#include "utils/definitions.hpp"
#include "utils/urinay_params.hpp"

/**
 * @brief 路径输出模式，用于 getPathLimitsGlobal()。
 * 替代原有的魔法整数，使调用方意图一目了然。
 */
enum class PathMode {
    GlobalPath = 0,         ///< 全部全局坐标系路径
    InterpolateToNext = 1,  ///< 车到下一目标点线性插值
    LocalInterp = 2,        ///< 局部路径插值（全局坐标）
    FullInterp = 3,         ///< 全路径插值（全局坐标）
    LocalInterpLocal = 4,   ///< 局部路径插值（局部坐标）— 默认模式
    FullInterpLocal = 5,    ///< 全路径插值（局部坐标）
    LocalCoordsPath = 6,    ///< 局部坐标系下的路径
};

/**
 * @brief 一个包含所有计算路径（Way）的工具和函数的类。
 * 它接收 Delaunay 三角网集合和车辆位置来进行计算。
 */
class WayComputer {
   private:
    /**
     * @brief 所有与 WayComputer 类相关的参数。
     */
    const UrinayParams::WayComputer params_;

    /**
     * @brief 搜索参数的故障保护（Failsafe）。
     */
    Failsafe<UrinayParams::WayComputer::Search> generalFailsafe_;

    /**
     * @brief 计算结果和上一次迭代的结果。
     * 当前迭代的路径计算结果。它是一个类型为 `Way` 的对象，用于存储路径的信息，包括路径的节点、方向、曲率等
     * 上一次迭代的路径计算结果。它也是一个类型为 `Way` 的对象，用于存储上一次迭代计算得到的路径信息。
     */
    Way way_, lastWay_;

    /**
     * @brief 这个 Way 对象是为了解决非停止循环计算而创建的。
     * 它是每次都会发布的路径（包括完整路径和部分路径）。
     */
    Way wayToPublish_;

    /**
     * @brief 上一次数据的时间戳。
     */
    rclcpp::Time lastStamp_;

    /**
     * @brief way_ 的循环是否已经闭合。
     */
    bool isLoopClosed_ = false;

    /**
     * @brief 全局坐标系与局部坐标系之间的变换。
     */
    Eigen::Affine3d localTf_;

    /**
     * @brief localTf_ 是否有效。
     */
    bool localTfValid_ = false;

    /**
     * @brief 车身位置
     */
    geometry_msgs::msg::Pose pose;

    common_msgs::msg::HuatCarstate CarState;

    mutable std::mutex state_mutex_;

    /** @brief 保护 way_/lastWay_/wayToPublish_/isLoopClosed_/lastStamp_ 的读写互斥锁 */
    mutable std::mutex way_mutex_;

    /**
     * @brief 过滤三角网，移除所有不需要的三角形。
     *
     * @param[in,out] triangulation
     */
    void filterTriangulation(TriangleSet& triangulation) const;

    /**
     * @brief 根据中点过滤边，移除所有不需要的边。
     *
     * @param[in,out] edges
     * @param[in] triangulation
     */
    void filterMidpoints(EdgeSet& edges, const TriangleSet& triangulation) const;

    /**
     * @brief 基于角度和距离计算并返回启发式值。
     * 使用 params 作为其参数。
     *
     * @param[in] actPos
     * @param[in] nextPos
     * @param[in] dir
     * @param[in] params
     */
    double getHeuristic(const Point& actPos, const Point& nextPos, const Vector& dir,
                        const UrinayParams::WayComputer::Search& params) const;

    /**
     * @brief 返回路径（Way）和 Trace *trace（如果有）的平均边长。
     *
     * @param[in] trace
     */
    inline double avgEdgeLen(const Trace* trace) const;

    /**
     * @brief 根据所有指标和阈值找到所有可能的下一条边。
     * 使用 params 作为其参数。
     *
     * @param[out] nextEdges
     * @param[in] actTrace
     * @param[in] midpointsKDT
     * @param[in] edges
     * @param[in] params
     */
    bool shouldExcludeEdge(const Edge& candidate, const Edge* actEdge, const Point& actPos, const Point& lastPos,
                           const Vector& dir, const Trace* actTrace,
                           const UrinayParams::WayComputer::Search& params) const;

    void findNextEdges(std::vector<HeurInd>& nextEdges, const Trace* actTrace, const KDTree& midpointsKDT,
                       const std::vector<Edge>& edges, const UrinayParams::WayComputer::Search& params) const;

    void ResolveSearchContext(const Trace* actTrace, const std::vector<Edge>& edges, const Edge*& actEdge,
                              Point& actPos, Point& lastPos, Vector& dir) const;

    void FillTracklimits(common_msgs::msg::HuatPathLimits& res) const;
    bool ShouldRemoveTriangle(const Triangle& t) const;
    static void AppendPointsToPath(const std::vector<Point>& pts, common_msgs::msg::HuatPathLimits& res);

    /**
     * @brief 从之前最优的 Trace 和候选 Trace 中计算出最优的 Trace。
     *
     * @param best 之前最优的 Trace
     * @param t 最优 Trace 的候选
     * @return Trace 新的最优 Trace
     */
    Trace computeBestTraceWithFinishedT(const Trace& best, const Trace& t) const;

    /**
     * @brief 执行有限高度的启发式加权树搜索，并返回最佳下一条边的索引。
     * 使用 params 作为其参数。
     *
     * @param[in] nextEdges
     * @param[in] midpointsKDT
     * @param[in] edges
     * @param[in] params
     */
    size_t treeSearch(std::vector<HeurInd>& nextEdges, const KDTree& midpointsKDT, const std::vector<Edge>& edges,
                      const UrinayParams::WayComputer::Search& params) const;

    /**
     * @brief 类的主函数，接收所有边并计算最佳的可能中心线（Way）。
     * 使用 params 作为其参数。
     *
     * @param[in] edges
     * @param[in] params
     */
    void computeWay(const std::vector<Edge>& edges, const UrinayParams::WayComputer::Search& params);

   public:
    /**
     * @brief 构造一个新的 WayComputer 对象。
     *
     * @param[in] params
     */
    WayComputer(const UrinayParams::WayComputer& params);

    /**
     * @brief 车辆状态的回调函数。
     *
     * @param[in] data
     */
    void stateCallback(common_msgs::msg::HuatCarstate::ConstSharedPtr data);

    /**
     * @brief 接收 Delaunay 三角网集合并计算路径（Way）。
     *
     * @param[in,out] triangulation
     * @param[in] stamp
     */
    void update(TriangleSet& triangulation, const rclcpp::Time& stamp);

    /**
     * @brief 返回循环是否已经闭合。
     */
    const bool& isLoopClosed() const;

    /**
     * @brief 将路径（Way）写入指定的文件路径。
     *
     * @param[in] file_path
     */
    void writeWayToFile(const std::string& file_path) const;

    /**
     * @brief 返回属性 localTf 是否有效。
     */
    bool isLocalTfValid() const;

    /**
     * @brief 返回从全局坐标系到局部坐标系的变换。
     */
    Eigen::Affine3d getLocalTf() const;

    /**
     * @brief 返回全局坐标系下的中心线向量。
     */
    std::vector<Point> getPath() const;

    /**
     * @brief 返回全局坐标系下的赛道边界。
     */
    Tracklimits getTracklimits() const;

    /**
     * @brief 返回全局坐标系下的中心线和赛道边界，格式为 as_msgs。
     */
    common_msgs::msg::HuatPathLimits getPathLimits() const;

    /**
     * @brief 按指定模式传出路径。
     * @param mode 路径输出模式，参见 PathMode 枚举。
     */
    common_msgs::msg::HuatPathLimits getPathLimitsGlobal(PathMode mode);

    common_msgs::msg::HuatCarstate getCarState();
};
