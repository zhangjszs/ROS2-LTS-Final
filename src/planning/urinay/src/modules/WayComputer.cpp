/**
 * @file WayComputer.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief WayComputer 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/WayComputer.hpp"

#include <numbers>

/* ----------------------------- 私有方法 ---------------------------- */

bool WayComputer::ShouldRemoveTriangle(const Triangle &t) const {
    for (const Edge &e : t.edges)
        if (e.len > this->params_.max_triangle_edge_len)
            return true;
    for (const double &angle : t.angles())
        if (angle < this->params_.min_triangle_angle)
            return true;
    return false;
}

void WayComputer::filterTriangulation(TriangleSet &triangulation) const {
    std::erase_if(triangulation, [this](const Triangle &t) {
        return ShouldRemoveTriangle(t);
    });
}

void WayComputer::filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const {
    std::vector<Point> circums;
    circums.reserve(triangulation.size());
    for (const Triangle &t : triangulation) {
        circums.push_back(t.circumCenter());
    }
    KDTree circumKDTree(circums);

    const double max_dist_sq = this->params_.max_dist_circum_midPoint * this->params_.max_dist_circum_midPoint;
    std::erase_if(edges, [&](const Edge &e) {
        Point midPoint = e.midPoint();
        std::optional<size_t> nearestCC = circumKDTree.nearest_index(midPoint);
        return nearestCC.has_value() && Point::distSq(circums[*nearestCC], midPoint) > max_dist_sq;
    });
}

double WayComputer::getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir,
                                 const UrinayParams::WayComputer::Search &params) const {
    double distHeur = Point::dist(actPos, nextPos);  //距离

    double angle = Vector(actPos, nextPos).angleWith(dir);
    constexpr double kHalfPi = std::numbers::pi_v<double> / 2.0;
    double angleHeur = -std::log(
        std::max(1e-9, ((kHalfPi - std::abs(angle)) / kHalfPi) - 0.2));  //这个计算过程的目的是将夹角越接近0，启发式函数值越大

    return params.heur_dist_ponderation * distHeur + (1 - params.heur_dist_ponderation) * angleHeur;
}

inline double WayComputer::avgEdgeLen(const Trace *trace) const {
    if (not trace or trace->empty())
        return this->way_.getAvgEdgeLen();
    else if (this->way_.empty())
        return trace->avgEdgeLen();
    else
        return ((trace->avgEdgeLen() * trace->size()) / (trace->size() + this->way_.size())) +
               ((this->way_.getAvgEdgeLen() * this->way_.size()) / (trace->size() + this->way_.size()));
}

bool WayComputer::shouldExcludeEdge(const Edge &candidate, const Edge *actEdge, const Point &actPos,
                                    const Point &lastPos, const Vector &dir, const Trace *actTrace,
                                    const UrinayParams::WayComputer::Search &params) const {
    if (not actEdge)
        return false;
    // 1. 移除自身
    if (candidate == *actEdge)
        return true;
    // 2. 夹角过大
    if (std::abs(dir.angleWith(Vector(actPos, candidate.midPoint()))) > params.max_angle_diff)
        return true;
    // 3. 路径中已存在但不是闭环边
    if (not this->way_.closesLoopWith(candidate) and (not actTrace or not actTrace->isLoopClosed()) and
        this->way_.containsEdge(candidate))
        return true;
    // 4. 中点与 lastPos 位于 actEdge 同侧（避免反弹）
    if ((this->way_.size() >= 2 or actTrace) and
        Vector::pointBehind(actEdge->midPoint(), lastPos, actEdge->normal()) ==
            Vector::pointBehind(actEdge->midPoint(), candidate.midPoint(), actEdge->normal()))
        return true;
    // 5. 边长偏差过大
    double avg = this->avgEdgeLen(actTrace);
    if (candidate.len < (1 - params.edge_len_diff_factor) * avg or
        candidate.len > (1 + params.edge_len_diff_factor) * avg)
        return true;
    // 6. 添加后产生相交
    if (not params.allow_intersection and (not actTrace or not actTrace->isLoopClosed()) and
        not this->way_.closesLoopWith(candidate) and this->way_.intersectsWith(candidate))
        return true;
    return false;
}

void WayComputer::ResolveSearchContext(const Trace *actTrace, const std::vector<Edge> &edges, const Edge *&actEdge,
                                       Point &actPos, Point &lastPos, Vector &dir) const {
    actEdge = nullptr;
    actPos = Point(0, 0);
    lastPos = Point(0, 0);

    if (actTrace and not actTrace->empty()) {
        actEdge = &edges[actTrace->edgeInd()];
        actPos = edges[actTrace->edgeInd()].midPoint();
        if (actTrace->size() >= 2)
            lastPos = edges[actTrace->before().edgeInd()].midPoint();
    }
    if (not this->way_.empty()) {
        if (not actEdge) {
            actEdge = &this->way_.back();
            actPos = this->way_.back().midPoint();
            if (this->way_.size() >= 2)
                lastPos = this->way_.beforeBack().midPoint();
        } else if (actTrace->size() < 2) {
            lastPos = this->way_.back().midPoint();
        }
    }

    if (actEdge and (actTrace ? actTrace->size() >= 2 : this->way_.size() >= 2))
        dir = Vector(lastPos, actPos);
    else if (actEdge)
        dir = Vector(Point(0, 0), actPos);
    else if (not this->lastWay_.empty())
        dir = this->lastWay_.trackDirection();
    else
        dir = Vector(1, 0);
}

void WayComputer::AppendPointsToPath(const std::vector<Point> &pts, common_msgs::msg::HuatPathLimits &res) {
    res.path.reserve(res.path.size() + pts.size());
    std::ranges::transform(pts, std::back_inserter(res.path), [](const Point &p) {
        return p.gmPoint();
    });
}

void WayComputer::FillTracklimits(common_msgs::msg::HuatPathLimits &res) const {
    const Tracklimits tracklimits = this->wayToPublish_.getTracklimits();
    res.tracklimits.header.stamp = this->lastStamp_;
    res.tracklimits.left.reserve(tracklimits.left.size());
    std::ranges::transform(tracklimits.left, std::back_inserter(res.tracklimits.left), [](const Node &n) {
        return n.cone();
    });
    res.tracklimits.right.reserve(tracklimits.right.size());
    std::ranges::transform(tracklimits.right, std::back_inserter(res.tracklimits.right), [](const Node &n) {
        return n.cone();
    });
    res.tracklimits.replan = this->way_.quinEhLobjetiuDeLaSevaDiresio(this->lastWay_);
}

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Trace *actTrace, const KDTree &midpointsKDT,
                                const std::vector<Edge> &edges, const UrinayParams::WayComputer::Search &params) const {
    nextEdges.clear();

    const Edge *actEdge;
    Point actPos, lastPos;
    Vector dir;
    ResolveSearchContext(actTrace, edges, actEdge, actPos, lastPos, dir);

    std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params.search_radius);

    // C++20 统一容器擦除
    std::erase_if(nextPossibleEdges, [&](size_t edge_idx) {
        return shouldExcludeEdge(edges[edge_idx], actEdge, actPos, lastPos, dir, actTrace, params);
    });

    std::vector<HeurInd> privilege_runner;
    privilege_runner.reserve(nextPossibleEdges.size());
    for (const size_t &nextPossibleEdgeInd : nextPossibleEdges) {
        double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd].midPoint(), dir, params);
        if (heuristic <= params.max_next_heuristic)
            privilege_runner.emplace_back(heuristic, nextPossibleEdgeInd);
    }

    nextEdges.resize(std::min(privilege_runner.size(), (size_t)params.max_search_options));
    std::partial_sort_copy(privilege_runner.begin(), privilege_runner.end(), nextEdges.begin(), nextEdges.end());
}

Trace WayComputer::computeBestTraceWithFinishedT(const Trace &best, const Trace &t) const {
    // 最长的路径胜出；长度相同时，累计启发值更小的胜出
    if (t.size() > best.size() or (t.size() == best.size() and t.sumHeur() < best.sumHeur())) {
        return t;
    } else
        return best;
}

size_t WayComputer::treeSearch(std::vector<HeurInd> &nextEdges, const KDTree &midpointsKDT,
                               const std::vector<Edge> &edges, const UrinayParams::WayComputer::Search &params) const {
    std::queue<Trace> cua;
    for (const HeurInd &nextEdge : nextEdges) {
        bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second]);
        cua.emplace(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
    }
    if (cua.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("urinay"), "[urinay] treeSearch called with empty nextEdges, returning 0");
        return 0;
    }
    Trace best = cua.front();

    auto searchBeginTime = std::chrono::steady_clock::now();
    while (not cua.empty()) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - searchBeginTime).count();
        if (elapsed > params.max_treeSearch_time) {
            RCLCPP_WARN(rclcpp::get_logger("urinay"), "[urinay] Tree search timeout");
            break;
        }
        Trace t = cua.front();
        cua.pop();

        bool trace_at_max_height = false;
        if (params.max_search_tree_height > 0 && t.size() >= static_cast<size_t>(params.max_search_tree_height))
            trace_at_max_height = true;
        else
            this->findNextEdges(nextEdges, &t, midpointsKDT, edges, params);

        if (trace_at_max_height or nextEdges.empty()) {
            best = this->computeBestTraceWithFinishedT(best, t);
        } else {
            for (const HeurInd &nextEdge : nextEdges) {
                Point actPos = edges[t.edgeInd()].midPoint();
                bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second], &actPos);
                Trace aux = t;
                aux.addEdge(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
                cua.push(aux);
            }
        }
    }
    return best.first().edgeInd();
}

void WayComputer::computeWay(const std::vector<Edge> &edges, const UrinayParams::WayComputer::Search &params) {
    this->way_.trimByLocal();

    std::vector<Point> midpoints(edges.size());
    std::ranges::transform(edges, midpoints.begin(), &Edge::midPoint);
    KDTree midpointsKDT(midpoints);

    std::vector<HeurInd> nextEdges;
    this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);

    while (rclcpp::ok() and not nextEdges.empty() and
           (params.max_way_horizon_size <= 0 or this->way_.sizeAheadOfCar() <= static_cast<uint32_t>(params.max_way_horizon_size))) {
        size_t nextEdgeInd = this->treeSearch(nextEdges, midpointsKDT, edges, params);
        this->way_.addEdge(edges[nextEdgeInd]);

        if (this->way_.closesLoop()) {
            Way closed = this->way_.restructureClosure();
            std::scoped_lock lock(way_mutex_);
            this->wayToPublish_ = closed;
            this->isLoopClosed_ = true;
            return;
        }
        this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);
    }
    std::scoped_lock lock(way_mutex_);
    this->isLoopClosed_ = false;
    this->wayToPublish_ = this->way_;
}

/* ----------------------------- 公有方法 ----------------------------- */

WayComputer::WayComputer(const UrinayParams::WayComputer &params) : params_(params) {
    Way::init(params.way);
    this->generalFailsafe_.initGeneral(this->params_.search, this->params_.general_failsafe_safetyFactor,
                                       this->params_.failsafe_max_way_horizon_size);
}

void WayComputer::stateCallback(common_msgs::msg::HuatCarstate::ConstSharedPtr ins) {
    common_msgs::msg::HuatCarstate next_state;
    geometry_msgs::msg::Pose next_pose;
    Eigen::Affine3d next_local_tf;

    next_state.car_state = ins->car_state;
    next_state.header = ins->header;
    next_state.v = ins->v;
    next_pose.position.x = ins->car_state.x;
    next_pose.position.y = ins->car_state.y;
    next_pose.position.z = 0;
    tf2::Quaternion qAux;
    qAux.setRPY(0.0, 0.0, ins->car_state.theta);
    next_pose.orientation = tf2::toMsg(qAux);
    tf2::fromMsg(next_pose, next_local_tf);  // localTf_ 我在获取了全局姿态下的位资后。
    next_local_tf = next_local_tf.inverse();

    std::scoped_lock lock(state_mutex_);
    CarState = next_state;
    pose = next_pose;
    this->localTf_ = next_local_tf;
    this->localTfValid_ = true;
}

void WayComputer::update(TriangleSet &triangulation, const rclcpp::Time &stamp) {
    Eigen::Affine3d local_tf_snapshot;
    {
        std::scoped_lock lock(state_mutex_);
        if (not this->localTfValid_) {
            RCLCPP_WARN(rclcpp::get_logger("urinay"), "[urinay] Vehicle state not received");
            return;
        }
        local_tf_snapshot = this->localTf_;
    }

    // #0: 更新上一条路径（用于计算 replan 标志）。
    //     并更新时间戳。
    //根据代码，`this->way_` 可以认为是一个不断累加的全局路径，而 `this->lastWay_` 用于存储上一次的全局路径
    this->lastWay_ = this->way_;  // Way构造函数直接赋值  表示路径
    this->lastStamp_ = stamp;

    // #1: 移除所有已知不会成为赛道一部分的三角形
    // 实际上内部运算仍使用局部坐标系上的点进行操作。
    this->filterTriangulation(triangulation);

    // #2: 提取所有不重复的中点，通过 EdgeSet 实现，
    // 确保没有中点被重复获取。
    // 使用 EdgeSet = std::unordered_set<Edge>;
    // `std::unordered_set` 实现了一个无序集合，其中元素类型为 `Edge`
    EdgeSet edgeSet;  // 方便地存储并检索一组不重复的边对象
    for (const Triangle &t : triangulation) {
        for (const Edge &e : t.edges) {
            edgeSet.insert(e);  //调用 insert() 函数，将边e插入到edgeSet中，如果
                                // e已经存在于集合中，则不会重复插入，刚好解决了中点不被获取两次的问题
        }
    }

    // #3: 过滤中点。只保留那些与外接圆心距离较近的中点。
    this->filterMidpoints(edgeSet, triangulation);

    std::vector<Edge> edgeVec;
    edgeVec.reserve(edgeSet.size());
    for (const Edge &e : edgeSet) {
        edgeVec.push_back(e);
    }

    this->way_.updateLocal(local_tf_snapshot);
    for (const Edge &e : edgeVec) {
        e.updateLocal(local_tf_snapshot);
    }

    this->computeWay(edgeVec, this->params_.search);

    if (this->params_.general_failsafe and this->way_.sizeAheadOfCar() < MIN_FAILSAFE_WAY_SIZE and
        !this->isLoopClosed_) {
        RCLCPP_WARN(rclcpp::get_logger("urinay"), "[urinay] General failsafe activated");
        this->computeWay(edgeVec, this->generalFailsafe_);
    }

    // #7: 可视化（快照保护，避免与 computeWay 竞态）
    Way waySnapshot;
    {
        std::scoped_lock lock(way_mutex_);
        waySnapshot = this->wayToPublish_;
    }
    UrinayVisualizer::getInstance().setTimestamp(stamp);
    UrinayVisualizer::getInstance().visualize(edgeSet);
    UrinayVisualizer::getInstance().visualize(triangulation);
    UrinayVisualizer::getInstance().visualize(waySnapshot);
}

const bool &WayComputer::isLoopClosed() const {
    std::scoped_lock lock(way_mutex_);
    return this->isLoopClosed_;
}

void WayComputer::writeWayToFile(const std::string &file_path) const {
    std::scoped_lock lock(way_mutex_);
    std::ofstream oStreamToWrite(file_path);
    if (!oStreamToWrite.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("urinay"), "[urinay] Failed to open file for writing: %s", file_path.c_str());
        return;
    }
    oStreamToWrite << this->wayToPublish_;
}

bool WayComputer::isLocalTfValid() const {
    std::scoped_lock lock(state_mutex_);
    return this->localTfValid_;
}

Eigen::Affine3d WayComputer::getLocalTf() const {
    std::scoped_lock lock(state_mutex_);
    return this->localTf_;
}

std::vector<Point> WayComputer::getPath() const {
    std::scoped_lock lock(way_mutex_);
    return this->wayToPublish_.getPath();
}

Tracklimits WayComputer::getTracklimits() const {
    std::scoped_lock lock(way_mutex_);
    return this->wayToPublish_.getTracklimits();
}

common_msgs::msg::HuatPathLimits WayComputer::getPathLimits() const {
    std::scoped_lock lock(way_mutex_);
    common_msgs::msg::HuatPathLimits res;
    res.header.stamp = this->lastStamp_;
    res.header.frame_id = "map";

    // res.replan 表示路径是否与上次迭代不同
    // 判断当前 way_ 是否与上一次迭代的 lastWay_ 不同，不同则设置 replan 为 true，否则为 false。
    res.replan = this->way_ != this->lastWay_;

    AppendPointsToPath(this->wayToPublish_.getPathLocal(), res);
    FillTracklimits(res);
    return res;
}

/**
 * @brief 按 PathMode 枚举传出路径。
 * mode 4/5（局部坐标插值）和 mode 6（局部坐标）路径在 base_link 帧，其余在 map 帧。
 */
common_msgs::msg::HuatPathLimits WayComputer::getPathLimitsGlobal(PathMode mode) {
    std::scoped_lock lock(way_mutex_);
    common_msgs::msg::HuatPathLimits res;
    res.header.stamp = this->lastStamp_;
    res.header.frame_id =
        (mode == PathMode::LocalInterpLocal || mode == PathMode::FullInterpLocal || mode == PathMode::LocalCoordsPath)
            ? "base_link"
            : "map";
    geometry_msgs::msg::Pose pose_snapshot;
    {
        std::scoped_lock lock(state_mutex_);
        pose_snapshot = pose;
    }
    // res.replan 表示路径是否与上次迭代不同
    res.replan = (this->way_ != this->lastWay_);

    switch (mode) {
        case PathMode::GlobalPath:
            AppendPointsToPath(this->wayToPublish_.getPath(), res);
            break;
        case PathMode::InterpolateToNext: {
            Point nextPoint = this->wayToPublish_.getNextPathPoint();
            res.path.reserve(10);
            double diffX = (nextPoint.x - pose_snapshot.position.x) / 10.0;
            double diffY = (nextPoint.y - pose_snapshot.position.y) / 10.0;
            for (int i = 0; i < 10; i++) {
                geometry_msgs::msg::Point p;
                p.x = pose_snapshot.position.x + (i + 1) * diffX;
                p.y = pose_snapshot.position.y + (i + 1) * diffY;
                p.z = 0;
                res.path.push_back(p);
            }
            break;
        }
        case PathMode::LocalInterp:
            res.path = this->wayToPublish_.getPathInterpolation(pose_snapshot.position.x, pose_snapshot.position.y);
            break;
        case PathMode::FullInterp:
            res.path = this->wayToPublish_.getPathFullInterpolation();
            break;
        case PathMode::LocalInterpLocal:
            res.path =
                this->wayToPublish_.getPathInterpolationLocal(pose_snapshot.position.x, pose_snapshot.position.y);
            break;
        case PathMode::FullInterpLocal:
            res.path = this->wayToPublish_.getPathFullInterpolationLocal();
            break;
        case PathMode::LocalCoordsPath:
            AppendPointsToPath(this->wayToPublish_.getPathLocal(), res);
            break;
    }

    FillTracklimits(res);
    return res;
}

common_msgs::msg::HuatCarstate WayComputer::getCarState() {
    std::scoped_lock lock(state_mutex_);
    return CarState;
}
