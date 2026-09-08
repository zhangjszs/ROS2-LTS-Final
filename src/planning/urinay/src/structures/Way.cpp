/**
 * @file Way.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Way 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Way.hpp"

#include <cassert>

/* ----------------------------- 私有方法 ---------------------------- */

UrinayParams::WayComputer::Way Way::params_;

// 更新 way 对象中与车辆最接近的元素位置
void Way::updateClosestToCarElem() {
    // 默认指向末尾（表示没有车前方的边）
    auto smallestDWFIt = this->path_.cend();
    if (!this->path_.empty()) {
        double smallestDWF = std::numeric_limits<double>::max();
        bool found = false;

        // 只考虑车辆前方（x > 0）的边
        for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++) {
            Point point = it->midPoint();
            if (point.x > 0) {
                double distSq = Point::distSq(point);
                if (!found || distSq <= smallestDWF) {
                    smallestDWF = distSq;
                    smallestDWFIt = it;
                    found = true;
                }
            }
        }
    }
    this->closestToCarElem_ = smallestDWFIt;
}

bool Way::segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D) {
    return Point::ccw(A, C, D) != Point::ccw(B, C, D) and Point::ccw(A, B, C) != Point::ccw(A, B, D);
}

/* ----------------------------- 公有方法 ---------------------------- */

void Way::init(const UrinayParams::WayComputer::Way &params) {
    params_ = params;
}

Way::Way() : avgEdgeLen_(0.0) {
    closestToCarElem_ =
        this->path_.cend();  // 返回指向 path_ 列表末尾的常量迭代器，表示迭代范围的终点，不指向任何有效元素。
    sizeToCar_ = 0;
}

Way::Way(const Way &way)
    : path_(way.path_),
      avgEdgeLen_(way.avgEdgeLen_),
      sizeToCar_(way.sizeToCar_) {
    updateClosestToCarElem();
}

Way::Way(Way &&way) noexcept
    : path_(std::move(way.path_)),
      avgEdgeLen_(way.avgEdgeLen_),
      sizeToCar_(way.sizeToCar_) {
    updateClosestToCarElem();
}

bool Way::empty() const {
    return this->path_.empty();
}

size_t Way::size() const {
    return this->path_.size();
}

const Edge &Way::back() const {
    return this->path_.back();
}

const Edge &Way::beforeBack() const {
    assert(this->size() >= 2);
    return *(++this->path_.rbegin());
}

const Edge &Way::front() const {
    return this->path_.front();
}

void Way::updateLocal(const Eigen::Affine3d &tf) {
    for (Edge &e : this->path_) {
        e.updateLocal(tf);
    }
    // 更新最近点
    this->updateClosestToCarElem();
}

void Way::addEdge(const Edge &edge) {
    this->path_.push_back(edge);
    if (this->path_.size() == 1)
        closestToCarElem_ = this->path_.cbegin();
    this->avgEdgeLen_ += (edge.len - this->avgEdgeLen_) / this->size();
}

void Way::trimByLocal() {
    if (this->size() < 2)
        return;
    auto closestToCarElem = this->closestToCarElem_;

    // 所有边都在车后方时，保留最后几条边作为方向上下文，而不是完全清空
    if (closestToCarElem == this->path_.cend()) {
        constexpr int kKeepBehindEdges = 3;
        if (this->size() > static_cast<size_t>(kKeepBehindEdges)) {
            this->path_.erase(this->path_.begin(), std::prev(this->path_.cend(), kKeepBehindEdges));
        }
        this->sizeToCar_ = this->size();
        // 重算 avgEdgeLen
        size_t t = 1;
        this->avgEdgeLen_ = 0.0;
        for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++) {
            this->avgEdgeLen_ += (it->len - this->avgEdgeLen_) / t;
            t++;
        }
        return;
    }

    if (closestToCarElem != std::prev(this->path_.cend())) {
        // 删除最接近元素之后的所有元素，即删除车辆后方的边
        auto eraseStart = closestToCarElem;
        ++eraseStart;
        this->path_.erase(eraseStart, this->path_.cend());
    }

    // 重新计算平均边长属性 avgEdgeLen_
    // 公式含义：每次将当前边长与当前平均边长的偏差除以迭代次数 t，再累加到平均边长上
    size_t t = 1;
    this->avgEdgeLen_ = 0.0;
    for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++) {
        this->avgEdgeLen_ += (it->len - this->avgEdgeLen_) / t;
        t++;
    }

    // 重新计算 sizeToCar_ 属性
    this->sizeToCar_ = this->size();
}

bool Way::closesLoop() const {
    return this->size() >= MIN_LOOP_SIZE and Point::distSq(this->front().midPoint(), this->back().midPoint()) <=
                                                 params_.max_dist_loop_closure * params_.max_dist_loop_closure;
}

// 闭环检查
// a. 路径长度（this->size()）加 1 是否大于等于最小闭环长度（MIN_LOOP_SIZE）。
// b. 路径第一个点的中点（this->front().midPoint()）与给定边的中点（e.midPoint()）
//    之间的距离平方是否小于等于 params_.max_dist_loop_closure 的平方（距离条件）。
// c. 使用路径第一个点和第二个点的中点构造两个向量，计算其夹角绝对值（角度差），
//    检查角度差是否小于等于 params_.max_angle_diff_loop_closure（角度条件）。
// 若以上所有条件均满足，则返回 true，否则返回 false。
bool Way::closesLoopWith(const Edge &e, const Point *lastPosInTrace) const {
    Point actPos = lastPosInTrace ? *lastPosInTrace : this->back().midPoint();
    return
        // 必须满足最小长度
        this->size() + 1 >= MIN_LOOP_SIZE and
        // 距离条件满足
        Point::distSq(this->front().midPoint(), e.midPoint()) <=
            params_.max_dist_loop_closure * params_.max_dist_loop_closure and
        // 检查与第一个点的闭合角度
        std::abs(Vector(this->front().midPoint(), (++this->path_.begin())->midPoint())
                .angleWith(Vector(actPos, e.midPoint()))) <= params_.max_angle_diff_loop_closure;
}

Way Way::restructureClosure() const {
    Way res = *this;
    if (res.front() != res.back()) {
        // 假设最后一条边是闭环边
        double distClosestWithLast = Point::distSq(res.front().midPoint(), res.back().midPoint());
        auto closestWithLastIt = res.path_.begin();

        for (auto it = res.path_.begin(); it != std::prev(res.path_.end(), 5); it++) {  // 5 是安全余量
            double distWithLast = Point::distSq(res.back().midPoint(), it->midPoint());
            if (distWithLast <= distClosestWithLast) {
                distClosestWithLast = distWithLast;
                closestWithLastIt = it;
            }
        }

        // 移除所有会导致“环”不闭合的边（即闭环边之前的所有边）
        res.path_.erase(res.path_.begin(), closestWithLastIt);
    }
    // 此处最后一个中点（现在最接近第一个中点）将被第一个中点替换，
    // 确保它们完全相同（id 和 值）。
    if (res.front() == res.back() or
        Vector::pointBehind(res.back().midPointGlobal(), res.front().midPointGlobal(),
                            Vector(res.beforeBack().midPointGlobal(), res.back().midPointGlobal())) or
        Point::dist(res.back().midPointGlobal(), res.front().midPointGlobal()) < SAME_MIDPOINT_DIST_THRESHOLD) {
        res.path_.pop_back();
    }
    res.path_.push_back(res.front());

    return res;
}

bool Way::intersectsWith(const Edge &e) const {
    if (this->size() <= 2)
        return false;
    Point s1p1, s1p2;
    const Point s2p1 = this->back().midPoint();
    const Point s2p2 = e.midPoint();
    auto it1 = std::next(this->path_.crbegin());
    auto it2 = std::next(it1);
    while (it2 != this->path_.crend()) {
        s1p1 = it1->midPoint();
        s1p2 = it2->midPoint();
        if (this->segmentsIntersect(s1p1, s1p2, s2p1, s2p2))
            return true;
        it1++;
        it2++;
    }
    return false;
}

bool Way::containsEdge(const Edge &e) const {
    for (const Edge &edge : this->path_) {
        if (e == edge)
            return true;
    }
    return false;
}

std::vector<Point> Way::getPath() const {
    std::vector<Point> res;
    res.reserve(this->path_.size());
    for (const Edge &e : this->path_) {
        res.push_back(e.midPointGlobal());
    }
    return res;
}

std::vector<Point> Way::getPathLocal() const {
    std::vector<Point> res;
    res.reserve(this->path_.size());
    for (const Edge &e : this->path_) {
        res.push_back(e.midPoint());
    }
    return res;
}

Tracklimits Way::getTracklimits() const {
    Tracklimits res;
    res.left.reserve(this->size());
    res.right.reserve(this->size());
    Point pAnt = this->empty() ? Point(0, 0)
                               : Point(-50, this->front().midPointGlobal().y);  // 仅在全局坐标系下有效，-50 为任意值

    const Node *left, *firstLeft;
    const Node *right, *firstRight;

    size_t edgeInd = 0;
    for (const Edge &e : this->path_) {
        Vector pAntPAct(pAnt, e.midPointGlobal());

        // 检查边两个节点的左右侧
        if (Vector::pointBehind(e.n0.pointGlobal(), e.midPointGlobal(), pAntPAct.rotClock())) {
            left = &e.n0;
            right = &e.n1;
        } else {
            left = &e.n1;
            right = &e.n0;
        }

        // 保存第一个节点
        if (edgeInd == 0) {
            firstLeft = left;
            firstRight = right;
        }

        // 仅追加尚未添加过的节点（去重）
        // 移除了方向过滤（pointBehind），避免 C 型弯中段锥桶因行进方向翻转而被误判跳过
        if (res.left.empty() or *left != res.left.back()) {
            if (*left == *firstLeft) {
                res.left.push_back(*firstLeft);
            } else
                res.left.push_back(*left);
        }
        if (res.right.empty() or *right != res.right.back()) {
            if (*right == *firstRight) {
                res.right.push_back(*firstRight);
            } else
                res.right.push_back(*right);
        }

        pAnt = e.midPointGlobal();
        edgeInd++;
    }
    return res;
}

Way &Way::operator=(const Way &way) {
    if (this != &way) {
        this->path_ = std::list<Edge>(way.path_);
        this->avgEdgeLen_ = way.avgEdgeLen_;
        this->sizeToCar_ = way.sizeToCar_;
        this->updateClosestToCarElem();  // 直接复制该属性是不安全的
    }
    return *this;
}

Way &Way::operator=(Way &&way) noexcept {
    if (this != &way) {
        this->path_ = std::move(way.path_);
        this->avgEdgeLen_ = way.avgEdgeLen_;
        this->sizeToCar_ = way.sizeToCar_;
        this->updateClosestToCarElem();
    }
    return *this;
}

bool Way::quinEhLobjetiuDeLaSevaDiresio(const Way &way) const {
    if (this->empty() != way.empty())
        return true;
    if (this->empty())
        return false;
    auto wayIt = way.closestToCarElem_;
    auto thisIt = this->closestToCarElem_;
    if (wayIt == way.path_.cend() || thisIt == this->path_.cend())
        return (wayIt == way.path_.cend()) != (thisIt == this->path_.cend());

    while (wayIt != way.path_.cend()) {
        if (*wayIt == *thisIt) {
            break;
        }
        wayIt++;
    }

    if (wayIt == way.path_.cend() || *wayIt != *thisIt)
        return true;

    // 逐个比较 vital_num_midpoints
    int midpoint_num = 0;
    while (wayIt != way.path_.cend() and thisIt != this->path_.cend()) {
        if (midpoint_num >= this->params_.vital_num_midpoints)
            return false;
        if (*wayIt != *thisIt)
            return true;
        midpoint_num++;
        wayIt++;
        thisIt++;
    }

    // 当两个 Way 中有一个到达末尾时，可以检查在 vital_num_midpoints 范围内
    // 是否一个比另一个长并据此返回，但这种情况由规划器自身处理更好
    //（当没有中点时会重新规划）。
    return false;
}

const double &Way::getAvgEdgeLen() const {
    return this->avgEdgeLen_;
}

Vector Way::trackDirection() const {
    if (this->size() >= 2) {
        return Vector((++this->path_.rbegin())->midPoint(), this->path_.rbegin()->midPoint());
    }
    return Vector(1, 0);
}

uint32_t Way::sizeAheadOfCar() const {
    return this->path_.size() - this->sizeToCar_;
}

std::ostream &operator<<(std::ostream &os, const Way &way) {
    const Tracklimits tracklimits = way.getTracklimits();
    for (const Node &n : tracklimits.left) {
        os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 0 << ' ' << n.id << std::endl;
    }
    for (const Node &n : tracklimits.right) {
        os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 1 << ' ' << n.id << std::endl;
    }
    return os;
}

void Way::deleteWayPassed() {
    if (this->size() < 2)
        return;
    if (closestToCarElem_ == path_.cend() || closestToCarElem_ == path_.cbegin())
        return;
    path_.erase(path_.cbegin(), closestToCarElem_);
}

Point Way::getNextPathPoint() {
    Point point;
    if (path_.empty() || closestToCarElem_ == path_.cend()) {
        point.x = 0;
        point.y = 0;
        return point;
    }
    auto closestToCarElem = this->closestToCarElem_;
    point.x = closestToCarElem->midPointGlobal().x;
    point.y = closestToCarElem->midPointGlobal().y;
    return point;
}

std::vector<geometry_msgs::msg::Point> Way::getPathInterpolation(double x, double y) {
    deleteWayPassed();
    std::vector<geometry_msgs::msg::Point> res;
    if (path_.empty())
        return res;
    auto it = this->path_.cbegin();

    geometry_msgs::msg::Point p;
    double diffX = (it->midPointGlobal().x - x) / 10.0;
    double diffY = (it->midPointGlobal().y - y) / 10.0;

    for (int i = 0; i < 10; i++) {
        p.x = x + diffX * i;
        p.y = y + diffY * i;
        res.push_back(p);
    }

    auto lastIt = it;
    it++;

    for (size_t j = 0; j + 1 < path_.size(); ++j) {
        diffX = (it->midPointGlobal().x - lastIt->midPointGlobal().x) / 10.0;
        diffY = (it->midPointGlobal().y - lastIt->midPointGlobal().y) / 10.0;

        for (int i = 0; i < 10; i++) {
            p.x = lastIt->midPointGlobal().x + diffX * i;
            p.y = lastIt->midPointGlobal().y + diffY * i;
            res.push_back(p);
        }
        lastIt = it;
        it++;
    }
    return res;
}

std::vector<geometry_msgs::msg::Point> Way::getPathFullInterpolation() {
    std::vector<geometry_msgs::msg::Point> res;
    if (path_.size() < 2)
        return res;
    auto it = this->path_.cbegin();
    auto lastIt = it;
    double diffX, diffY;
    geometry_msgs::msg::Point p;
    it++;  //跳到下一个
    for (size_t j = 0; j + 1 < path_.size(); ++j) {
        diffX = (it->midPointGlobal().x - lastIt->midPointGlobal().x) / 10.0;
        diffY = (it->midPointGlobal().y - lastIt->midPointGlobal().y) / 10.0;

        for (int i = 0; i < 10; i++) {
            p.x = lastIt->midPointGlobal().x + diffX * i;
            p.y = lastIt->midPointGlobal().y + diffY * i;
            res.push_back(p);
        }
        lastIt = it;
        it++;
    }
    return res;
}

std::vector<geometry_msgs::msg::Point> Way::getPathInterpolationLocal([[maybe_unused]] double x, [[maybe_unused]] double y) {
    deleteWayPassed();
    std::vector<geometry_msgs::msg::Point> res;
    if (path_.empty())
        return res;
    auto it = this->path_.cbegin();

    geometry_msgs::msg::Point p;
    double diffX = (it->midPoint().x) / 10.0;
    double diffY = (it->midPoint().y) / 10.0;

    for (int i = 0; i < 10; i++) {
        p.x = diffX * i;
        p.y = diffY * i;
        res.push_back(p);
    }

    auto lastIt = it;
    it++;

    for (size_t j = 0; j + 1 < path_.size(); ++j) {
        diffX = (it->midPoint().x - lastIt->midPoint().x) / 10.0;
        diffY = (it->midPoint().y - lastIt->midPoint().y) / 10.0;

        for (int i = 0; i < 10; i++) {
            p.x = lastIt->midPoint().x + diffX * i;
            p.y = lastIt->midPoint().y + diffY * i;
            res.push_back(p);
        }
        lastIt = it;
        it++;
    }
    return res;
}

std::vector<geometry_msgs::msg::Point> Way::getPathFullInterpolationLocal() {
    std::vector<geometry_msgs::msg::Point> res;
    if (path_.size() < 2)
        return res;
    auto it = this->path_.cbegin();
    auto lastIt = it;
    double diffX, diffY;
    geometry_msgs::msg::Point p;
    it++;  //跳到下一个
    for (size_t j = 0; j + 1 < path_.size(); ++j) {
        diffX = (it->midPoint().x - lastIt->midPoint().x) / 10.0;
        diffY = (it->midPoint().y - lastIt->midPoint().y) / 10.0;

        for (int i = 0; i < 10; i++) {
            p.x = lastIt->midPoint().x + diffX * i;
            p.y = lastIt->midPoint().y + diffY * i;
            res.push_back(p);
        }
        lastIt = it;
        it++;
    }
    return res;
}
