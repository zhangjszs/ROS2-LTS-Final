/*
 * file: KDTree.cpp
 *
 * 平坦数组 KD-tree 实现。
 * 内部用连续 std::vector<Node> 代替原 shared_ptr 树，
 * 消除每帧大量堆分配和 CPU cache miss。
 */

#include "utils/KDTree.hpp"

#include <algorithm>
#include <cmath>

// ---------- 建树 ----------

int KDTree::buildRec(std::vector<Entry>& tmp, int lo, int hi, int lv) {
    if (lo >= hi)
        return -1;

    int mid = lo + (hi - lo) / 2;
    std::nth_element(tmp.begin() + lo, tmp.begin() + mid, tmp.begin() + hi,
                     [lv](const Entry& a, const Entry& b) { return a.first[lv] < b.first[lv]; });

    int ni = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});
    Node& n = nodes_.back();
    n.c[0] = tmp[mid].first[0];
    n.c[1] = tmp[mid].first[1];
    n.idx = tmp[mid].second;

    // 注意：push_back 可能使 nodes_ 重分配，必须先拿 ni 再递归
    int lc = buildRec(tmp, lo, mid, 1 - lv);
    int rc = buildRec(tmp, mid + 1, hi, 1 - lv);
    nodes_[ni].left = lc;
    nodes_[ni].right = rc;
    return ni;
}

KDTree::KDTree(const std::vector<Point>& pts) {
    if (pts.empty())
        return;
    pts_ = pts;
    nodes_.reserve(pts.size());

    std::vector<Entry> tmp(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        tmp[i] = {{pts[i].x, pts[i].y}, i};

    buildRec(tmp, 0, static_cast<int>(tmp.size()), 0);
}

KDTree::KDTree(const std::list<Point>& pts) {
    if (pts.empty())
        return;
    pts_.assign(pts.begin(), pts.end());
    nodes_.reserve(pts_.size());

    std::vector<Entry> tmp(pts_.size());
    for (size_t i = 0; i < pts_.size(); ++i)
        tmp[i] = {{pts_[i].x, pts_[i].y}, i};

    buildRec(tmp, 0, static_cast<int>(tmp.size()), 0);
}

// ---------- 最近邻 ----------

void KDTree::searchNN(int ni, const double q[2], int lv, int& best, double& bestD2,
                      const std::set<size_t>& excs) const {
    if (ni < 0)
        return;
    const Node& n = nodes_[ni];

    double dx = q[0] - n.c[0];
    double dy = q[1] - n.c[1];
    double d2 = dx * dx + dy * dy;

    if (excs.find(n.idx) == excs.end()) {
        if (best < 0 || d2 < bestD2) {
            bestD2 = d2;
            best = ni;
        }
    }

    double diff = q[lv] - n.c[lv];
    int near = diff <= 0 ? n.left : n.right;
    int far = diff <= 0 ? n.right : n.left;

    searchNN(near, q, 1 - lv, best, bestD2, excs);
    if (diff * diff < bestD2 || best < 0)
        searchNN(far, q, 1 - lv, best, bestD2, excs);
}

KDTData<size_t> KDTree::nearest_index(const Point& pt, const std::set<size_t>& excs) const {
    if (nodes_.empty())
        return KDTData<size_t>();
    int best = -1;
    double bestD2 = std::numeric_limits<double>::max();
    double q[2] = {pt.x, pt.y};
    searchNN(0, q, 0, best, bestD2, excs);
    if (best < 0)
        return KDTData<size_t>();
    return KDTData<size_t>(nodes_[best].idx);
}

KDTData<Point> KDTree::nearest_point(const Point& pt, const std::set<size_t>& excs) const {
    auto idx = nearest_index(pt, excs);
    if (!idx)
        return KDTData<Point>();
    return KDTData<Point>(pts_[*idx]);
}

pointIndexV KDTree::nearest_pointIndex(const Point& pt, const std::set<size_t>& excs) const {
    auto idx = nearest_index(pt, excs);
    if (!idx)
        return pointIndexV();
    return pointIndexV(pointIndex(pts_[*idx], *idx));
}

// ---------- 半径查询 ----------

void KDTree::searchRadius(int ni, const double q[2], double r2, int lv, std::vector<size_t>& out) const {
    if (ni < 0)
        return;
    const Node& n = nodes_[ni];

    double dx = q[0] - n.c[0];
    double dy = q[1] - n.c[1];
    if (dx * dx + dy * dy <= r2)
        out.push_back(n.idx);

    double diff = q[lv] - n.c[lv];
    searchRadius(diff <= 0 ? n.left : n.right, q, r2, 1 - lv, out);
    if (diff * diff <= r2)
        searchRadius(diff <= 0 ? n.right : n.left, q, r2, 1 - lv, out);
}

pointIndexArr KDTree::neighborhood(const Point& pt, const double& rad) const {
    if (nodes_.empty())
        return {};
    std::vector<size_t> idxs;
    double q[2] = {pt.x, pt.y};
    searchRadius(0, q, rad * rad, 0, idxs);
    pointIndexArr res;
    res.reserve(idxs.size());
    for (size_t i : idxs)
        res.emplace_back(pts_[i], i);
    return res;
}

std::vector<Point> KDTree::neighborhood_points(const Point& pt, const double& rad) const {
    if (nodes_.empty())
        return {};
    std::vector<size_t> idxs;
    double q[2] = {pt.x, pt.y};
    searchRadius(0, q, rad * rad, 0, idxs);
    std::vector<Point> res;
    res.reserve(idxs.size());
    for (size_t i : idxs)
        res.push_back(pts_[i]);
    return res;
}

indexArr KDTree::neighborhood_indices(const Point& pt, const double& rad) const {
    if (nodes_.empty())
        return {};
    std::vector<size_t> idxs;
    double q[2] = {pt.x, pt.y};
    searchRadius(0, q, rad * rad, 0, idxs);
    return idxs;
}

std::unordered_set<size_t> KDTree::neighborhood_indices_set(const Point& pt, const double& rad) const {
    if (nodes_.empty())
        return {};
    std::vector<size_t> idxs;
    double q[2] = {pt.x, pt.y};
    searchRadius(0, q, rad * rad, 0, idxs);
    return std::unordered_set<size_t>(idxs.begin(), idxs.end());
}
