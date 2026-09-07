#include "icp_apf_planner.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace {

/**
 * @brief 使用 KD-tree 查找 src 中每个点在 tgt 中的最近邻（O(|src| log |tgt|)）。
 * 仅保留距离 ≤ max_dist_sq 的配对。
 */
static std::pair<std::vector<skidpad::Point2D>, std::vector<skidpad::Point2D>> FindNearestPairsKDTree(
    const std::vector<skidpad::Point2D>& src, const std::vector<skidpad::Point2D>& tgt, double max_dist_sq) {
    std::vector<skidpad::Point2D> ms, mt;
    if (src.empty() || tgt.empty()) {
        return {ms, mt};
    }

    // 将 tgt 构建为 PCL 点云并建 KD-tree
    auto tgt_cloud = std::make_shared<pcl::PointCloud<pcl::PointXY>>();
    tgt_cloud->points.reserve(tgt.size());
    for (const auto& q : tgt) {
        pcl::PointXY pt;
        pt.x = static_cast<float>(q.x);
        pt.y = static_cast<float>(q.y);
        tgt_cloud->points.push_back(pt);
    }
    tgt_cloud->width = tgt_cloud->points.size();
    tgt_cloud->height = 1;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(tgt_cloud);

    ms.reserve(src.size());
    mt.reserve(src.size());
    for (const auto& p : src) {
        pcl::PointXY query;
        query.x = static_cast<float>(p.x);
        query.y = static_cast<float>(p.y);
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        if (kdtree.nearestKSearch(query, 1, idx, dist_sq) == 1 && dist_sq[0] <= max_dist_sq) {
            ms.push_back(p);
            mt.push_back(tgt[idx[0]]);
        }
    }
    return {ms, mt};
}

static Eigen::Vector2d Centroid2D(const std::vector<skidpad::Point2D>& pts) {
    Eigen::Vector2d c(0.0, 0.0);
    for (const auto& p : pts)
        c += Eigen::Vector2d(p.x, p.y);
    return c / static_cast<double>(pts.size());
}

static Eigen::Matrix2d CovarianceH2D(const std::vector<skidpad::Point2D>& src, const std::vector<skidpad::Point2D>& tgt,
                                     const Eigen::Vector2d& sc, const Eigen::Vector2d& tc) {
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < src.size(); ++i)
        H += (Eigen::Vector2d(src[i].x, src[i].y) - sc) * (Eigen::Vector2d(tgt[i].x, tgt[i].y) - tc).transpose();
    return H;
}

static Eigen::Matrix2d SvdRotation2D(const Eigen::Matrix2d& H) {
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix2d Vr = svd.matrixV();
        Vr.col(1) *= -1;
        R = Vr * svd.matrixU().transpose();
    }
    return R;
}

/**
 * @brief 对 primary 中每个锥桶，在 secondary 中找最近邻（KD-tree），
 * 若距离 ≤ max_dist 则取中点加入 out。
 */
static void AppendMidpoints(const std::vector<common_msgs::msg::HuatCone>& primary,
                            const std::vector<common_msgs::msg::HuatCone>& secondary, double max_dist,
                            std::vector<skidpad::Point2D>& out) {
    if (primary.empty() || secondary.empty())
        return;

    const double max_dist_sq = max_dist * max_dist;

    // 用 secondary 构建 KD-tree
    pcl::PointCloud<pcl::PointXY>::Ptr sec_cloud(new pcl::PointCloud<pcl::PointXY>);
    sec_cloud->points.reserve(secondary.size());
    for (const auto& s : secondary) {
        pcl::PointXY pt;
        pt.x = static_cast<float>(s.position_base_link.x);
        pt.y = static_cast<float>(s.position_base_link.y);
        sec_cloud->points.push_back(pt);
    }
    sec_cloud->width = sec_cloud->points.size();
    sec_cloud->height = 1;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(sec_cloud);

    for (const auto& p : primary) {
        double px = p.position_base_link.x, py = p.position_base_link.y;
        pcl::PointXY query;
        query.x = static_cast<float>(px);
        query.y = static_cast<float>(py);
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        if (kdtree.nearestKSearch(query, 1, idx, dist_sq) == 1 && dist_sq[0] <= max_dist_sq) {
            const auto& best = secondary[idx[0]];
            out.emplace_back((px + best.position_base_link.x) * 0.5, (py + best.position_base_link.y) * 0.5);
        }
    }
}

}  // namespace

namespace skidpad {

IcpApfPlanner::IcpApfPlanner(const IcpApfConfig& cfg)
    : center_margin_(cfg.center_margin),
      icp_max_iter_(cfg.icp_max_iter),
      icp_convergence_(cfg.icp_convergence),
      k_a_(cfg.k_a),
      k_r_(cfg.k_r),
      d_0_(cfg.d_0),
      path_spacing_(cfg.path_spacing),
      path_lookahead_(cfg.path_lookahead),
      apf_max_iter_(cfg.apf_max_iter),
      apf_step_limit_(cfg.apf_step_limit),
      icp_max_match_dist_(cfg.icp_max_match_dist) {}

void IcpApfPlanner::ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                                 std::vector<common_msgs::msg::HuatCone>& left,
                                 std::vector<common_msgs::msg::HuatCone>& right) {
    left.clear();
    right.clear();
    std::ranges::copy_if(cones, std::back_inserter(left),
                         [this](float y) { return y < -center_margin_; },
                         [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::copy_if(cones, std::back_inserter(right),
                         [this](float y) { return y > center_margin_; },
                         [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
}

std::vector<Point2D> IcpApfPlanner::ComputeCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                                      const std::vector<common_msgs::msg::HuatCone>& right) {
    if (left.empty() || right.empty())
        return {};

    constexpr double kMaxPairDist = 6.0;
    std::vector<Point2D> centerline;
    centerline.reserve(left.size() + right.size());

    AppendMidpoints(left, right, kMaxPairDist, centerline);
    AppendMidpoints(right, left, kMaxPairDist, centerline);

    // 从原点最近邻串起来，避免按 x 排序在圆环上锯齿化
    std::vector<Point2D> ordered;
    ordered.reserve(centerline.size());
    std::vector<bool> used(centerline.size(), false);
    auto take_nearest = [&](double x0, double y0) -> int {
        auto unused_indices = std::views::iota(size_t{0}, centerline.size())
                            | std::views::filter([&used](size_t i) { return !used[i]; });
        auto it = std::ranges::min_element(unused_indices, {}, [&](size_t i) {
            const double dx = centerline[i].x - x0;
            const double dy = centerline[i].y - y0;
            return dx * dx + dy * dy;
        });
        if (it == unused_indices.end())
            return -1;
        return static_cast<int>(*it);
    };
    int start = take_nearest(0.0, 0.0);
    while (start >= 0) {
        used[static_cast<size_t>(start)] = true;
        ordered.push_back(centerline[static_cast<size_t>(start)]);
        start = take_nearest(ordered.back().x, ordered.back().y);
    }
    centerline.swap(ordered);

    const double dedup_sq = (path_spacing_ * 0.5) * (path_spacing_ * 0.5);
    std::vector<Point2D> deduped;
    deduped.reserve(centerline.size());
    for (const auto& p : centerline) {
        if (deduped.empty()) {
            deduped.push_back(p);
            continue;
        }
        double dx = p.x - deduped.back().x, dy = p.y - deduped.back().y;
        if (dx * dx + dy * dy > dedup_sq)
            deduped.push_back(p);
    }

    std::vector<Point2D> result;
    result.reserve(deduped.size());
    const double lookahead_sq = path_lookahead_ * path_lookahead_;
    std::ranges::copy_if(deduped, std::back_inserter(result),
        [lookahead_sq](double d2) { return d2 <= lookahead_sq; },
        [](const Point2D& p) { return p.x * p.x + p.y * p.y; });
    return result;
}

Circle2D IcpApfPlanner::FitCircleTaubin(const std::vector<common_msgs::msg::HuatCone>& cones) {
    Circle2D result;
    const int n = static_cast<int>(cones.size());
    if (n < 3)
        return result;

    double mean_x = 0.0, mean_y = 0.0;
    for (const auto& c : cones) {
        mean_x += c.position_base_link.x;
        mean_y += c.position_base_link.y;
    }
    mean_x /= n;
    mean_y /= n;

    // 计算各阶矩（以质心为原点的坐标系）
    double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
    for (const auto& c : cones) {
        double u = c.position_base_link.x - mean_x;
        double v = c.position_base_link.y - mean_y;
        double z = u * u + v * v;
        Mxx += u * u;
        Myy += v * v;
        Mxy += u * v;
        Mxz += u * z;
        Myz += v * z;
        Mzz += z * z;
    }
    Mxx /= n;
    Myy /= n;
    Mxy /= n;
    Mxz /= n;
    Myz /= n;
    Mzz /= n;
    const double Mz = Mxx + Myy;
    if (Mz < 1e-10)
        return result;

    // 构造 B^{-1}*M 矩阵（广义特征值问题的标准化）
    // B = diag(4*Mz, 1, 1)，B^{-1}*M 的最小正特征值对应最优圆参数
    Eigen::Matrix3d A;
    A << Mzz / (4.0 * Mz), Mxz / (4.0 * Mz), Myz / (4.0 * Mz), Mxz, Mxx, Mxy, Myz, Mxy, Myy;

    Eigen::EigenSolver<Eigen::Matrix3d> es(A);

    double min_eval = std::numeric_limits<double>::max();
    Eigen::Vector3d min_evec = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(es.eigenvalues()[i].imag()) > 1e-8)
            continue;
        double eval = es.eigenvalues()[i].real();
        if (eval > 1e-10 && eval < min_eval) {
            min_eval = eval;
            min_evec = es.eigenvectors().col(i).real();
        }
    }
    if (min_eval == std::numeric_limits<double>::max())
        return result;

    const double A_coef = min_evec(0);
    if (std::abs(A_coef) < 1e-10)
        return result;

    // 圆心（质心坐标系）：(-B/2A, -C/2A)
    // 半径²：cx_c² + cy_c² + Mz（其中 D = -A*Mz）
    const double cx_c = -min_evec(1) / (2.0 * A_coef);
    const double cy_c = -min_evec(2) / (2.0 * A_coef);
    const double r_sq = cx_c * cx_c + cy_c * cy_c + Mz;
    if (r_sq <= 0.0)
        return result;

    result.cx = cx_c + mean_x;
    result.cy = cy_c + mean_y;
    result.r = std::sqrt(r_sq);
    result.valid = true;
    return result;
}

std::vector<Point2D> IcpApfPlanner::ComputeCircleCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                                            const std::vector<common_msgs::msg::HuatCone>& right) {
    constexpr double kDefaultHalfTrackWidth = 1.5;  // 赛道半宽 ~1.5m
    constexpr double kMinRadius = 3.0;
    constexpr double kMaxRadius = 25.0;

    Circle2D lc = FitCircleTaubin(left);
    Circle2D rc = FitCircleTaubin(right);

    Circle2D center_circle;
    if (lc.valid && rc.valid) {
        center_circle.cx = (lc.cx + rc.cx) * 0.5;
        center_circle.cy = (lc.cy + rc.cy) * 0.5;
        center_circle.r = (lc.r + rc.r) * 0.5;
        center_circle.valid = true;
    } else if (lc.valid) {
        center_circle.cx = lc.cx;
        center_circle.cy = lc.cy;
        center_circle.r = std::max(lc.r - kDefaultHalfTrackWidth, lc.r * 0.5);
        center_circle.valid = true;
    } else if (rc.valid) {
        center_circle.cx = rc.cx;
        center_circle.cy = rc.cy;
        center_circle.r = std::max(rc.r - kDefaultHalfTrackWidth, rc.r * 0.5);
        center_circle.valid = true;
    }

    if (!center_circle.valid || center_circle.r < kMinRadius || center_circle.r > kMaxRadius) {
        return {};
    }

    RCLCPP_INFO(rclcpp::get_logger("skidpad_planner"),
                "[skidpad_planner] Taubin: left(valid=%d r=%.2f) right(valid=%d r=%.2f) center(cx=%.2f cy=%.2f r=%.2f)",
                lc.valid, lc.r, rc.valid, rc.r, center_circle.cx, center_circle.cy, center_circle.r);

    // 从车辆当前位置（base_link 原点）出发沿弧线采样
    // 车辆相对圆心的角度
    const double start_angle = std::atan2(-center_circle.cy, -center_circle.cx);
    const double arc_step = path_spacing_ / center_circle.r;
    auto first_x = [&](double dir) {
        double angle = start_angle + dir * arc_step;
        return center_circle.cx + center_circle.r * std::cos(angle);
    };
    const double dir = (first_x(1.0) >= first_x(-1.0)) ? 1.0 : -1.0;
    const int num_steps = static_cast<int>(path_lookahead_ / path_spacing_) + 1;

    std::vector<Point2D> path;
    path.reserve(num_steps);
    for (int i = 1; i <= num_steps; ++i) {
        double angle = start_angle + dir * arc_step * static_cast<double>(i);
        double px = center_circle.cx + center_circle.r * std::cos(angle);
        double py = center_circle.cy + center_circle.r * std::sin(angle);
        path.emplace_back(px, py);
    }
    return path;
}

IcpResult IcpApfPlanner::IcpAlign(const std::vector<Point2D>& src, const std::vector<Point2D>& tgt) {
    IcpResult result;
    if (src.empty() || tgt.empty())
        return result;

    std::vector<Point2D> aligned = src;
    const double max_dist_sq = icp_max_match_dist_ * icp_max_match_dist_;
    double prev_rmse = std::numeric_limits<double>::max();

    for (int iter = 0; iter < icp_max_iter_; ++iter) {
        auto [ms, mt] = FindNearestPairsKDTree(aligned, tgt, max_dist_sq);
        if (ms.empty())
            break;

        Eigen::Vector2d sc = Centroid2D(ms);
        Eigen::Vector2d tc = Centroid2D(mt);
        Eigen::Matrix2d R = SvdRotation2D(CovarianceH2D(ms, mt, sc, tc));
        Eigen::Vector2d t = tc - R * sc;

        for (auto& p : aligned) {
            Eigen::Vector2d pv = R * Eigen::Vector2d(p.x, p.y) + t;
            p.x = pv.x();
            p.y = pv.y();
        }

        size_t n = ms.size();
        double rmse = 0.0;
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector2d pv = R * Eigen::Vector2d(ms[i].x, ms[i].y) + t;
            double dx = pv.x() - mt[i].x, dy = pv.y() - mt[i].y;
            rmse += dx * dx + dy * dy;
        }
        rmse = std::sqrt(rmse / static_cast<double>(n));

        if (std::abs(prev_rmse - rmse) < icp_convergence_) {
            result.converged = true;
            result.rmse = rmse;
            break;
        }
        prev_rmse = rmse;
        result.rmse = rmse;
    }

    // 从原始 src 到最终 aligned 的总变换（通过 SVD 重建）
    Eigen::Vector2d src_c = Centroid2D(src);
    Eigen::Vector2d aligned_c = Centroid2D(aligned);
    Eigen::Matrix2d R_total = SvdRotation2D(CovarianceH2D(src, aligned, src_c, aligned_c));
    Eigen::Vector2d t_total = aligned_c - R_total * src_c;

    result.rotation = std::atan2(R_total(1, 0), R_total(0, 0));
    result.tx = t_total.x();
    result.ty = t_total.y();

    return result;
}

std::vector<Point2D> IcpApfPlanner::TransformPoints(const std::vector<Point2D>& points, double rotation, double tx,
                                                    double ty) {
    std::vector<Point2D> result;
    result.reserve(points.size());
    const double c = std::cos(rotation);
    const double s = std::sin(rotation);
    std::ranges::transform(points, std::back_inserter(result), [c, s, tx, ty](const Point2D& p) {
        return Point2D(c * p.x - s * p.y + tx, s * p.x + c * p.y + ty);
    });
    return result;
}

Point2D IcpApfPlanner::ComputeApfForce(const Point2D& p, const std::vector<Point2D>& centerline,
                                       const std::vector<common_msgs::msg::HuatCone>& left,
                                       const std::vector<common_msgs::msg::HuatCone>& right) {
    Point2D force;

    // 1. 朝向中心线的吸引力：使用 std::ranges::min_element 配合距离投影
    if (!centerline.empty()) {
        auto nearest_it = std::ranges::min_element(centerline, {}, [&](const Point2D& c) {
            const double dx = p.x - c.x;
            const double dy = p.y - c.y;
            return dx * dx + dy * dy;
        });
        const double dx = p.x - nearest_it->x;
        const double dy = p.y - nearest_it->y;
        const double min_dist = std::sqrt(dx * dx + dy * dy);
        if (min_dist > 1e-6) {
            force.x += -k_a_ * dx;
            force.y += -k_a_ * dy;
        }
    }

    // 2. 来自边界的排斥力（距离截断至 0.1m，防止近距离排斥力爆炸）
    constexpr double kMinRepulsionDist = 0.1;
    auto ComputeRepulsion = [&](const std::vector<common_msgs::msg::HuatCone>& cones) {
        for (const auto& c : cones) {
            double dx = p.x - c.position_base_link.x;
            double dy = p.y - c.position_base_link.y;
            double dist = std::max(std::sqrt(dx * dx + dy * dy), kMinRepulsionDist);
            if (dist < d_0_) {
                double factor = k_r_ * (1.0 / dist - 1.0 / d_0_) / (dist * dist);
                force.x += factor * dx;
                force.y += factor * dy;
            }
        }
    };
    ComputeRepulsion(left);
    ComputeRepulsion(right);

    return force;
}

std::vector<Point2D> IcpApfPlanner::ApplyApfCorrection(const std::vector<Point2D>& path,
                                                       const std::vector<Point2D>& centerline,
                                                       const std::vector<common_msgs::msg::HuatCone>& left,
                                                       const std::vector<common_msgs::msg::HuatCone>& right) {
    std::vector<Point2D> corrected = path;
    for (auto& p : corrected) {
        for (int iter = 0; iter < apf_max_iter_; ++iter) {
            Point2D f = ComputeApfForce(p, centerline, left, right);
            double step = std::sqrt(f.x * f.x + f.y * f.y);
            if (step < 1e-6)
                break;
            if (step > apf_step_limit_) {
                double scale = apf_step_limit_ / step;
                f.x *= scale;
                f.y *= scale;
            }
            p.x += f.x;
            p.y += f.y;
        }
    }
    return corrected;
}

std::vector<Point2D> IcpApfPlanner::AlignPath(const std::vector<Point2D>& centerline,
                                              const std::vector<Point2D>& prev_path, double icp_rmse_limit) {
    static rclcpp::Clock throttle_clock(RCL_SYSTEM_TIME);
    if (prev_path.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("skidpad_planner"), "[skidpad_planner] No previous path, using raw centerline");
        return centerline;
    }
    if (centerline.size() < 3) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("skidpad_planner"), throttle_clock, 1000,
                             "[skidpad_planner] centerline_pts=%zu < 3, applying APF on prev_path",
                             centerline.size());
        return prev_path;
    }
    IcpResult icp = IcpAlign(centerline, prev_path);
    RCLCPP_INFO(rclcpp::get_logger("skidpad_planner"),
                "[skidpad_planner] ICP: converged=%d, rmse=%.4f, rot=%.3f, tx=%.3f, ty=%.3f", icp.converged, icp.rmse,
                icp.rotation, icp.tx, icp.ty);
    if (icp.rmse <= icp_rmse_limit) {
        return TransformPoints(centerline, icp.rotation, icp.tx, icp.ty);
    }
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("skidpad_planner"), throttle_clock, 1000,
                         "[skidpad_planner] ICP rmse %.3f > limit %.3f, using raw centerline", icp.rmse,
                         icp_rmse_limit);
    return centerline;
}

std::vector<Point2D> IcpApfPlanner::GeneratePath(const std::vector<common_msgs::msg::HuatCone>& cones,
                                                 const std::vector<Point2D>& prev_path, double icp_rmse_limit) {
    static rclcpp::Clock throttle_clock(RCL_SYSTEM_TIME);
    std::vector<common_msgs::msg::HuatCone> left, right;
    ClusterCones(cones, left, right);

    if (left.empty() || right.empty()) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("skidpad_planner"), throttle_clock, 1000,
                             "[skidpad_planner] Missing left or right cones, cannot generate path");
        return prev_path;
    }

    const bool use_circle = (left.size() >= 3 || right.size() >= 3);
    std::vector<Point2D> centerline;
    if (use_circle) {
        centerline = ComputeCircleCenterline(left, right);
    }
    if (centerline.empty()) {
        if (use_circle)
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("skidpad_planner"), throttle_clock, 1000,
                                 "[skidpad_planner] Circle fit failed/invalid, falling back to midpoint method");
        centerline = ComputeCenterline(left, right);
    }
    if (centerline.empty()) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("skidpad_planner"), throttle_clock, 1000,
                             "[skidpad_planner] Failed to compute centerline");
        return prev_path;
    }

    RCLCPP_INFO(rclcpp::get_logger("skidpad_planner"),
                "[skidpad_planner] Cones: total=%zu, left=%zu, right=%zu, centerline_pts=%zu (method=%s)", cones.size(),
                left.size(), right.size(), centerline.size(), use_circle ? "circle" : "midpoint");

    return ApplyApfCorrection(AlignPath(centerline, prev_path, icp_rmse_limit), centerline, left, right);
}

}  // namespace skidpad
