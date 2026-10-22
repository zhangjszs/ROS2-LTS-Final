#include "icp_apf_planner.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <rclcpp/rclcpp.hpp>

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

}  // namespace

namespace skidpad {

IcpApfPlanner::IcpApfPlanner(const IcpApfConfig& cfg) {
    const auto sanitized = SanitizeSkidpadConfig(cfg);
    const auto& c = sanitized.cfg;
    center_margin_ = c.center_margin;
    icp_max_iter_ = c.icp_max_iter;
    icp_convergence_ = c.icp_convergence;
    k_a_ = c.k_a;
    k_r_ = c.k_r;
    d_0_ = c.d_0;
    path_spacing_ = c.path_spacing;
    path_lookahead_ = c.path_lookahead;
    apf_max_iter_ = c.apf_max_iter;
    apf_step_limit_ = c.apf_step_limit;
    icp_max_match_dist_ = c.icp_max_match_dist;
}

void IcpApfPlanner::ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                                 std::vector<common_msgs::msg::HuatCone>& left,
                                 std::vector<common_msgs::msg::HuatCone>& right) {
    left.clear();
    right.clear();
    // #89: 参数兜底防除零与 NaN 比较恒 false 导致清空点云；
    // 注：历史坐标约定中 left 为 y < -margin，right 为 y > margin（与 ROS +y 左 / -y 右相反，保留历史约定）
    const double margin = (std::isfinite(center_margin_) && center_margin_ >= 0.0) ? center_margin_ : 0.3;
    std::ranges::copy_if(
        cones, std::back_inserter(left), [margin](float y) { return y < -margin; },
        [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::copy_if(
        cones, std::back_inserter(right), [margin](float y) { return y > margin; },
        [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
}

std::vector<Point2D> IcpApfPlanner::ComputeCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                                      const std::vector<common_msgs::msg::HuatCone>& right) {
    if (left.empty() || right.empty())
        return {};

    std::vector<Point2D> left_pts;
    left_pts.reserve(left.size());
    for (const auto& c : left) {
        left_pts.emplace_back(c.position_base_link.x, c.position_base_link.y);
    }

    std::vector<Point2D> right_pts;
    right_pts.reserve(right.size());
    for (const auto& c : right) {
        right_pts.emplace_back(c.position_base_link.x, c.position_base_link.y);
    }

    constexpr double kMaxPairDist = 6.0;
    const CenterlineParams params{
        .max_pair_dist = kMaxPairDist,
        .path_spacing = path_spacing_,
        .path_lookahead = path_lookahead_,
    };

    return ComputeCenterlineCore(left_pts, right_pts, params);
}

Circle2D IcpApfPlanner::FitCircleTaubin(const std::vector<common_msgs::msg::HuatCone>& cones) {
    std::vector<Point2D> pts;
    pts.reserve(cones.size());
    for (const auto& c : cones) {
        pts.emplace_back(c.position_base_link.x, c.position_base_link.y);
    }
    return FitCircleTaubinCore(pts);
}

std::vector<Point2D> IcpApfPlanner::ComputeCircleCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                                            const std::vector<common_msgs::msg::HuatCone>& right) {
    std::vector<Point2D> left_pts;
    left_pts.reserve(left.size());
    for (const auto& c : left) {
        left_pts.emplace_back(c.position_base_link.x, c.position_base_link.y);
    }
    std::vector<Point2D> right_pts;
    right_pts.reserve(right.size());
    for (const auto& c : right) {
        right_pts.emplace_back(c.position_base_link.x, c.position_base_link.y);
    }

    const CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = path_spacing_,
        .path_lookahead = path_lookahead_,
    };
    return ComputeCircleCenterlineCore(left_pts, right_pts, params);
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
    const double ka = (std::isfinite(k_a_) && k_a_ >= 0.0) ? k_a_ : 0.5;
    const double kr = (std::isfinite(k_r_) && k_r_ >= 0.0) ? k_r_ : 2.0;
    const double d0 = (std::isfinite(d_0_) && d_0_ > 0.0) ? d_0_ : 2.0;
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
            force.x += -ka * dx;
            force.y += -ka * dy;
        }
    }

    // 2. 来自边界的排斥力（距离截断至 0.1m，防止近距离排斥力爆炸）
    constexpr double kMinRepulsionDist = 0.1;
    auto ComputeRepulsion = [&](const std::vector<common_msgs::msg::HuatCone>& cones) {
        for (const auto& c : cones) {
            double dx = p.x - c.position_base_link.x;
            double dy = p.y - c.position_base_link.y;
            double dist = std::max(std::sqrt(dx * dx + dy * dy), kMinRepulsionDist);
            if (dist < d0) {
                double factor = kr * (1.0 / dist - 1.0 / d0) / (dist * dist);
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
        const double step_limit = (std::isfinite(apf_step_limit_) && apf_step_limit_ > 0.0) ? apf_step_limit_ : 0.2;
        const int max_iter = (apf_max_iter_ > 0) ? apf_max_iter_ : 10;
        for (int iter = 0; iter < max_iter; ++iter) {
            Point2D f = ComputeApfForce(p, centerline, left, right);
            double step = std::sqrt(f.x * f.x + f.y * f.y);
            if (step < 1e-6)
                break;
            if (step > step_limit) {
                double scale = step_limit / step;
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
                             "[skidpad_planner] centerline_pts=%zu < 3, applying APF on prev_path", centerline.size());
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
