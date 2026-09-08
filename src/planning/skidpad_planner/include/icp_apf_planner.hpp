#pragma once

#include <algorithm>
#include <common_msgs/msg/huat_cone.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace skidpad {

struct Circle2D {
    double cx, cy, r;
    bool valid;
    Circle2D() : cx(0.0), cy(0.0), r(0.0), valid(false) {}
};

struct Point2D {
    double x;
    double y;
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

struct IcpResult {
    double rotation;  // 旋转角度，单位为弧度
    double tx;
    double ty;
    double rmse;
    bool converged;
    IcpResult() : rotation(0.0), tx(0.0), ty(0.0), rmse(0.0), converged(false) {}
};

struct IcpApfConfig {
    double center_margin = 0.3;
    int icp_max_iter = 20;
    double icp_convergence = 0.001;
    double k_a = 0.5;
    double k_r = 2.0;
    double d_0 = 2.0;
    double path_spacing = 0.5;
    double path_lookahead = 20.0;
    int apf_max_iter = 10;
    double apf_step_limit = 0.2;
    double icp_max_match_dist = 3.0;
};

class IcpApfPlanner {
   public:
    explicit IcpApfPlanner(const IcpApfConfig& cfg);

    // 根据当前锥桶生成路径
    // 如果 prev_path 非空，则使用 ICP 将当前中心线对齐到 prev_path
    // icp_rmse_limit: ICP rmse 超过此值时拒绝对齐，回退到原始中心线
    std::vector<Point2D> GeneratePath(const std::vector<common_msgs::msg::HuatCone>& cones,
                                      const std::vector<Point2D>& prev_path, double icp_rmse_limit = 0.5);

    void ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                      std::vector<common_msgs::msg::HuatCone>& left, std::vector<common_msgs::msg::HuatCone>& right);

    std::vector<Point2D> TransformPoints(const std::vector<Point2D>& points, double rotation, double tx, double ty);

   private:
    std::vector<Point2D> ComputeCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                           const std::vector<common_msgs::msg::HuatCone>& right);

    // Taubin 圆拟合：从锥桶点集估计圆心和半径
    static Circle2D FitCircleTaubin(const std::vector<common_msgs::msg::HuatCone>& cones);

    // 基于圆拟合的中心线生成（skidpad 场景优化）
    std::vector<Point2D> ComputeCircleCenterline(const std::vector<common_msgs::msg::HuatCone>& left,
                                                 const std::vector<common_msgs::msg::HuatCone>& right);

    // ICP：将源点云 src 对齐到目标点云 tgt
    IcpResult IcpAlign(const std::vector<Point2D>& src, const std::vector<Point2D>& tgt);

    // APF：计算位置 p 处的受力
    Point2D ComputeApfForce(const Point2D& p, const std::vector<Point2D>& centerline,
                            const std::vector<common_msgs::msg::HuatCone>& left,
                            const std::vector<common_msgs::msg::HuatCone>& right);

    // 将 centerline 与历史路径 ICP 对齐（或在无历史/不足时直接返回）
    std::vector<Point2D> AlignPath(const std::vector<Point2D>& centerline, const std::vector<Point2D>& prev_path,
                                   double icp_rmse_limit);

    // 对路径点应用 APF 修正
    std::vector<Point2D> ApplyApfCorrection(const std::vector<Point2D>& path, const std::vector<Point2D>& centerline,
                                            const std::vector<common_msgs::msg::HuatCone>& left,
                                            const std::vector<common_msgs::msg::HuatCone>& right);

    double center_margin_;
    int icp_max_iter_;
    double icp_convergence_;
    double k_a_;
    double k_r_;
    double d_0_;
    double path_spacing_;
    double path_lookahead_;
    int apf_max_iter_;
    double apf_step_limit_;
    double icp_max_match_dist_;
};

}  // namespace skidpad
