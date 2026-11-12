#pragma once

#include <algorithm>
#include <common_msgs/msg/huat_cone.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

#include "centerline_core.hpp"
#include "corridor_model_core.hpp"

namespace skidpad {

struct IcpResult {
    double rotation;  // 旋转角度，单位为弧度
    double tx;
    double ty;
    double rmse;
    bool converged;
    IcpResult() : rotation(0.0), tx(0.0), ty(0.0), rmse(0.0), converged(false) {}
};

class IcpApfPlanner {
   public:
    explicit IcpApfPlanner(const IcpApfConfig& cfg);

    // 根据当前锥桶生成路径
    // 如果 prev_path 非空，则使用 ICP 将当前中心线对齐到 prev_path
    // icp_rmse_limit: ICP rmse 超过此值时拒绝对齐，回退到原始中心线
    //
    // #101：走廊模型的**持久状态**（走廊不会突然改变，模型应当跨帧保持）：
    //   - trusted_model（入参）：上一帧已验证的走廊模型（base_link 坐标）。
    //     本帧壁证据变差时用它从当前位姿重新生成弧；
    //   - fresh_trusted_model（出参）：本帧**新得到**的可信走廊模型
    //     （base_link 坐标）；本帧没拿到新模型时保持调用方给的值不变
    //     （调用方据此判断「继续沿用自己持有的那一个」）。
    std::vector<Point2D> GeneratePath(const std::vector<common_msgs::msg::HuatCone>& cones,
                                      const std::vector<Point2D>& prev_path, double icp_rmse_limit = 0.5,
                                      const CorridorModel* trusted_model = nullptr,
                                      CorridorModel* fresh_trusted_model = nullptr, const LobePrior* prior = nullptr);

    void ClusterCones(const std::vector<common_msgs::msg::HuatCone>& cones,
                      std::vector<common_msgs::msg::HuatCone>& left, std::vector<common_msgs::msg::HuatCone>& right);

    std::vector<Point2D> TransformPoints(const std::vector<Point2D>& points, double rotation, double tx, double ty);

   private:
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

    // #95：走廊中线路径生成（相位感知单圆 → 中点法回退 → 锚到自车并等距重采样）。
    // 自车在 base_link 系的原点。
    CorridorPathResult ComputeCorridorPath(const std::vector<common_msgs::msg::HuatCone>& left,
                                           const std::vector<common_msgs::msg::HuatCone>& right,
                                           const std::vector<Point2D>& prev_path, const CorridorModel* trusted_model,
                                           const LobePrior* prior = nullptr);

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
