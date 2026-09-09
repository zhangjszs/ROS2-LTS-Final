#pragma once

#include <cone_types.h>

#include <cmath>
#include <string>
#include <vector>

namespace benchmark {

/**
 * @brief 赛道锥桶
 */
struct BenchmarkCone {
    double x{0.0};
    double y{0.0};
    uint32_t type{huat_cone::BLUE};  // 0: Blue, 1: Yellow, 2: YellowBig, 3: Red
    uint32_t id{0};
};

/**
 * @brief 赛道理想中心轨迹航路点 (Ground Truth Centerline Waypoint)
 */
struct CenterlinePoint {
    double x{0.0};          // X 坐标 (m)
    double y{0.0};          // Y 坐标 (m)
    double theta{0.0};      // 航向角 (rad)
    double curvature{0.0};  // 曲率 kappa = 1 / R (1/m)
    double s{0.0};          // 累计里程弧长 (m)
};

/**
 * @brief 完整赛道定义 (包含边界锥桶与中心参考线)
 */
struct TrackDefinition {
    std::string name{"unnamed"};
    std::vector<BenchmarkCone> cones;
    std::vector<CenterlinePoint> centerline;
    double track_width{3.0};
    double total_length{0.0};
};

/**
 * @brief 赛事标准赛道程序化生成器
 */
class TrackGenerator {
   public:
    TrackGenerator() = default;

    /**
     * @brief 生成 FSAC 规则标准八字绕环赛道 (Skidpad)
     */
    [[nodiscard]] static TrackDefinition GenerateSkidpad();

    /**
     * @brief 生成 FSAC 规则 75m 直线加速赛道 (Acceleration)
     */
    [[nodiscard]] static TrackDefinition GenerateAcceleration(double length = 75.0, double track_width = 3.0);

    /**
     * @brief 生成参数化闭合操控赛道 (Trackdrive Loop)
     */
    [[nodiscard]] static TrackDefinition GenerateTrackdriveLoop(double rx = 35.0, double ry = 20.0,
                                                                double track_width = 3.0, size_t num_points = 250);

    /**
     * @brief 保存赛道锥桶到 CSV (格式: x,y,type)
     */
    static bool SaveConesToCSV(const std::vector<BenchmarkCone>& cones, const std::string& path);

    /**
     * @brief 保存中心参考线到 CSV (格式: x,y,theta,curvature,s)
     */
    static bool SaveCenterlineToCSV(const std::vector<CenterlinePoint>& centerline, const std::string& path);

    /**
     * @brief 从 CSV 读取中心参考线
     */
    static std::vector<CenterlinePoint> LoadCenterlineFromCSV(const std::string& path);
};

}  // namespace benchmark
