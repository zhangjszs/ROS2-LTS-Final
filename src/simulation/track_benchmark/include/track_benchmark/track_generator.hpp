#pragma once

#include <cone_types.h>

#include <cmath>
#include <cstdint>
#include <optional>
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
 * @brief #90：八字绕环（skidpad）完赛判据元数据。
 *
 * 完赛 = 进场 → 第一圆（y>0）绕 required_laps 圈 → 换圆 → 第二圆（y<0）反向绕
 * required_laps 圈 → 出场。数值由 GenerateSkidpad() 随几何一同填充（唯一事实
 * 来源），消费方（skidpad_finish_judge.hpp、ROS 闭环门禁）不得硬编码 9.125 / 2。
 */
struct SkidpadCircuitMetadata {
    double right_center_x{0.0};    // 第一绕行圆（y>0）圆心 X (m)
    double right_center_y{9.125};  // 第一绕行圆圆心 Y (m)
    double left_center_x{0.0};     // 第二绕行圆（y<0）圆心 X (m)
    double left_center_y{-9.125};  // 第二绕行圆圆心 Y (m)
    double path_radius{9.125};     // 中心线绕圈半径 (m)
    int required_laps{2};          // 每圆要求圈数（FSAC 规则 = 2）
    double exit_x{0.0};            // 出场通道起点 X (m)：越过即视为驶向出场区
};

/**
 * @brief 完整赛道定义 (包含边界锥桶与中心参考线)
 *
 * #17：`id` + `version` 是全仓唯一的赛道标识：仿真器读的 CSV、评测器生成的几何、
 * 回归结果里的 track_version 必须都来自同一个 (id, version)，避免“名字相同、几何不同”。
 */
struct TrackDefinition {
    std::string name{"unnamed"};
    // 稳定标识（如 "acceleration-75m"）：选择赛道用它，不用显示名。
    std::string id{"unnamed"};
    // 几何版本：任何改变锥桶/中心线几何的修改必须递增（回归基线据此判口径）。
    std::string version{"v1"};
    // 是否闭合赛道（计圈/越界判据依赖它）。由生成器指定，消费者不得再靠“名字是不是 acceleration”猜。
    bool closed_circuit{true};
    std::vector<BenchmarkCone> cones;
    std::vector<CenterlinePoint> centerline;
    double track_width{3.0};
    double total_length{0.0};

    // #81：终点/缓冲区元数据（机读）。完赛判定的消费方（离线 runner、ROS 闭环门禁）
    // 必须从这里取值，不得再硬编码 75/100，也不得从 RED 锥桶反推终点位置。
    // 闭合赛道没有「终点线」语义 → 保持空值（hasFinish() 为 false）。
    std::optional<double> finish_x;      // 终点线 X 坐标 (m)
    std::optional<double> runout_end_m;  // 终点后停车缓冲区末端 X 坐标 (m)

    // #90：八字绕环完赛元数据（机读）。skidpad 的完赛语义是「进场 → 右圆两圈 →
    // 换圆 → 左圆两圈 → 出场」，圆心/半径/圈数只能从这里取，消费方不得硬编码
    // 9.125 / 2。非 skidpad 赛道保持空值（hasSkidpad() 为 false）。
    std::optional<SkidpadCircuitMetadata> skidpad;

    // 写入报告/回归基线的版本串（单一构造点）。
    [[nodiscard]] std::string versionedId() const { return id + "/" + version; }

    // #81：终点语义是否完整（终点线与缓冲区末端同时存在）。消费者据此决定
    // 「本赛道能否做直线赛完赛判定」，而不是各自去猜字段是否存在。
    [[nodiscard]] bool hasFinish() const noexcept { return finish_x.has_value() && runout_end_m.has_value(); }

    // #90：八字绕环完赛语义是否完整（元数据随几何生成）。
    [[nodiscard]] bool hasSkidpad() const noexcept { return skidpad.has_value(); }
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
     * @brief 保存赛道锥桶到 CSV（格式: x,y,type；固定 3 位小数）
     *
     * 文件头部额外写一行 `# id=<id> version=<v> cones=<n> geometry_fnv1a64=<hex>`：
     * 每份几何自带身份，仿真器读到的就是“哪一个版本的哪条赛道”。
     * 固定精度 + 确定行序是为了让“生成器输出”能与仓内已提交 CSV 逐字节 diff：
     * 仿真器与评测共用同一份几何，漂移会被回归门禁（benchmark_regression.sh）直接查出。
     */
    static bool SaveConesToCSV(const TrackDefinition& track, const std::string& path);

    /**
     * @brief 赛道清单文件 (tracks.json)：id/version/文件名/锥桶数/几何校验和。
     *        返回是否写入成功；校验和用 fnv1a64，使“同一赛道”有可机读的等同判据。
     */
    static bool SaveTrackManifest(const std::vector<TrackDefinition>& tracks, const std::string& dir,
                                  const std::vector<std::string>& filenames);

    /// 非加密校验和（FNV-1a 64bit）：只用于检测几何漂移，不做安全用途。
    [[nodiscard]] static std::uint64_t geometryChecksum(const std::vector<BenchmarkCone>& cones) noexcept;

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
