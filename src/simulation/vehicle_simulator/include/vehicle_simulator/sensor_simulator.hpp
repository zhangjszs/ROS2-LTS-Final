#pragma once

#include <cone_types.h>

#include <common_msgs/msg/huat_cone.hpp>
#include <common_msgs/msg/huat_map.hpp>
#include <random>
#include <string>
#include <vector>

#include "vehicle_simulator/bicycle_model.hpp"

namespace simulation {

/**
 * @brief 赛道锥桶定义
 */
struct TrackCone {
    double x{0.0};
    double y{0.0};
    uint32_t type{huat_cone::BLUE};  // huat_cone::Color
    uint32_t id{0};
};

/**
 * @brief 传感器模拟器 (LiDAR & Camera 感知模拟)
 */
class SensorSimulator {
   public:
    SensorSimulator() = default;

    /**
     * @brief 从 CSV 文件加载赛道锥桶地图 (格式: x, y, type)
     */
    bool LoadTrackFromCSV(const std::string& csv_path);

    /**
     * @brief 设置全局锥桶列表
     */
    void SetTrackCones(std::vector<TrackCone> cones) { global_cones_ = std::move(cones); }

    /**
     * @brief 根据车辆当前位姿动态生成车载感知到的锥桶地图
     * @param state 车辆当前状态
     * @param fov_deg 水平视场角 (度)
     * @param max_range 最大探测距离 (米)
     * @param noise_stddev 测距高斯噪声标准差 (米)
     */
    [[nodiscard]] common_msgs::msg::HuatMap GeneratePerceivedCones(const VehicleState& state, double fov_deg = 120.0,
                                                                   double max_range = 15.0, double noise_stddev = 0.0);

    /**
     * @brief 获取全局全量赛道地图 (用于真值可视化或全局规划器)
     */
    [[nodiscard]] common_msgs::msg::HuatMap GetGlobalGroundTruthMap() const;

    [[nodiscard]] const std::vector<TrackCone>& global_cones() const noexcept { return global_cones_; }
    [[nodiscard]] size_t cone_count() const noexcept { return global_cones_.size(); }

   private:
    std::vector<TrackCone> global_cones_;
    std::mt19937 rng_{42};
};

}  // namespace simulation
