#include "vehicle_simulator/sensor_simulator.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <sstream>

namespace simulation {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kDegToRad = kPi / 180.0;

}  // namespace

bool SensorSimulator::LoadTrackFromCSV(const std::string& csv_path) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "[SensorSimulator] Failed to open track file: " << csv_path << "\n";
        return false;
    }

    global_cones_.clear();
    std::string line;
    uint32_t current_id = 1;

    while (std::getline(file, line)) {
        // 跳过空行与注释行
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string sx, sy, stype;
        if (std::getline(ss, sx, ',') && std::getline(ss, sy, ',') && std::getline(ss, stype, ',')) {
            try {
                double x = std::stod(sx);
                double y = std::stod(sy);
                uint32_t type = static_cast<uint32_t>(std::stoul(stype));
                global_cones_.push_back(TrackCone{.x = x, .y = y, .type = type, .id = current_id++});
            } catch (const std::exception& e) {
                // 跳过表头等非数值行
                continue;
            }
        }
    }

    std::cout << "[SensorSimulator] Loaded " << global_cones_.size() << " cones from " << csv_path << "\n";
    return !global_cones_.empty();
}

common_msgs::msg::HuatMap SensorSimulator::GeneratePerceivedCones(const VehicleState& state, double fov_deg,
                                                                  double max_range, double noise_stddev) {
    common_msgs::msg::HuatMap map_msg;
    const double half_fov_rad = (fov_deg * 0.5) * kDegToRad;
    const double max_range_sq = max_range * max_range;
    std::normal_distribution<double> dist(0.0, (noise_stddev > 0.0) ? noise_stddev : 1.0);

    const double cos_th = std::cos(state.theta);
    const double sin_th = std::sin(state.theta);

    for (const auto& cone : global_cones_) {
        // 1. 全局坐标平移到车辆质心
        const double dx = cone.x - state.x;
        const double dy = cone.y - state.y;
        const double dist_sq = dx * dx + dy * dy;

        // 距离粗筛
        if (dist_sq > max_range_sq || dist_sq < 0.25)
            continue;

        // 2. 旋转到车体坐标系 (base_link: X 朝前, Y 朝左)
        const double x_base = dx * cos_th + dy * sin_th;
        const double y_base = -dx * sin_th + dy * cos_th;

        // 必须在车体前方
        if (x_base <= 0.2)
            continue;

        // 3. 水平视场角 (FOV) 判定
        const double angle = std::atan2(y_base, x_base);
        if (std::abs(angle) > half_fov_rad)
            continue;

        // 4. 生成探测结果并加入测距高斯噪声
        common_msgs::msg::HuatCone detected;
        detected.id = cone.id;
        detected.type = cone.type;
        detected.confidence = 95;

        double n_x = (noise_stddev > 0.0) ? dist(rng_) : 0.0;
        double n_y = (noise_stddev > 0.0) ? dist(rng_) : 0.0;

        detected.position_base_link.x = static_cast<float>(x_base + n_x);
        detected.position_base_link.y = static_cast<float>(y_base + n_y);
        detected.position_base_link.z = 0.0f;

        detected.position_global.x = static_cast<float>(cone.x + n_x);
        detected.position_global.y = static_cast<float>(cone.y + n_y);
        detected.position_global.z = 0.0f;

        map_msg.cone.push_back(detected);
    }

    return map_msg;
}

common_msgs::msg::HuatMap SensorSimulator::GetGlobalGroundTruthMap() const {
    common_msgs::msg::HuatMap map_msg;
    for (const auto& cone : global_cones_) {
        common_msgs::msg::HuatCone c;
        c.id = cone.id;
        c.type = cone.type;
        c.confidence = 100;
        c.position_global.x = static_cast<float>(cone.x);
        c.position_global.y = static_cast<float>(cone.y);
        c.position_global.z = 0.0f;
        map_msg.cone.push_back(c);
    }
    return map_msg;
}

}  // namespace simulation
