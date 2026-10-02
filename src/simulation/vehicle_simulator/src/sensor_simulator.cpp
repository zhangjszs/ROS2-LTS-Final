#include "vehicle_simulator/sensor_simulator.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace simulation {

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
    // #54：几何全部交给纯 std core（PredictVisibleCones），本层只做中性类型 → ROS 消息组装。
    common_msgs::msg::HuatMap map_msg;
    for (const auto& d : PredictVisibleCones(global_cones_, state, fov_deg, max_range, noise_stddev, rng_)) {
        common_msgs::msg::HuatCone detected;
        detected.id = d.id;
        detected.type = d.type;
        detected.confidence = d.confidence;

        detected.position_base_link.x = d.x_base;
        detected.position_base_link.y = d.y_base;
        detected.position_base_link.z = 0.0f;

        detected.position_global.x = d.x_global;
        detected.position_global.y = d.y_global;
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
