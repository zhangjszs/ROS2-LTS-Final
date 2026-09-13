#pragma once

#include <string>

namespace velocity_profiler {

struct ProfilerLimits {
    double max_velocity{20.0};          // 最高允许车速 (m/s)
    double min_velocity{2.0};           // 最低过弯/巡航车速 (m/s)
    double max_lat_accel{9.8};          // 轮胎最大侧向加速度 (m/s^2)
    double max_lon_accel{3.5};          // 最大牵引纵向加速度 (m/s^2)
    double max_lon_decel{5.0};          // 最大制动纵向减速度 (m/s^2)
    int curvature_smoothing_window{5};  // 曲率滑动平均滤波窗口
    bool enable_friction_circle{true};  // 是否启用纵横向摩擦椭圆耦合
};

struct ProfilerTopics {
    std::string input_path{"/planning/raw_pathlimits"};
    std::string output_path{"/planning/pathlimits"};
    std::string vehicle_state{"/localization/vehicle_state"};
    std::string speed_markers{"/planning/viz/speed_markers"};
};

struct ProfilerConfig {
    ProfilerLimits limits;
    ProfilerTopics topics;
};

struct ProfilePoint {
    double x{0.0};
    double y{0.0};
    double s{0.0};             // 沿线累计弧长 (m)
    double theta{0.0};         // 航向角 (rad)
    double curvature{0.0};     // 曲率 kappa (1/m)
    double v_corner{0.0};      // 侧向附着极限车速 (m/s)
    double v_back{0.0};        // 后向制动规划车速 (m/s)
    double v_fwd{0.0};         // 前向牵引规划车速 (m/s)
    double target_speed{0.0};  // 最终最优目标车速 v*(s) (m/s)
};

}  // namespace velocity_profiler
