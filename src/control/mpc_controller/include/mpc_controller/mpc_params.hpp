#pragma once

#include <cstddef>
#include <string>

namespace mpc {

// 车辆几何/执行器默认值真值源: src/simulation/vehicle_simulator/config/simulator_params.yaml
// (#19 B0) 未加载 mpc_params.yaml 时的回退默认值，须与 YAML 逐值一致。
struct SystemParams {
    double control_rate{50.0};
    double startup_delay{0.0};
    int racing_num{1};
    double wheelbase{1.55};
};

struct MpcWeights {
    double q_lat{50.0};      // 横向跟踪误差权重 e_y
    double q_heading{25.0};  // 航向角误差权重 e_psi
    double q_speed{5.0};     // 速度跟踪误差权重 (v - v_ref)
    double r_steer{1.0};     // 前轮转角绝对值权重
    double r_accel{0.5};     // 加速度绝对值权重
    double rd_steer{8.0};    // 前轮转角变化率权重 (平滑防抖)
    double rd_accel{2.0};    // 加速度变化率权重
};

struct MpcLimits {
    // max_steer_rad: 最大转角 (rad)，= 仿真器 max_steer_angle 真值 (#19 B0)
    double max_steer_rad{0.40};
    double max_steer_rate{3.0};  // 最大角速度 (rad/s)
    double min_accel{-4.0};      // 最大制动减速度 (m/s^2)
    double max_accel{3.0};       // 最大驱动加速度 (m/s^2)
};

struct MpcHorizon {
    size_t Np{38};             // 预测时域长度 (38 x 0.02s ≈ 0.76s, #19 B0 等时长换算)
    size_t Nc{25};             // 控制时域长度 (25 x 0.02s = 0.50s)
    double Ts{0.02};           // 控制步长 (s), 与 1/control_rate 一致
    double target_speed{8.0};  // 巡航目标车速 (m/s)
};

struct MpcTopics {
    std::string vehicle_state{"/localization/vehicle_state"};
    std::string path{"/planning/pathlimits"};
    std::string stop{"/system/stop"};
    std::string vehicle_command{"/vehicle_command"};
    std::string predicted_path{"/control/mpc_predicted_path"};
    std::string reference_path{"/control/mpc_reference_path"};
};

struct MpcConfig {
    SystemParams system;
    MpcWeights weights;
    MpcLimits limits;
    MpcHorizon horizon;
    MpcTopics topics;
};

}  // namespace mpc
