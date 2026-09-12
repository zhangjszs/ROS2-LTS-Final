#pragma once

#include <cstddef>
#include <string>

namespace mpc {

struct SystemParams {
    double control_rate{50.0};
    double startup_delay{0.0};
    int racing_num{1};
    double wheelbase{1.53};
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
    double max_steer_rad{0.436332};  // 最大转角 (25度)
    double max_steer_rate{3.0};      // 最大角速度 (rad/s)
    double min_accel{-4.0};          // 最大制动减速度 (m/s^2)
    double max_accel{3.0};           // 最大驱动加速度 (m/s^2)
};

struct MpcHorizon {
    size_t Np{15};             // 预测时域长度
    size_t Nc{10};             // 控制时域长度
    double Ts{0.05};           // 控制步长 (s)
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
