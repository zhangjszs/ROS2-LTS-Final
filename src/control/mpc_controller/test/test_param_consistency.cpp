// #45（#19 协议 P0 前置）：车辆参数一致性自检。
//
// 协议要求：「新增回归断言：MPC config 派生值 == simulator YAML 真值；不一致即红」。
// B0（0682075）只是把四处数值改到一致，**没有任何机检**守着它们——谁改一处 YAML 或
// 头文件默认值，F1/F2 就会静默复活，而四道门禁全是 PP 链路，不会报警。本文件补这道机检。
//
// 四处真值：
//   ① src/simulation/vehicle_simulator/config/simulator_params.yaml   （唯一真值源）
//   ② src/simulation/vehicle_simulator/include/vehicle_simulator/bicycle_model.hpp（被控对象默认值）
//   ③ src/control/mpc_controller/config/mpc_params.yaml               （节点运行时参数）
//   ④ src/control/mpc_controller/include/mpc_controller/mpc_params.hpp（MpcConfig 编译期默认值）
//
// 只做文本扫描 + std 解析：不引入 pyyaml，也不给 control 包加 simulation 依赖（方向会反）。
// 仓库根由构建系统以 FSAC_REPO_ROOT 宏注入；未注入时不编译报错（对 IDE 友好），
// 而是让文件路径拼不出来→第一个用例失败，在 CI/本地门禁上同样是红（不会默声通过）。
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "mpc_controller/mpc_params.hpp"

#ifndef FSAC_REPO_ROOT
#define FSAC_REPO_ROOT ""  // 故意置空：TruthSourcesAreReachable 会因此判失败
#endif

namespace {

constexpr double kTol = 1e-9;

/// 从 YAML/C++ 头文本里取 `key: value` 或 `key{value}` 的数值。
/// 两种形态的分隔符长度不同（':' 与 '{' 都在 key 末尾后的同一位置），
/// 所以统一取“分隔符之后”为起点，再交给 istringstream 跳过空白。
/// 行尾注释：YAML 用 '#'，C++ 头用 '//'，两者都要剪掉。
[[nodiscard]] std::optional<double> ScanNumber(const std::string& path, const std::string& key) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return std::nullopt;
    }
    std::string line;
    while (std::getline(in, line)) {
        std::size_t vpos = std::string::npos;
        if (const auto c = line.find(key + ":"); c != std::string::npos) {
            vpos = c + key.size() + 1;  // ':' 之后
        } else if (const auto b = line.find(key + "{"); b != std::string::npos) {
            vpos = b + key.size() + 1;  // '{' 之后
        } else {
            continue;
        }
        std::string rest = line.substr(vpos);
        for (const char* marker : {"#", "//"}) {
            if (const auto p = rest.find(marker); p != std::string::npos) {
                rest = rest.substr(0, p);
            }
        }
        std::istringstream ss(rest);
        double v = 0.0;
        if (ss >> v) {
            return v;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::string RepoFile(const char* rel) {
    return std::string(FSAC_REPO_ROOT) + "/" + rel;
}

/// 从节点源文件里取 declare_parameter<...>("<param>", <value>) 的字面默认值（#50）。
[[nodiscard]] std::optional<double> ScanDeclareParameter(const std::string& path, const std::string& param) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return std::nullopt;
    }
    std::string line;
    while (std::getline(in, line)) {
        const auto c = line.find("\"" + param + "\"");
        if (c == std::string::npos || line.find("declare_parameter") == std::string::npos) {
            continue;
        }
        const auto comma = line.find(',', c + param.size() + 2);
        if (comma == std::string::npos) {
            continue;
        }
        std::istringstream ss(line.substr(comma + 1));
        double v = 0.0;
        if (ss >> v) {
            return v;
        }
    }
    return std::nullopt;
}

constexpr const char* kSimulatorYaml = "src/simulation/vehicle_simulator/config/simulator_params.yaml";
constexpr const char* kBicycleHeader = "src/simulation/vehicle_simulator/include/vehicle_simulator/bicycle_model.hpp";
constexpr const char* kMpcYaml = "src/control/mpc_controller/config/mpc_params.yaml";
// #50：新暴露的 QP 节点参数的默认值必须等于 qp_solver.hpp 现值（"看起来可配其实已漂移"不算零漂移）。
constexpr const char* kMpcNodeCpp = "src/control/mpc_controller/src/mpc_controller_node.cpp";
constexpr const char* kQpSolverHpp = "src/control/mpc_controller/include/mpc_controller/qp_solver.hpp";

}  // namespace

// 文件必须存在：路径写错就等于门没关过（比"读到空值然后放行"安全得多）。
// 本用例同时兼职"FSAC_REPO_ROOT 已正确注入"的哨兵。
TEST(ParamConsistency, TruthSourcesAreReachable) {
    for (const char* rel : {kSimulatorYaml, kBicycleHeader, kMpcYaml}) {
        std::ifstream in(RepoFile(rel));
        EXPECT_TRUE(in.is_open()) << "缺真值文件：" << rel;
    }
}

// F1：轴距与最大转角在四处真值上必须逐值一致（simulator YAML 为唯一真值源）。
TEST(ParamConsistency, VehicleGeometryMatchesSimulatorTruth) {
    const auto sim_wb = ScanNumber(RepoFile(kSimulatorYaml), "wheelbase");
    const auto model_wb = ScanNumber(RepoFile(kBicycleHeader), "wheelbase");
    const auto mpc_yaml_wb = ScanNumber(RepoFile(kMpcYaml), "wheelbase");

    ASSERT_TRUE(sim_wb.has_value()) << "simulator_params.yaml 里找不到 wheelbase";
    ASSERT_TRUE(model_wb.has_value()) << "bicycle_model.hpp 里找不到 wheelbase 默认值";
    ASSERT_TRUE(mpc_yaml_wb.has_value()) << "mpc_params.yaml 里找不到 wheelbase";

    // ④ 编译期默认值（无 YAML 加载时的回退面）
    const mpc::MpcConfig cfg;
    ASSERT_NEAR(*sim_wb, *model_wb, kTol) << "被控对象头文件默认值已偏离唯一真值源";
    ASSERT_NEAR(*sim_wb, *mpc_yaml_wb, kTol) << "MPC YAML 轴距偏离唯一真值源（F1 复活）";
    ASSERT_NEAR(*sim_wb, cfg.system.wheelbase, kTol) << "MpcConfig 编译期默认轴距偏离唯一真值源";

    const auto sim_steer = ScanNumber(RepoFile(kSimulatorYaml), "max_steer_angle");
    const auto model_steer = ScanNumber(RepoFile(kBicycleHeader), "max_steer_angle");
    const auto mpc_yaml_steer = ScanNumber(RepoFile(kMpcYaml), "max_steer_rad");
    ASSERT_TRUE(sim_steer.has_value() && model_steer.has_value() && mpc_yaml_steer.has_value());
    ASSERT_NEAR(*sim_steer, *model_steer, kTol) << "被控对象最大转角偏离唯一真值源";
    ASSERT_NEAR(*sim_steer, *mpc_yaml_steer, kTol) << "MPC 预测模型最大转角偏离唯一真值源（F1 复活）";
    ASSERT_NEAR(*sim_steer, cfg.limits.max_steer_rad, kTol) << "MpcConfig 默认最大转角偏离唯一真值源";

    // 转角速率：仿真器侧只有头文件里一个来源（YAML 未列），三方一致即可
    const auto model_steer_rate = ScanNumber(RepoFile(kBicycleHeader), "max_steer_rate");
    const auto mpc_yaml_rate = ScanNumber(RepoFile(kMpcYaml), "max_steer_rate");
    ASSERT_TRUE(model_steer_rate.has_value() && mpc_yaml_rate.has_value());
    ASSERT_NEAR(*model_steer_rate, *mpc_yaml_rate, kTol) << "转角速率限幅两处不一致";
    ASSERT_NEAR(*model_steer_rate, cfg.limits.max_steer_rate, kTol) << "MpcConfig 默认转角速率不一致";
}

// F2：预测步长必须等于控制周期（sample_time == 1/control_rate），否则"定时器与预测时域
// 自相矛盾"这个原始缺陷会静默复活；同时守住 MpcConfig 默认值这一回退面。
TEST(ParamConsistency, SampleTimeMatchesControlRate) {
    const auto rate = ScanNumber(RepoFile(kMpcYaml), "control_rate");
    const auto ts = ScanNumber(RepoFile(kMpcYaml), "sample_time");
    ASSERT_TRUE(rate.has_value() && ts.has_value()) << "mpc_params.yaml 缺 control_rate 或 sample_time";
    ASSERT_GT(*rate, 0.0);
    ASSERT_NEAR(*ts * *rate, 1.0, 1e-9) << "sample_time 与 1/control_rate 不一致（F2 复活）：ts=" << *ts
                                        << " rate=" << *rate;

    const mpc::MpcConfig cfg;
    ASSERT_NEAR(cfg.horizon.Ts * cfg.system.control_rate, 1.0, 1e-9)
        << "MpcConfig 默认值面同样失配：Ts=" << cfg.horizon.Ts << " rate=" << cfg.system.control_rate;
}

// 时域物理长度守恒（D9 的换算不能被人改步长后悄悄失效）：
// Np·Ts ≈ 0.76s、Nc·Ts ≈ 0.50s，且 YAML 与编译期默认值一致。
TEST(ParamConsistency, HorizonPhysicalLengthIsConserved) {
    const auto np = ScanNumber(RepoFile(kMpcYaml), "horizon_steps");
    const auto nc = ScanNumber(RepoFile(kMpcYaml), "control_horizon");
    const auto ts = ScanNumber(RepoFile(kMpcYaml), "sample_time");
    ASSERT_TRUE(np.has_value() && nc.has_value() && ts.has_value());
    const mpc::MpcConfig cfg;

    ASSERT_NEAR(*np, static_cast<double>(cfg.horizon.Np), kTol) << "YAML 预测步数与默认值不一致";
    ASSERT_NEAR(*nc, static_cast<double>(cfg.horizon.Nc), kTol) << "YAML 控制步数与默认值不一致";
    ASSERT_NEAR(*ts, cfg.horizon.Ts, kTol) << "YAML 步长与默认值不一致";

    EXPECT_NEAR(*np * *ts, 0.76, 5e-3) << "预测时域物理长度偏离 D9 的 0.76s 换算";
    EXPECT_NEAR(*nc * *ts, 0.50, 5e-3) << "控制时域物理长度偏离 D9 的 0.50s 换算";
    EXPECT_LE(cfg.horizon.Nc, cfg.horizon.Np) << "控制时域不得长于预测时域";
}

// #50：节点 QP 旋钮默认值 ↔ qp_solver.hpp 现值逐字段对账。
// 背景：#50 把 eps_abs/eps_rel/rho/max_iter 暴露为节点参数（故障注入冒烟需要）。
// 若声明的默认值与结构体现值不同，"不调参时零漂移"就是假话，且 #48 预检的
// "参考配置=代码默认值"链条在 ROS 面断裂——所以这是一致性事实，不是风格问题。
TEST(ParamConsistency, QpParamDefaultsMatchSolverHeader) {
    struct Pair {
        const char* param;
        const char* field;
    };
    for (const Pair& p : {Pair{"mpc.qp_eps_abs", "eps_abs"}, Pair{"mpc.qp_eps_rel", "eps_rel"},
                          Pair{"mpc.qp_rho", "rho"}, Pair{"mpc.qp_max_iter", "max_iter"}}) {
        const auto decl = ScanDeclareParameter(RepoFile(kMpcNodeCpp), p.param);
        ASSERT_TRUE(decl.has_value()) << "节点源里找不到 " << p.param << " 的 declare_parameter 默认值";
        const auto cur = ScanNumber(RepoFile(kQpSolverHpp), p.field);
        ASSERT_TRUE(cur.has_value()) << "qp_solver.hpp 里找不到 " << p.field << " 默认值";
        ASSERT_NEAR(*decl, *cur, kTol) << p.param << " 的节点默认值已偏离 qp_solver.hpp 现值（零漂移是假话）";
    }
}
