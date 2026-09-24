// issue #22：pp_core::ThrottleController 单元测试 —— 从 Pure Pursuit 节点抽出的纵向控制律。
// 纯 std（无 ROS），验证与原节点内联实现逐条件等价：积分冻结/抗饱和/低高速定值/中段钳制/限幅/reset。
#include <gtest/gtest.h>

#include "pure_pursuit/throttle_controller.h"

using pp_core::ThrottleController;
using pp_core::ThrottleParamsLike;

namespace {
// 满足 concept 的最小参数结构（字段名与 PurePursuitParams.algorithm.throttle 一致）。
struct Params {
    double target_speed = 10.0;
    double pid_kp = 1.0;
    double pid_ki = 0.0;
    double pid_integral_max = 100.0;
    double speed_low_threshold = 1.0;
    double current_low_speed = 30.0;
    double speed_high_threshold = 20.0;
    double current_high_speed = 15.0;
    double current_clamp_max = 1000.0;
    double pedal_min = 0.0;
    double pedal_max = 100.0;
    double speed_blend_zone = 0.0;  // 默认不过渡，便于确定性断言
};

static_assert(ThrottleParamsLike<Params>, "Params must satisfy ThrottleParamsLike");
}  // namespace

TEST(ThrottleController, ProportionalOnlyInMidBand) {
    ThrottleController ctrl;
    Params p;  // target 10, kp 1, ki 0, v=5 ∈ (1,20)
    EXPECT_EQ(ctrl.update(5.0, p), 5);
    EXPECT_DOUBLE_EQ(ctrl.integral(), 5.0);  // 未冻结→积分累计 err=5（但 ki=0 不影响本帧输出）
}

TEST(ThrottleController, LowSpeedFixedCurrentAndIntegralFrozen) {
    ThrottleController ctrl;
    Params p;
    p.pid_ki = 1.0;
    // v=0 <= lo(1) => 冻结积分；输出走 current_low_speed=30
    EXPECT_EQ(ctrl.update(0.0, p), 30);
    EXPECT_DOUBLE_EQ(ctrl.integral(), 0.0);  // 冻结：积分保持 0
}

TEST(ThrottleController, HighSpeedFixedCurrent) {
    ThrottleController ctrl;
    Params p;
    // v=25 >= hi(20) => 冻结积分；输出 current_high_speed=15
    EXPECT_EQ(ctrl.update(25.0, p), 15);
    EXPECT_DOUBLE_EQ(ctrl.integral(), 0.0);
}

TEST(ThrottleController, IntegralAccumulatesAndAntiWindupClamps) {
    ThrottleController ctrl;
    Params p;
    p.target_speed = 25.0;
    p.pid_kp = 0.0;
    p.pid_ki = 1.0;
    p.pid_integral_max = 12.0;
    // v=10 ∈ (1,20) 不冻结，每帧 err=15 累加并 clamp 到 ±12
    ctrl.update(10.0, p);  // sum=15 => clamp 12
    EXPECT_DOUBLE_EQ(ctrl.integral(), 12.0);
    ctrl.update(10.0, p);  // 再加 15 => 27 => 仍 clamp 12
    EXPECT_DOUBLE_EQ(ctrl.integral(), 12.0);
}

TEST(ThrottleController, MidBandClampMaxApplies) {
    ThrottleController ctrl;
    Params p;
    p.target_speed = 100.0;  // 大误差
    p.current_clamp_max = 40.0;
    // v=10 ∈ (1,20)：long_current=kp*(90)=90，中段 > clamp_max => 40
    EXPECT_EQ(ctrl.update(10.0, p), 40);
}

TEST(ThrottleController, PedalMinAndMaxClamp) {
    ThrottleController ctrl;
    Params hi;
    hi.pedal_max = 20.0;  // current_low_speed 30 超上限 => 20
    EXPECT_EQ(ctrl.update(0.0, hi), 20);
    Params lo;
    lo.target_speed = -5.0;         // 负误差
    lo.pedal_min = 5.0;             // 下限抬到 5
    lo.current_clamp_max = 1000.0;  // v=15 中段，long_current=kp*(-20)=-20 => int -20 => clamp min 5
    EXPECT_EQ(ctrl.update(15.0, lo), 5);
}

TEST(ThrottleController, LowBlendInterpolates) {
    ThrottleController ctrl;
    Params p;
    p.pid_kp = 1.0;
    p.pid_ki = 0.0;
    p.speed_blend_zone = 1.0;  // lo=1, lo_lo=0, 过渡带 [0,1)
    // v=0.5：过渡带中点 t=(0.5-0)/1=0.5；纯 P 项=kp*(10-0.5)=9.5；blend=0.5*30+0.5*9.5=19.75 => int 19
    EXPECT_EQ(ctrl.update(0.5, p), 19);
}

TEST(ThrottleController, ResetClearsIntegral) {
    ThrottleController ctrl;
    Params p;
    p.pid_ki = 1.0;
    ctrl.update(5.0, p);  // 中段 v=5、target=10 → err=5、ki=1 累计非零积分
    EXPECT_GT(ctrl.integral(), 0.0);
    ctrl.reset();
    EXPECT_DOUBLE_EQ(ctrl.integral(), 0.0);
}
