#include <gtest/gtest.h>

#include "safety_monitor/input_freshness.h"

TEST(InputFreshness, OneVehicleSampleWithContinuousTenHzPathMustExpire) {
    safety_monitor::InputFreshness inputs(0.3, 0.5);
    inputs.observeVehicle(true, 100.0, 100.0);
    inputs.observePath(true, 100.0, 100.0);
    ASSERT_TRUE(inputs.healthy(100.0, 100.0));
    for (int tick = 1; tick <= 10; ++tick) {
        const double now = 100.0 + tick * 0.1;
        inputs.observePath(true, now, now);
        if (tick >= 4)
            EXPECT_FALSE(inputs.healthy(now, now)) << "vehicle last rx=100, path rx=" << now;
    }
}

#include <array>
#include <limits>

#include "command_arbitration.h"
#include "safety_monitor/stop_state_machine.h"
#include "task_state_machine.h"

using common_msgs::vehicle::Freshness;
using safety_monitor::InputFreshness;

TEST(InputFreshness, AbsentSourcesNeverAuthorizeAndStateDoesNotSeedPlanner) {
    InputFreshness inputs;
    EXPECT_EQ(inputs.check(10, 10).vehicle, Freshness::kAbsent);
    inputs.observePath(true, 10, 10);
    EXPECT_FALSE(inputs.healthy(10, 10));
    InputFreshness state_only;
    state_only.observeVehicle(true, 10, 10);
    EXPECT_EQ(state_only.check(10, 10).planner, Freshness::kAbsent);
    EXPECT_FALSE(state_only.healthy(10, 10));
}

TEST(InputFreshness, IndependentConfiguredDeadlinesIncludingBoundary) {
    InputFreshness inputs(0.25, 0.5);
    inputs.observeVehicle(true, 10, 10);
    inputs.observePath(true, 10, 10);
    EXPECT_TRUE(inputs.healthy(10.25, 10.25));
    EXPECT_EQ(inputs.check(10.26, 10.26).vehicle, Freshness::kStale);
    EXPECT_EQ(inputs.check(10.26, 10.26).planner, Freshness::kFresh);
    inputs.observeVehicle(true, 10.5, 10.5);
    EXPECT_TRUE(inputs.healthy(10.5, 10.5));
    EXPECT_EQ(inputs.check(10.51, 10.51).planner, Freshness::kStale);
}

TEST(InputFreshness, BothSourcesExpire) {
    InputFreshness inputs;
    inputs.observeVehicle(true, 10, 10);
    inputs.observePath(true, 10, 10);
    const auto health = inputs.check(11, 11);
    EXPECT_EQ(health.vehicle, Freshness::kStale);
    EXPECT_EQ(health.planner, Freshness::kStale);
    EXPECT_FALSE(health.healthy());
}

TEST(InputFreshness, InvalidVehicleAndInvalidPathDoNotRenewEitherLease) {
    InputFreshness inputs;
    inputs.observeVehicle(true, 10, 10);
    inputs.observePath(true, 10, 10);
    for (int i = 1; i <= 10; ++i) {
        const double t = 10 + i * 0.1;
        inputs.observeVehicle(false, t, t);
        inputs.observePath(false, t, t);
    }
    EXPECT_DOUBLE_EQ(inputs.check(11, 11).vehicle_age, 1);
    EXPECT_DOUBLE_EQ(inputs.check(11, 11).planner_age, 1);
    EXPECT_FALSE(inputs.healthy(11, 11));
}

TEST(InputFreshness, RecoveryRequiresBothSourcesInEitherOrder) {
    for (bool path_first : {true, false}) {
        InputFreshness inputs;
        inputs.observeVehicle(true, 10, 10);
        inputs.observePath(true, 10, 10);
        EXPECT_FALSE(inputs.healthy(11, 11));
        if (path_first)
            inputs.observePath(true, 11, 11);
        else
            inputs.observeVehicle(true, 11, 11);
        EXPECT_FALSE(inputs.healthy(11, 11));
        if (path_first)
            inputs.observeVehicle(true, 11.1, 11.1);
        else
            inputs.observePath(true, 11.1, 11.1);
        EXPECT_TRUE(inputs.healthy(11.1, 11.1));
    }
}

TEST(InputFreshness, PausedRosClockCannotFreezeReceiveTimeout) {
    InputFreshness inputs;
    inputs.observeVehicle(true, 10, 100);
    inputs.observePath(true, 10, 100);
    EXPECT_FALSE(inputs.healthy(11, 100));
}

TEST(InputFreshness, RosRollbackInvalidatesBothUntilNewEpochSamplesArrive) {
    InputFreshness inputs;
    inputs.observeVehicle(true, 10, 100);
    inputs.observePath(true, 10, 100);
    inputs.observePath(true, 10.1, 50);
    EXPECT_EQ(inputs.check(10.1, 50).vehicle, Freshness::kAbsent);
    EXPECT_FALSE(inputs.healthy(10.1, 50));
    inputs.observeVehicle(true, 10.2, 50.1);
    EXPECT_TRUE(inputs.healthy(10.2, 50.1));
    EXPECT_FALSE(inputs.healthy(10.3, 40));
}

TEST(InputFreshness, FutureReceiveEpochCannotReviveOldLeaseAfterClockCatchesUp) {
    InputFreshness inputs;
    inputs.observeVehicle(true, 10, 100);
    inputs.observePath(true, 10, 100);
    EXPECT_FALSE(inputs.healthy(9, 100));
    EXPECT_FALSE(inputs.healthy(10.1, 100));
    common_msgs::vehicle::FreshnessLease lease;
    lease.observe(10);
    EXPECT_EQ(lease.check(9), Freshness::kFromFuture);
}

TEST(InputFreshness, NonFiniteClockFailsClosedAndRequiresBothAgain) {
    for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        InputFreshness inputs;
        inputs.observeVehicle(true, 10, 100);
        inputs.observePath(true, 10, 100);
        EXPECT_FALSE(inputs.healthy(bad, 100));
        EXPECT_FALSE(inputs.healthy(10.1, 100));
        inputs.observeVehicle(true, 10.1, 100);
        inputs.observePath(true, 10.1, 100);
        EXPECT_FALSE(inputs.healthy(10.2, bad));
    }
}

TEST(InputFreshness, InvalidBudgetsAreRejectedRatherThanDisableSafety) {
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        EXPECT_THROW((InputFreshness{bad, 0.5}), std::invalid_argument);
        EXPECT_THROW((InputFreshness{0.3, bad}), std::invalid_argument);
    }
}

// 组合生产的租约、停车状态机、任务状态机和仲裁器；不替代 ROS 适配层/DDS 测试。
class InputSafetyChain : public ::testing::Test {
   protected:
    InputFreshness inputs;
    StopStateMachine monitor;
    common_msgs::vehicle::TaskSafetyStateMachine task;

    InputSafetyChain() { monitor.onTimeoutExpired(); }
    void tick(double t) {
        if (inputs.healthy(t, t))
            monitor.onPathReceived();
        else
            monitor.onTimeoutExpired();
        if (monitor.state() == StopState::IDLE)
            task.onResume();
        else
            task.onTimeoutStop();
    }
    void healthy(double t) {
        inputs.observeVehicle(true, t, t);
        inputs.observePath(true, t, t);
        tick(t);
    }
    common_msgs::vehicle::VehicleCommandRaw output() {
        using namespace common_msgs::vehicle;
        SourceCandidate command;
        command.present = true;
        command.source = ControlSource::MPC;
        command.cmd.pedal_ratio = 30;
        command.cmd.gear_position = 1;
        command.cmd.working_mode = 1;
        command.age_sec = 0;
        command.checksum = checksumRaw(command.cmd);
        const std::array commands{command};
        return CommandArbitrator{ActuatorCalibration{}, 90}
            .decide(task.stopActive(), task.canDrive(), commands, ArbitrationConfig{})
            .cmd;
    }
    void expectBrake() {
        EXPECT_EQ(output().pedal_ratio, 0);
        EXPECT_GT(output().brake_force, 0);
    }
};

TEST_F(InputSafetyChain, NoArmOrNoStartWithValidThrottleStillBrakes) {
    healthy(10);
    expectBrake();
    task.onStart();
    expectBrake();
    task.onArm();
    expectBrake();
    task.onStart();
    EXPECT_EQ(output().pedal_ratio, 30);
}

TEST_F(InputSafetyChain, StateLossStopsDespiteHealthyPlannerAndTrustedMpc) {
    healthy(10);
    task.onArm();
    task.onStart();
    ASSERT_EQ(output().pedal_ratio, 30);
    for (int i = 1; i <= 10; ++i) {
        double t = 10 + i * 0.1;
        inputs.observePath(true, t, t);
        tick(t);
        if (i >= 4) {
            EXPECT_EQ(monitor.state(), StopState::TIMEOUT_STOP);
            expectBrake();
        }
    }
}

TEST_F(InputSafetyChain, RecoveryNeverClearsRequestOrFault) {
    healthy(10);
    monitor.onStopRequested();
    task.onStopRequest();
    healthy(11);
    EXPECT_EQ(monitor.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(task.stopKind(), common_msgs::vehicle::StopKind::REQUEST);
    expectBrake();
    task.onFault();
    healthy(12);
    EXPECT_EQ(task.stopKind(), common_msgs::vehicle::StopKind::FAULT);
    expectBrake();
}

TEST_F(InputSafetyChain, MonitorRestartBeginsTimeoutAndArbiterRestartNeedsAuthorization) {
    healthy(10);
    task.onArm();
    task.onStart();
    ASSERT_EQ(output().pedal_ratio, 30);
    inputs = InputFreshness{};
    monitor = StopStateMachine{};
    monitor.onTimeoutExpired();
    tick(10.1);
    expectBrake();
    inputs.observePath(true, 10.2, 10.2);
    tick(10.2);
    expectBrake();
    task = common_msgs::vehicle::TaskSafetyStateMachine{};
    healthy(10.3);
    EXPECT_EQ(task.task(), common_msgs::vehicle::TaskState::IDLE);
    expectBrake();
}

TEST(InputFreshness, NonFiniteVehicleFieldsCannotRefreshLease) {
    for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
        for (int field = 0; field < 4; ++field) {
            std::array<double, 4> values{1, 2, 0, 3};
            values[field] = bad;
            const bool valid = safety_monitor::validVehicleState(values[0], values[1], values[2], values[3]);
            ASSERT_FALSE(valid);
            InputFreshness inputs;
            inputs.observeVehicle(valid, 10, 10);
            inputs.observePath(true, 10, 10);
            EXPECT_EQ(inputs.check(10, 10).vehicle, Freshness::kAbsent);
            EXPECT_FALSE(inputs.healthy(10, 10));
        }
    }
}
