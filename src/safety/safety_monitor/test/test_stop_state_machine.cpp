#include <gtest/gtest.h>

#include "safety_monitor/stop_state_machine.h"

// ── Initial state ─────────────────────────────────────────────────────────

TEST(StopStateMachineTest, InitialStateIsIdle) {
    StopStateMachine fsm;
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
}

// ── Timeout flow: IDLE → TIMEOUT_STOP → IDLE ──────────────────────────────

TEST(StopStateMachineTest, TimeoutFromIdlePublishesStop) {
    StopStateMachine fsm;
    auto result = fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::TIMEOUT);
    EXPECT_EQ(result.action, StopAction::PUBLISH_STOP);
    EXPECT_EQ(result.reason, StopReason::TIMEOUT);
}

TEST(StopStateMachineTest, PathReceivedClearsTimeoutStop) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
    EXPECT_EQ(result.action, StopAction::CLEAR_STOP);
    EXPECT_EQ(result.reason, StopReason::NONE);
}

// ── Request flow: IDLE → REQUEST_STOP → IDLE ──────────────────────────────

TEST(StopStateMachineTest, StopRequestFromIdlePublishesStop) {
    StopStateMachine fsm;
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::REQUEST);
    EXPECT_EQ(result.action, StopAction::PUBLISH_STOP);
    EXPECT_EQ(result.reason, StopReason::REQUEST);
}

TEST(StopStateMachineTest, ManualResetClearsRequestStop) {
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
    EXPECT_EQ(result.action, StopAction::CLEAR_STOP);
    EXPECT_EQ(result.reason, StopReason::NONE);
}

// ── Guard: no-op on invalid transitions ────────────────────────────────────

TEST(StopStateMachineTest, TimeoutWhileAlreadyInTimeoutIsNoOp) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, StopRequestWhileAlreadyInRequestIsNoOp) {
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, PathReceivedWhileIdleIsNoOp) {
    StopStateMachine fsm;
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, ManualResetWhileIdleIsNoOp) {
    StopStateMachine fsm;
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Guard: manual reset only works from REQUEST_STOP ───────────────────────

TEST(StopStateMachineTest, ManualResetFromTimeoutStopIsNoOp) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Recovery: path received during REQUEST_STOP does NOT clear ─────────────

TEST(StopStateMachineTest, PathReceivedDuringRequestStopIsNoOp) {
    // REQUEST_STOP requires explicit manual reset, path alone won't clear it
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, StopRequestDuringTimeoutPromotesToRequestStop) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::REQUEST);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, PathReceivedAfterTimeoutThenRequestDoesNotResume) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    fsm.onStopRequested();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Full cycle: timeout → recover → request → reset ────────────────────────

TEST(StopStateMachineTest, FullCycle) {
    StopStateMachine fsm;
    // timeout triggers stop
    fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    // planner recovers
    fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    // race finish triggers request stop
    fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    // manual reset clears
    fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
