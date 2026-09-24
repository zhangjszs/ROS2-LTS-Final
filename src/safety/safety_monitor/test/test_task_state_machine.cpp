// issue #16：统一任务×安全状态机 + 故障注入回归（纯 std，无 ROS context）。
#include <gtest/gtest.h>

#include "task_state_machine.h"  // 仓库约定：common_msgs 手写头不带前缀

using common_msgs::vehicle::SafetyState;
using common_msgs::vehicle::StopEdge;
using common_msgs::vehicle::StopKind;
using common_msgs::vehicle::TaskSafetyStateMachine;
using common_msgs::vehicle::TaskState;

TEST(TaskSafetyFSM, InitialStateIdleNormal) {
    TaskSafetyStateMachine sm;
    EXPECT_EQ(sm.task(), TaskState::IDLE);
    EXPECT_EQ(sm.safety(), SafetyState::NORMAL);
    EXPECT_FALSE(sm.stopActive());
    EXPECT_FALSE(sm.canDrive());  // IDLE 未武装不可行驶
}

TEST(TaskSafetyFSM, ArmThenStartAllowsDriving) {
    TaskSafetyStateMachine sm;
    EXPECT_TRUE(sm.onArm().changed);
    EXPECT_EQ(sm.task(), TaskState::ARMED);
    EXPECT_FALSE(sm.canDrive());  // 仅 ARMED，未 START
    EXPECT_TRUE(sm.onStart().changed);
    EXPECT_EQ(sm.task(), TaskState::RUNNING);
    EXPECT_TRUE(sm.canDrive());
}

TEST(TaskSafetyFSM, TimeoutStopIsRecoverableAndPublishesOnce) {
    TaskSafetyStateMachine sm;
    sm.onArm();
    sm.onStart();
    auto e = sm.onTimeoutStop();
    EXPECT_TRUE(e.changed);
    EXPECT_EQ(e.stop_edge, StopEdge::PUBLISH);
    EXPECT_TRUE(sm.stopActive());
    EXPECT_FALSE(sm.canDrive());

    auto again = sm.onTimeoutStop();
    EXPECT_FALSE(again.changed);  // 幂等：已在停不再产生边沿
    EXPECT_EQ(again.stop_edge, StopEdge::NONE);

    auto r = sm.onResume();
    EXPECT_TRUE(r.changed);
    EXPECT_EQ(r.stop_edge, StopEdge::CLEAR);
    EXPECT_FALSE(sm.stopActive());
    EXPECT_TRUE(sm.canDrive());  // 回到 RUNNING 可行驶
}

TEST(TaskSafetyFSM, RequestStopIsLatchedNotClearedByResume) {
    TaskSafetyStateMachine sm;
    sm.onArm();
    sm.onStart();
    sm.onStopRequest();
    EXPECT_EQ(sm.stopKind(), StopKind::REQUEST);
    auto r = sm.onResume();
    EXPECT_FALSE(r.changed);  // 锁存：路径恢复不得解除 REQUEST 停
    EXPECT_TRUE(sm.stopActive());
    auto reset = sm.onReset();
    EXPECT_TRUE(reset.changed);
    EXPECT_EQ(reset.stop_edge, StopEdge::CLEAR);
    EXPECT_FALSE(sm.stopActive());
}

TEST(TaskSafetyFSM, LatchedStopBeatsTimeoutPriority) {
    TaskSafetyStateMachine sm;
    sm.onStopRequest();           // 锁存停
    auto t = sm.onTimeoutStop();  // 后续超时不得覆盖锁存来源/不得新发边沿
    EXPECT_FALSE(t.changed);
    EXPECT_EQ(sm.stopKind(), StopKind::REQUEST);
}

TEST(TaskSafetyFSM, FaultIsStickyAndForbidsArmUntilReset) {
    TaskSafetyStateMachine sm;
    auto f = sm.onFault();
    EXPECT_EQ(sm.task(), TaskState::FAULT);
    EXPECT_EQ(sm.stopKind(), StopKind::FAULT);
    EXPECT_TRUE(sm.stopActive());
    EXPECT_EQ(f.stop_edge, StopEdge::PUBLISH);
    EXPECT_FALSE(sm.onArm().changed);  // 故障锁存期禁止武装
    auto r = sm.onResume();
    EXPECT_FALSE(r.changed);  // 故障非 TIMEOUT，不可恢复
    EXPECT_TRUE(sm.onReset().changed);
    EXPECT_EQ(sm.task(), TaskState::IDLE);
    EXPECT_FALSE(sm.stopActive());
    EXPECT_TRUE(sm.onArm().changed);  // 复位后可重新武装
}

TEST(TaskSafetyFSM, FinishTriggersLatchedStopAndResetToIdle) {
    TaskSafetyStateMachine sm;
    sm.onArm();
    sm.onStart();
    auto f = sm.onFinish();
    EXPECT_EQ(sm.task(), TaskState::FINISHED);
    EXPECT_TRUE(sm.stopActive());
    EXPECT_EQ(f.stop_edge, StopEdge::PUBLISH);
    EXPECT_FALSE(sm.canDrive());
    EXPECT_TRUE(sm.onReset().changed);
    EXPECT_EQ(sm.task(), TaskState::IDLE);
}

TEST(TaskSafetyFSM, StartWhileStoppedRejected) {
    TaskSafetyStateMachine sm;
    sm.onStopRequest();
    EXPECT_FALSE(sm.onStart().changed);
    EXPECT_NE(sm.task(), TaskState::RUNNING);
}
