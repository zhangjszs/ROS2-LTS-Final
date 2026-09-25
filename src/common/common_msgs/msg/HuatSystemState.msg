# issue #16：系统级“任务 × 安全”状态 + 最终指令仲裁遥测。
# 由 command_arbiter_node 按控制率发布在契约话题 /system/state（见 docs/INTERFACE_CONTRACT.md §7）。
# 用途：故障注入验收要求“每个故障都有可重复的预期终态、结果可自动判断，不能只凭 RViz 或日志观察”，
#       本消息就是那份机读证据；数值常量与 common_msgs 纯 std 层的枚举顺序**一致**，
#       由 command_arbiter_node.cpp 的 static_assert 在编译期锁定，防止漂移。
std_msgs/Header header

# —— 任务态（TaskState）——
uint8 TASK_IDLE=0
uint8 TASK_ARMED=1
uint8 TASK_RUNNING=2
uint8 TASK_FINISHED=3
uint8 TASK_FAULT=4
uint8 task_state

# —— 安全态（SafetyState）——
uint8 SAFETY_NORMAL=0
uint8 SAFETY_DEGRADED=1
uint8 SAFETY_STOP=2
uint8 safety_state

# 停车触发来源（StopKind）
uint8 STOPKIND_NONE=0
uint8 STOPKIND_TIMEOUT=1
uint8 STOPKIND_REQUEST=2
uint8 STOPKIND_FAULT=3
uint8 stop_kind

bool stop_active   # 锁存/超时停车是否有效（#8）
bool can_drive     # 仅 RUNNING 且非停态为真；ARMED 仍输出安全制动

# —— 仲裁结果 ——
uint8 SRC_NONE=0
uint8 SRC_PURE_PURSUIT=1
uint8 SRC_MPC=2
uint8 winner_source

uint8 REASON_CONTROL=0
uint8 REASON_STOP_ACTIVE=1
uint8 REASON_TASK_NOT_DRIVING=2
uint8 REASON_NO_TRUSTED_SOURCE=3
uint8 reason

bool safe_fallback           # 本帧是否为确定性安全降级输出
uint8 committed_source       # 去抖后已接管（committed）的源（SRC_*）

# —— 各控制源的信任状态（Untrusted；age_sec <0 = 从未收到）——
uint8 UNTRUSTED_NONE=0
uint8 UNTRUSTED_ABSENT=1
uint8 UNTRUSTED_BAD_FRAME=2
uint8 UNTRUSTED_BAD_CHECKSUM=3
uint8 UNTRUSTED_STALE=4
uint8 source_a_status
uint8 source_b_status
float64 source_a_age_sec
float64 source_b_age_sec
uint64 source_a_rejected     # 累计拒收次数（帧头/校验和/超龄）
uint64 source_b_rejected

# —— 最终出口指令字节（与底盘实际收到的帧对账，#15 帧契约）——
uint8 steering
uint8 brake_force
uint8 pedal_ratio
uint16 checksum

# 本帧所依据的执行器标定版本（#15）：使“这一帧是用哪份标定做出来的”可追溯
string calibration_version
