# issue #50：控制器求解健康度出口（"持续拒解"从日志提升为机读状态）。
# 由控制器节点按控制率发布在 /controller/<name>/health（非契约话题；契约五话题见
# docs/INTERFACE_CONTRACT.md）。与 /system/state 的分工：那里表达"输入超时/外部停车"，
# 这里表达"控制器自己解不出来"（RejectWatchdog 语义，mpc_controller/controller_health.hpp）。
std_msgs/Header header

string controller            # 控制器名（"mpc" ...）
bool rejecting               # 连续拒解达到阈值（一次成功求解即解除）
uint64 consecutive_rejects   # 当前连续拒解拍数
uint64 total_rejects         # 自启动累计拒解拍数
uint64 total_accepts         # 自启动累计成功求解拍数（负样本判据：默认参数必须真的在解）
uint64 threshold             # 生效阈值（0 = 看门狗禁用）
