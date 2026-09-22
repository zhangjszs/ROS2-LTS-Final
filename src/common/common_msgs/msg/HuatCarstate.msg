std_msgs/Header header                                                    # stamp = 发布时刻（非设备测量时刻，设备时间语义标定前不得混淆，见 issue #12）

common_msgs/Pose2D car_state    			# Position in x, y, theta

float32 v

# —— 定位质量/设备时间透传（issue #12）：均为设备原始值，解释以真实设备协议为准；
#    估计器未填充时保持零值，消费端若需区分"未知"请与发布链约定哨兵（如 ins_status=0 表示从未透传） ——
int32 ins_status                     # 透传自 HuatASENSING.ins_status（0 = 未透传）
int32 position_type                  # 透传自 HuatASENSING.position_type（0 = 未透传）
float64 device_gps_week_number       # 透传自 HuatASENSING.gps_week_number（<=0 = 未知）
float64 device_sec_of_week             # 透传自 HuatASENSING.sec_of_week（<0 = 未知）
bool quality_validated                 # 质量字段是否已由可信标定/映射判定（未标定的发布链保持 false）
