std_msgs/Header header
geometry_msgs/Point[] path                                                                     #路径点上的数组
float64[] target_speeds
			   # 与 path 等长时表示显式有效的目标速度 (m/s)，允许 0 表示停车
common_msgs/HuatTracklimits tracklimits                                        #赛道限制信息
bool replan                                                                                                          #

