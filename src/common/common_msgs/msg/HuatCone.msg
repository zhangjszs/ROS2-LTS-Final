std_msgs/Header header
geometry_msgs/Point32  position_base_link  # coordinate of cone in [x, y]                      圆锥的相对车体坐标
geometry_msgs/Point32  position_global  # coordinate of cone in [x, y]                        圆锥的相对全局坐标  参考起点
uint32 id           # 在整个运行过程中不可更改的唯一id
uint32 confidence     #置信度
uint32 type                  # color: 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
                             # Same enum as HuatVisionDetections.color_types. Not lidar size.