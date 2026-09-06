std_msgs/Header header
geometry_msgs/Point32[] points  # coordinate of cone in [x, y]
geometry_msgs/Point32[] max_points  # coordinate of cone in [x, y]
geometry_msgs/Point32[] min_points  # coordinate of cone in [x, y]

float32[] confidence
float32[] obj_dist
std_msgs/String color           # optional per-point chars: 'b'=blue 'y'=yellow 'r'=red 'n'=none (lidar does not fill this)
uint8[] type                    # SIZE only: 0=unknown, 1=large, 2=small. Do not copy into HuatCone.type.
