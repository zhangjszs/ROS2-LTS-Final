# /**
#  * \file lidar_only.launch.py
#  * \brief LiDAR only: Velodyne -> lidar_cluster
#  *
#  * Converts lidar_only.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch lidar_only.launch.py
#  */
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Velodyne driver
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_pointcloud'),
                'launch',
                'VLP-32C_points.launch.py',
            )
        ),
    )

    # LiDAR cluster
    lidar_cluster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cluster'),
                'launch',
                'lidar_cluster.launch.py',
            )
        ),
    )

    return LaunchDescription([
        velodyne_launch,
        lidar_cluster_launch,
    ])
