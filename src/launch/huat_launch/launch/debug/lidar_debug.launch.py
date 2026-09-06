# /**
#  * \file lidar_debug.launch.py
#  * \brief LiDAR debug: Velodyne + lidar_cluster with debug topics enabled
#  *
#  * Converts lidar_debug.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch lidar_debug.launch.py
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    # LiDAR cluster with debug topics enabled
    lidar_cluster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cluster'),
                'launch',
                'lidar_cluster.launch.py',
            )
        ),
        launch_arguments={
            'enable_debug_topics': 'true',
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('lidar_cluster'),
            'rviz_cfg',
            'lidar_cluster.rviz',
        )],
    )

    return LaunchDescription([
        velodyne_launch,
        lidar_cluster_launch,
        rviz_node,
    ])
