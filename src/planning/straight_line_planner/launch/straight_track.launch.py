# /**
#  * \file straight_track.launch.py
#  * \brief Straight line track: sensor stack + straight line planner + visualization + rosbag
#  *
#  * Converts straight_track.launch (ROS1) to ROS2 Python launch file.
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Sensor stack
    sensor_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'sensors',
                'sensor_stack.launch.py',
            )
        ),
    )

    # Straight line planner config
    straight_line_config = os.path.join(
        get_package_share_directory('straight_line_planner'),
        'config',
        'straight_line_planner.yaml',
    )

    # Straight line planner node
    straight_line_planner_node = Node(
        package='straight_line_planner',
        executable='straight_line_planner',
        name='straight_line_planner',
        output='screen',
        parameters=[straight_line_config],
    )

    # Unified visualization
    fsd_viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='cone_rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('huat_launch'),
            'config',
            'tuxiang.rviz',
        )],
    )

    # Rosbag playback
    rosbag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            '/home/kerwin/rosbag/accel.bag',
        ],
        output='screen',
    )

    return LaunchDescription([
        sensor_stack_launch,
        straight_line_planner_node,
        fsd_viz_node,
        rviz_node,
        rosbag_play_cmd,
    ])
