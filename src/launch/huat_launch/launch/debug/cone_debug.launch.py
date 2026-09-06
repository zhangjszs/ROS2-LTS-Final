# /**
#  * \file cone_debug.launch.py
#  * \brief Cone debug: sensor stack + track planner + visualization + RViz
#  *
#  * Converts cone_debug.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch cone_debug.launch.py
#  *   ros2 launch huat_launch cone_debug.launch.py bag:=/path/to/track.bag
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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to rosbag for playback',
    )

    # Sensor stack (disable velodyne driver for rosbag replay)
    sensor_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'sensors',
                'sensor_stack.launch.py',
            )
        ),
        launch_arguments={
            'enable_velodyne_driver': 'false',
        }.items(),
    )

    # Track planner
    track_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'planning',
                'track.launch.py',
            )
        ),
    )

    # Unified visualization node (2025 style)
    fsd_viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
        parameters=[{
            'fixed_frame': 'map',
            'use_mesh': False,
            'show_confidence_alpha': True,
            'show_distance_label': True,
            'show_trail': True,
            'show_velocity_arrow': True,
            'show_status_text': True,
            'show_boundaries': True,
            'vehicle_state_topic': '/localization/vehicle_state',
            'path_topic': '/planning/track/pathlimits',
            'ins_topic': '/INS/ASENSING_INS',
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('fsac_viz'),
            'config',
            'fsac_viz.rviz',
        )],
    )

    # Rosbag playback (only if bag argument is non-empty)
    # ROS2 uses 'ros2 bag play' via ExecuteProcess
    rosbag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '--clock',
            LaunchConfiguration('bag'),
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])
        ),
    )

    return LaunchDescription([
        # Arguments
        bag_arg,

        # Sensor stack
        sensor_stack_launch,

        # Track planner
        track_planner_launch,

        # Visualization
        fsd_viz_node,
        rviz_node,

        # Rosbag playback (conditional)
        rosbag_play_cmd,
    ])
