# /**
#  * \file control_only.launch.py
#  * \brief Control only: pure_pursuit + safety_monitor (no sensors/planners)
#  *
#  * Converts control_only.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch control_only.launch.py
#  *   ros2 launch huat_launch control_only.launch.py path_topic:=/planning/track/pathlimits
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
    # Declare launch arguments
    road_type_arg = DeclareLaunchArgument(
        'road_type',
        default_value='3',
        description='Road type selector',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/planning/pathlimits',
        description='Path topic from planner',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )
    stop_topic_arg = DeclareLaunchArgument(
        'stop_topic',
        default_value='/system/stop',
        description='Stop signal topic',
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/control/vehicle_command',
        description='Vehicle command topic',
    )

    # Pure pursuit controller
    pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit.yaml',
    )
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            pure_pursuit_config,
            {
                'topics/path': LaunchConfiguration('path_topic'),
                'topics/vehicle_state': LaunchConfiguration('vehicle_state_topic'),
                'topics/stop': LaunchConfiguration('stop_topic'),
                'topics/vehicle_command': LaunchConfiguration('vehicle_command_topic'),
            },
        ],
    )

    # Safety monitor
    safety_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'safety',
                'safety_monitor.launch.py',
            )
        ),
        launch_arguments={
            'pathlimits_topic': LaunchConfiguration('path_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'stop_topic': LaunchConfiguration('stop_topic'),
            'stop_request_topic': '',
        }.items(),
    )

    return LaunchDescription([
        # Arguments
        road_type_arg,
        path_topic_arg,
        vehicle_state_topic_arg,
        stop_topic_arg,
        vehicle_command_topic_arg,

        # Nodes
        pure_pursuit_node,
        safety_monitor_launch,
    ])
