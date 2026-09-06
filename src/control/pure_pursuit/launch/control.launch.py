# /**
#  * \file control.launch.py
#  * \brief Pure pursuit controller + safety monitor
#  *
#  * Converts control.launch (ROS1) to ROS2 Python launch file.
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
        default_value='1',
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
    path_timeout_arg = DeclareLaunchArgument(
        'path_timeout',
        default_value='0.5',
        description='Path timeout (seconds)',
    )
    state_timeout_arg = DeclareLaunchArgument(
        'state_timeout',
        default_value='0.3',
        description='State timeout (seconds)',
    )

    # Pure pursuit controller config
    pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit.yaml',
    )

    # Pure pursuit controller node
    pp_car_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='PP_car_node',
        output='screen',
        parameters=[
            pure_pursuit_config,
            {
                'road_type': LaunchConfiguration('road_type'),
                'topics/path': LaunchConfiguration('path_topic'),
                'topics/vehicle_state': LaunchConfiguration('vehicle_state_topic'),
                'topics/stop': LaunchConfiguration('stop_topic'),
                'topics/vehicle_command': LaunchConfiguration('vehicle_command_topic'),
                'safety/path_timeout': LaunchConfiguration('path_timeout'),
                'safety/state_timeout': LaunchConfiguration('state_timeout'),
            },
        ],
    )

    # Safety monitor
    safety_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('safety_monitor'),
                'launch',
                'safety_monitor.launch.py',
            )
        ),
        launch_arguments={
            'pathlimits_topic': LaunchConfiguration('path_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'stop_topic': LaunchConfiguration('stop_topic'),
        }.items(),
    )

    return LaunchDescription([
        # Arguments
        road_type_arg,
        path_topic_arg,
        vehicle_state_topic_arg,
        stop_topic_arg,
        vehicle_command_topic_arg,
        path_timeout_arg,
        state_timeout_arg,

        # Nodes
        pp_car_node,
        safety_monitor_launch,
    ])
