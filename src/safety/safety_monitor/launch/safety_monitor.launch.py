# /**
#  * \file safety_monitor.launch.py
#  * \brief Safety monitor: watches planner activity, publishes emergency stop
#  *
#  * Converts safety_monitor.launch (ROS1) to ROS2 Python launch file.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    pathlimits_topic_arg = DeclareLaunchArgument(
        'pathlimits_topic',
        default_value='/planning/pathlimits',
        description='Path limits topic from planner',
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
    stop_request_topic_arg = DeclareLaunchArgument(
        'stop_request_topic',
        default_value='/planning/track/stop_request',
        description='Stop request topic',
    )

    # Safety monitor config
    safety_monitor_config = os.path.join(
        get_package_share_directory('safety_monitor'),
        'config',
        'safety_monitor.yaml',
    )

    # Safety monitor node
    safety_monitor_node = Node(
        package='safety_monitor',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[
            safety_monitor_config,
            {
                'pathlimits_topic': LaunchConfiguration('pathlimits_topic'),
                'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
                'stop_topic': LaunchConfiguration('stop_topic'),
                'stop_request_topic': LaunchConfiguration('stop_request_topic'),
            },
        ],
    )

    return LaunchDescription([
        pathlimits_topic_arg,
        vehicle_state_topic_arg,
        stop_topic_arg,
        stop_request_topic_arg,
        safety_monitor_node,
    ])
