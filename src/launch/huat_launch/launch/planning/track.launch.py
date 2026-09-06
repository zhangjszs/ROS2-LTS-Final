# /**
#  * \file track.launch.py
#  * \brief Track planner (Delaunay triangulation, loop closure)
#  *
#  * Converts track.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Output: /planning/track/pathlimits
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
    input_cones_topic_arg = DeclareLaunchArgument(
        'input_cones_topic',
        default_value='/sensors/cones/fused',
        description='Input cones topic',
    )
    input_pose_topic_arg = DeclareLaunchArgument(
        'input_pose_topic',
        default_value='/localization/vehicle_state',
        description='Input vehicle pose topic',
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/planning/track/pathlimits',
        description='Output path limits topic',
    )
    stop_topic_arg = DeclareLaunchArgument(
        'stop_topic',
        default_value='/planning/track/stop_request',
        description='Stop request topic',
    )

    # Urinay config
    urinay_config = os.path.join(
        get_package_share_directory('urinay'),
        'config',
        'urinay.yml',
    )

    # Track planner (urinay) node
    urinay_node = Node(
        package='urinay',
        executable='urinay_exec',
        name='urinay',
        output='screen',
        parameters=[
            urinay_config,
            {
                'road_type': 3,
                'input_cones_topic': LaunchConfiguration('input_cones_topic'),
                'input_pose_topic': LaunchConfiguration('input_pose_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'stop_topic': LaunchConfiguration('stop_topic'),
            },
        ],
    )

    return LaunchDescription([
        input_cones_topic_arg,
        input_pose_topic_arg,
        output_topic_arg,
        stop_topic_arg,
        urinay_node,
    ])
