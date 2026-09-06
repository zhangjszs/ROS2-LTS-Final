# /**
#  * \file skidpad.launch.py
#  * \brief Skidpad planner (figure-8, ICP+APF)
#  *
#  * Converts skidpad.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Output: /planning/skidpad/pathlimits
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
    input_cone_map_topic_arg = DeclareLaunchArgument(
        'input_cone_map_topic',
        default_value='/sensors/cones/fused',
        description='Input cone map topic',
    )
    output_pathlimits_topic_arg = DeclareLaunchArgument(
        'output_pathlimits_topic',
        default_value='/planning/skidpad/pathlimits',
        description='Output path limits topic',
    )

    # Skidpad planner config
    skidpad_config = os.path.join(
        get_package_share_directory('skidpad_planner'),
        'config',
        'skidpad_planner.yaml',
    )

    # Skidpad planner node
    skidpad_planner_node = Node(
        package='skidpad_planner',
        executable='skidpad_planner',
        name='skidpad_planner',
        output='screen',
        parameters=[
            skidpad_config,
            {
                'road_type': 1,
                'input_cone_map_topic': LaunchConfiguration('input_cone_map_topic'),
                'output_pathlimits_topic': LaunchConfiguration('output_pathlimits_topic'),
            },
        ],
    )

    return LaunchDescription([
        input_cone_map_topic_arg,
        output_pathlimits_topic_arg,
        skidpad_planner_node,
    ])
