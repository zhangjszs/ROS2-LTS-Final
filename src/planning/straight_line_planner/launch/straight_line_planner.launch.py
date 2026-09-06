# /**
#  * \file straight_line_planner.launch.py
#  * \brief Straight line planner node (standalone)
#  *
#  * Converts straight_line_planner.launch (ROS1) to ROS2 Python launch file.
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
    road_type_arg = DeclareLaunchArgument(
        'road_type',
        default_value='2',
        description='Road type selector (2=straight line)',
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
        parameters=[
            straight_line_config,
            {
                'road_type': LaunchConfiguration('road_type'),
            },
        ],
    )

    return LaunchDescription([
        road_type_arg,
        straight_line_planner_node,
    ])
