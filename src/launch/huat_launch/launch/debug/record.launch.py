# /**
#  * \file record.launch.py
#  * \brief Record core topics for playback and analysis
#  *
#  * Converts record.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch record.launch.py bag_path:=/tmp/huat_run.bag
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/tmp/huat_run.bag',
        description='Output path for recorded bag',
    )

    # Record core topics using ros2 bag record
    record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_path'),
            '/velodyne_points',
            '/INS/ASENSING_INS',
            '/localization/vehicle_state',
            '/sensors/cones/raw',
            '/sensors/cones/fused',
            '/planning/pathlimits',
            '/planning/track/pathlimits',
            '/planning/skidpad/pathlimits',
            '/planning/straight_line/pathlimits',
            '/control/vehicle_command',
            '/system/stop',
            '/diagnostics',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        record_cmd,
    ])
