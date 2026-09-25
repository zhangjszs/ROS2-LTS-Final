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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    # #16：单一权威指令出口。true 时 PP 只作仲裁器的源 A，最终出口固定为契约话题 /vehicle_command。
    use_arbiter_arg = DeclareLaunchArgument(
        'use_arbiter',
        default_value='false',
        description='Route the final command through command_arbiter_node (#16 single authority)',
    )
    # 行驶许可默认不授予：需任务层发 arm+start 事件（#16 验收第 1 条）。
    arbiter_autostart_arg = DeclareLaunchArgument(
        'arbiter_autostart',
        default_value='false',
        description='Let the arbiter arm+start itself at startup (bench/simulation only)',
    )

    # Pure pursuit controller
    # #15：车辆执行器标定基线——放在 parameters 首项，包内 yaml 与显式 override 仍可覆盖。
    vehicle_calibration_config = os.path.join(
        get_package_share_directory('huat_launch'),
        'config',
        'vehicle_calibration.yaml',
    )
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
            vehicle_calibration_config,
            pure_pursuit_config,
            {
                'topics.path': LaunchConfiguration('path_topic'),
                'topics.vehicle_state': LaunchConfiguration('vehicle_state_topic'),
                'topics.stop': LaunchConfiguration('stop_topic'),
                # use_arbiter 时 PP 只作仲裁器的源 A，不得直接驱动底盘
                'topics.vehicle_command': PythonExpression([
                    "'/control/vehicle_command' if '", LaunchConfiguration('use_arbiter'),
                    "' == 'true' else '", LaunchConfiguration('vehicle_command_topic'), "'",
                ]),
            },
        ],
    )

    # #16：最终指令仲裁器（use_arbiter=true 时拉起）：两路源 + 锁存 stop → 唯一 /vehicle_command，
    # 并产出 /system/state 遥测供故障注入验收对账。
    command_arbiter_node = Node(
        package='safety_monitor',
        executable='command_arbiter_node',
        name='command_arbiter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_arbiter')),
        parameters=[
            vehicle_calibration_config,
            {
                'topics.source_a': '/control/vehicle_command',
                'topics.source_b': '/mpc/vehicle_command',
                'topics.output': '/vehicle_command',
                'topics.stop': LaunchConfiguration('stop_topic'),
                'arbitration.preferred': 'pure_pursuit',
                'task.autostart': PythonExpression([
                    "'", LaunchConfiguration('arbiter_autostart'), "' == 'true'",
                ]),
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
        use_arbiter_arg,
        arbiter_autostart_arg,

        # Nodes
        pure_pursuit_node,
        command_arbiter_node,
        safety_monitor_launch,
    ])
