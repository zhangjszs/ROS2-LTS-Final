# /**
#  * \file vision_track_replay.launch.py
#  * \brief Track replay + YOLO vision detection (comparison demo)
#  *
#  * Converts vision_track_replay.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Launches:
#  *   - Track run stack (LiDAR, planning, control, safety, fsd_viz + RViz)
#  *   - YOLO vision detection (vision_node)
#  *   - Cone visualizer OpenCV window (cone_visualizer)
#  *   - Rosbag playback
#  *
#  * Usage:
#  *   ros2 launch vision_ros vision_track_replay.launch.py
#  *   ros2 launch vision_ros vision_track_replay.launch.py bag:=/home/kerwin/rosbag/track.bag
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value=os.path.join(os.environ.get('HOME', ''), 'rosbag', 'track.bag'),
        description='Path to rosbag for track replay',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/resize_img_out',
        description='Image topic',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            get_package_share_directory('vision_ros'),
            'models',
            'yolo26l_48g',
            'weights',
            'best.onnx',
        ),
        description='ONNX model path',
    )
    save_video_arg = DeclareLaunchArgument(
        'save_video',
        default_value='true',
        description='Whether to save video',
    )
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='/tmp/cone_viz_track.avi',
        description='Output video path',
    )
    max_det_age_sec_arg = DeclareLaunchArgument(
        'max_det_age_sec',
        default_value='5.0',
        description='Detection result retention time',
    )

    # Track run stack (sensors + planning + control + safety + RViz)
    track_run_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'main',
                'track_run.launch.py',
            )
        ),
        launch_arguments={
            'enable_velodyne_driver': 'false',
            'vehicle_command_topic': '/debug/replay/vehicle_command',
        }.items(),
    )

    # Rosbag playback
    rosbag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '--clock',
            LaunchConfiguration('bag'),
        ],
        output='screen',
    )

    # Config file paths
    vision_base_config = os.path.join(
        get_package_share_directory('vision_ros'),
        'config',
        'vision_base.yaml',
    )
    vision_track_config = os.path.join(
        get_package_share_directory('vision_ros'),
        'config',
        'vision_track.yaml',
    )
    vision_local_config = os.path.join(
        get_package_share_directory('vision_ros'),
        'config',
        'vision_local_yolo26l_48g_onnx.yaml',
    )

    # YOLO vision detection node
    vision_node = Node(
        package='vision_ros',
        executable='vision_node_py.py',
        name='vision_node',
        output='screen',
        namespace='perception/vision',
        parameters=[
            vision_base_config,
            vision_track_config,
            vision_local_config,
            {
                'inference/model_path': LaunchConfiguration('model_path'),
                'node/image_topic': LaunchConfiguration('image_topic'),
                'node/require_model': True,
                'output/publish_debug_image': True,
            },
        ],
    )

    # Cone visualizer (OpenCV window, auto headless when no DISPLAY)
    cone_visualizer_node = Node(
        package='vision_ros',
        executable='cone_visualizer.py',
        name='cone_visualizer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'detection_topic': '/perception/vision/detections',
            'save_video': LaunchConfiguration('save_video'),
            'video_path': LaunchConfiguration('video_path'),
            'max_det_age_sec': LaunchConfiguration('max_det_age_sec'),
        }],
    )

    return LaunchDescription([
        # Arguments
        bag_arg,
        image_topic_arg,
        model_path_arg,
        save_video_arg,
        video_path_arg,
        max_det_age_sec_arg,

        # Launches
        track_run_launch,

        # Nodes
        rosbag_play_cmd,
        vision_node,
        cone_visualizer_node,
    ])
