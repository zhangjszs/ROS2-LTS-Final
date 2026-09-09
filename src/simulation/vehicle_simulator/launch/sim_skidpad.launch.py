import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('vehicle_simulator')
    skidpad_pkg_dir = get_package_share_directory('skidpad_planner')
    pp_pkg_dir = get_package_share_directory('pure_pursuit')
    viz_pkg_dir = get_package_share_directory('fsac_viz')
    huat_launch_dir = get_package_share_directory('huat_launch')

    default_track = os.path.join(sim_pkg_dir, 'tracks', 'skidpad_track.csv')
    default_sim_params = os.path.join(sim_pkg_dir, 'config', 'simulator_params.yaml')
    default_skidpad_params = os.path.join(skidpad_pkg_dir, 'config', 'skidpad_planner.yaml')
    default_pp_params = os.path.join(pp_pkg_dir, 'config', 'pure_pursuit.yaml')
    default_viz_params = os.path.join(viz_pkg_dir, 'config', 'fsd_viz.yaml')
    default_rviz_config = os.path.join(huat_launch_dir, 'config', 'tuxiang.rviz')

    # 1. 车辆与传感器运动学闭环仿真器
    sim_node = Node(
        package='vehicle_simulator',
        executable='vehicle_simulator_node',
        name='vehicle_simulator',
        output='screen',
        parameters=[
            default_sim_params,
            {
                'track_file': default_track,
                'init_x': -12.0,
                'init_y': 0.0,
                'init_theta': 0.0,
                'init_v': 0.0,
            }
        ]
    )

    # 2. 八字绕环规划器 (ICP + APF)
    skidpad_node = Node(
        package='skidpad_planner',
        executable='skidpad_planner',
        name='skidpad_planner',
        output='screen',
        parameters=[
            default_skidpad_params,
            {
                'road_type': 1,
                'input_cone_map_topic': '/sensors/cones/fused',
                'output_pathlimits_topic': '/planning/pathlimits',
                'vehicle_state_topic': '/localization/vehicle_state',
            }
        ]
    )

    # 3. 纯跟踪控制器 (Pure Pursuit)
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            default_pp_params,
            {
                'topics.vehicle_state': '/localization/vehicle_state',
                'topics.path': '/planning/pathlimits',
                'topics.vehicle_command': '/vehicle_command',
                'topics.stop': '/system/stop',
            }
        ]
    )

    # 4. 统一 RViz 赛车与赛道可视化器
    viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
        parameters=[
            default_viz_params,
            {
                'vehicle_state_topic': '/localization/vehicle_state',
                'cone_map_topic': '/sensors/cones/fused',
                'path_topic': '/planning/pathlimits',
                'show_trail': True,
                'fixed_frame': 'map',
            }
        ]
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else []
    )

    return LaunchDescription([
        sim_node,
        skidpad_node,
        pure_pursuit_node,
        viz_node,
        rviz_node
    ])
