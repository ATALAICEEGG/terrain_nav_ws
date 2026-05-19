from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('terrain_smac_bringup')
    config_file = os.path.join(pkg_dir, 'config', 'smac_params.yaml')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        respawn=False
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        static_tf_publisher,
        lifecycle_manager,
        planner_server,
    ])
