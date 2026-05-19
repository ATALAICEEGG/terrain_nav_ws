from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_simulation',
            executable='terrain_map_publisher_node',
            name='terrain_map_publisher_node',
            output='screen'
        ),
        Node(
            package='terrain_analysis',
            executable='terrain_analyzer_node',
            name='terrain_analyzer_node',
            output='screen'
        ),
        Node(
            package='terrain_costmap',
            executable='terrain_costmap_node',
            name='terrain_costmap_node',
            output='screen'
        ),
        Node(
            package='terrain_planner',
            executable='terrain_planner_node',
            name='terrain_planner_node',
            output='screen'
        ),
    ])
