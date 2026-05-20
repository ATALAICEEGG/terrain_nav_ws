from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulate_tilt = LaunchConfiguration('simulate_tilt')
    terrain_cost = LaunchConfiguration('terrain_cost')

    motion_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'motion_adapter.yaml',
    ])
    safety_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'safety_supervisor.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('simulate_tilt', default_value='false'),
        DeclareLaunchArgument('terrain_cost', default_value='20.0'),
        Node(
            package='dog_nav_step56',
            executable='simulated_input_node',
            name='simulated_input_node',
            output='screen',
            parameters=[{
                'simulate_tilt': simulate_tilt,
                'terrain_cost': terrain_cost,
            }],
        ),
        Node(
            package='dog_nav_step56',
            executable='local_controller_node',
            name='local_controller_node',
            output='screen',
            parameters=[{}],
        ),
        Node(
            package='dog_nav_step56',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[safety_params],
        ),
        Node(
            package='dog_nav_step56',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[motion_params],
        ),
    ])
