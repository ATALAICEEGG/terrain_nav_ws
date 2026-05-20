from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
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
        DeclareLaunchArgument('trace_file',
            default_value='/home/luym/terrain_nav_ws/log/step56_trace/step56_smac_integration.jsonl'),

        # 模拟 odom + imu（不模拟 global_path，不模拟 terrain_cost）
        Node(
            package='dog_nav_step56',
            executable='odom_imu_sim_node',
            name='odom_imu_sim_node',
            output='screen',
        ),

        # GridMap terrain_cost 层 → Float32
        Node(
            package='dog_nav_step56',
            executable='terrain_cost_scalar_bridge_node',
            name='terrain_cost_scalar_bridge_node',
            output='screen',
        ),

        # 全局路径跟踪 → cmd_vel（订阅 /global_path_smac）
        Node(
            package='dog_nav_step56',
            executable='local_controller_node',
            name='local_controller_node',
            output='screen',
            parameters=[{
                'path_topic': '/global_path_smac',
                'odom_topic': '/odometry/filtered',
                'cmd_vel_topic': '/cmd_vel',
            }],
        ),

        # 安全监控
        Node(
            package='dog_nav_step56',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[safety_params],
        ),

        # 速度指令 → 机器狗动作
        Node(
            package='dog_nav_step56',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[motion_params],
        ),

        # 记录 trace
        Node(
            package='dog_nav_step56',
            executable='test_trace_recorder_node',
            name='test_trace_recorder_node',
            output='screen',
            parameters=[{
                'trace_file': '/home/luym/terrain_nav_ws/log/step56_trace/step56_smac_integration.jsonl',
                'global_path_topic': '/global_path_smac',
                'odom_topic': '/odometry/filtered',
                'imu_topic': '/imu/data',
                'terrain_cost_topic': '/terrain_cost',
                'cmd_vel_topic': '/cmd_vel',
                'safety_topic': '/safety_state',
                'motion_topic': '/motion_command',
            }],
        ),
    ])
