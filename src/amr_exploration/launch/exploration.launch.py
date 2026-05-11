from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    exploration_mode = LaunchConfiguration('exploration_mode')
    use_ekf_filter = LaunchConfiguration('use_ekf_filter')

    return LaunchDescription([
        DeclareLaunchArgument(
            'exploration_mode',
            default_value='straight',
            description=(
                'Exploration mode: straight, free_space, random, '
                'random_small_rotation, or landmark_search'
            ),
        ),
        DeclareLaunchArgument(
            'use_ekf_filter',
            default_value='true',
            description=(
                'If true, fuse /odom + /imu through the EKF for QR pose '
                'estimation. If false, use raw /odom directly.'
            ),
        ),
        Node(
            package='amr_exploration',
            executable='exploration_node_filtered',
            name='exploration_node',
            output='screen',
            parameters=[{
                'exploration_mode': exploration_mode,
                'use_ekf_filter': use_ekf_filter,
            }],
        )
    ])
