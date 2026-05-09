from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_exploration',
            executable='exploration_node',
            name='exploration_node',
            output='screen'
        )
    ])
