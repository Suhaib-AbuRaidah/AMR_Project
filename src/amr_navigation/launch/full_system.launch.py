"""Top-level bringup for the full AMR service-robot system.

Composes:
  - Gazebo world (amr_gazebo/town.launch.py)
  - ROS↔Gazebo bridge (amr_gazebo/bridge.launch.py)
  - Nav2 stack with map + AMCL (amr_navigation/navigation.launch.py)
  - Mission executor (amr_mission_executor/mission_executor.launch.py)
  - Mission Console GUI (amr_gui/gui.launch.py)

Optional features (launch args):
  - use_rviz:=true            Launch RViz2 alongside the system.
  - auto_init_pose:=false     Skip the automatic /initialpose publish.

Default behavior: 8 seconds after launch, publishes an initial pose at
map (0, 0, 0) so AMCL converges without manual intervention. Override
this by passing auto_init_pose:=false and publishing /initialpose
yourself (terminal or RViz "2D Pose Estimate").
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---- Resolve sub-launch paths ----
    gazebo_share = get_package_share_directory("amr_gazebo")
    nav_share = get_package_share_directory("amr_navigation")
    executor_share = get_package_share_directory("amr_mission_executor")
    gui_share = get_package_share_directory("amr_gui")

    # ---- Launch arguments ----
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, also launch RViz2.",
    )
    auto_init_pose_arg = DeclareLaunchArgument(
        "auto_init_pose",
        default_value="true",
        description=(
            "If true, auto-publish an initial pose to /initialpose 8s "
            "after launch so AMCL doesn't need a manual click."
        ),
    )

    # ---- Sub-launches ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "town.launch.py")
        ),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "bridge.launch.py")
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "navigation.launch.py")
        ),
    )

    executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(executor_share, "launch", "mission_executor.launch.py")
        ),
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gui_share, "launch", "gui.launch.py")
        ),
    )

    # ---- Auto initial pose (delayed) ----
    # AMCL needs ~5-8 seconds after Nav2 launch to be ready to accept
    # /initialpose. We delay 8s to be safe. If your robot does not
    # actually spawn at world (0, 0, 0), pass auto_init_pose:=false and
    # publish manually.
    initial_pose_yaml = (
        "{header: {frame_id: 'map'}, "
        "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, "
        "covariance: ["
        "0.25, 0, 0, 0, 0, 0, "
        "0, 0.25, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0.0685"
        "]}}"
    )
    init_pose_publish = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub",
            "--times", "3",
            "--rate", "1",
            "--qos-reliability", "best_effort",
            "/initialpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            initial_pose_yaml,
        ],
        output="screen",
    )
    delayed_init_pose = TimerAction(
        period=8.0,
        actions=[init_pose_publish],
        condition=IfCondition(LaunchConfiguration("auto_init_pose")),
    )

    # ---- Optional RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        # args first
        use_rviz_arg,
        auto_init_pose_arg,
        # core stack
        gazebo_launch,
        bridge_launch,
        navigation_launch,
        executor_launch,
        gui_launch,
        # extras
        delayed_init_pose,
        rviz_node,
    ])
