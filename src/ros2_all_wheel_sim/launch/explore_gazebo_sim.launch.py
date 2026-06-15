import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


PACKAGE_NAME = "ros2_all_wheel_sim"

ARGUMENTS = [
    DeclareLaunchArgument("world", default_value="maze2", description="Gazebo world"),
    DeclareLaunchArgument("nav_rviz", default_value="true", description="Start navigation RViz"),
    DeclareLaunchArgument("headless", default_value="false", description="Start Gazebo server only"),
    DeclareLaunchArgument("explore", default_value="false", description="Start autonomous exploration"),
    DeclareLaunchArgument("nav2_use_composition", default_value="False"),
    DeclareLaunchArgument("nav2_log_level", default_value="info"),
    DeclareLaunchArgument(
        "map_save_path",
        default_value=str(Path.home() / "work" / "ROS2" / "maps" / "explored_maze2"),
        description="Output path prefix for the explored map",
    ),
    DeclareLaunchArgument(
        "log_dir",
        default_value=os.environ.get(
            "ROS_LOG_DIR",
            str(
                Path.home()
                / "work"
                / "ROS2"
                / "logs"
                / ("exploration_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
            ),
        ),
        description="Directory used by ROS nodes for log files",
    ),
]


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", "gazebo_sim.launch.py"])]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "rviz": "false",
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", "nav2.launch.py"])]
        ),
        launch_arguments={
            "slam": "True",
            "use_composition": LaunchConfiguration("nav2_use_composition"),
            "log_level": LaunchConfiguration("nav2_log_level"),
        }.items(),
    )

    explorer = Node(
        package=PACKAGE_NAME,
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "plan_period": 2.0,
                "min_frontier_size": 8,
                "frontier_goal_offset": 0.65,
                "goal_search_radius": 1.20,
                "min_goal_obstacle_clearance": 0.32,
                "path_obstacle_clearance": 0.30,
                "goal_blacklist_radius": 0.55,
                "max_goal_duration": 180.0,
                "completion_idle_cycles": 6,
                "auto_save_map": True,
                "map_save_path": LaunchConfiguration("map_save_path"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("explore")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([package_share, "rviz", "navigation.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("nav_rviz")),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(SetEnvironmentVariable(name="ROS_LOG_DIR", value=LaunchConfiguration("log_dir")))
    # RViz and Gazebo are more stable in many VM/Wayland setups when Qt uses X11
    # and Mesa falls back to software rendering.
    ld.add_action(SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"))
    ld.add_action(SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"))
    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(TimerAction(period=10.0, actions=[nav2]))
    # Give Gazebo controllers, SLAM, and Nav2 lifecycle nodes enough time to
    # publish stable TF/map data before the first autonomous frontier goal.
    ld.add_action(TimerAction(period=35.0, actions=[explorer]))
    return ld
