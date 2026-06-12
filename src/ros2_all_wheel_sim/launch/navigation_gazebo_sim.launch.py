import os
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "ros2_all_wheel_sim"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),
    DeclareLaunchArgument('map',
                          default_value="my_slam_map",
                          description='Map name in the maps directory'),
    DeclareLaunchArgument('nav_rviz',
                          default_value='true',
                          description='Start navigation RViz if true'),
    DeclareLaunchArgument('debug_rviz',
                          default_value='true',
                          description='Start laser/TF debug RViz if true'),
    DeclareLaunchArgument('sim_rviz',
                          default_value='false',
                          description='Start Gazebo-only RViz if true'),
    DeclareLaunchArgument('headless',
                          default_value='false',
                          description='Start Gazebo server only if true'),
    DeclareLaunchArgument('nav2_use_composition',
                          default_value='False',
                          description='Run Nav2 servers as separate processes if false'),
    DeclareLaunchArgument('nav2_log_level',
                          default_value='info',
                          description='Nav2 log level'),
    DeclareLaunchArgument('log_dir',
                          default_value=os.environ.get(
                              'ROS_LOG_DIR',
                              str(Path.home() / 'work' / 'ROS2' / 'logs' /
                                  ('navigation_' + datetime.now().strftime('%Y%m%d_%H%M%S')))
                          ),
                          description='Directory used by ROS nodes for log files'),
]

def generate_launch_description():
    ros_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value=LaunchConfiguration('log_dir')
    )

    # launch gazebo with spawned robot
    gazebo_sim_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'launch', 'gazebo_sim.launch.py'
            ])
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_sim_path]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('sim_rviz'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    nav2_launch_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'launch', 'nav2.launch.py'
            ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_composition': LaunchConfiguration('nav2_use_composition'),
            'log_level': LaunchConfiguration('nav2_log_level'),
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    rviz_config_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'rviz', 'navigation.rviz'
            ])
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('nav_rviz')),
        )

    debug_rviz_config_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'rviz', 'laser_tf.rviz'
            ])
    debug_rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='laser_tf_rviz2',
            output='screen',
            arguments=['-d', debug_rviz_config_path],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('debug_rviz')),
        )

    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz2, debug_rviz2]
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros_log_dir)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_rviz)
    return ld
