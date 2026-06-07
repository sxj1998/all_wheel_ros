from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ros2_all_wheel_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true.',
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution (meters per cell).',
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Map publish period in seconds.',
    )
    declare_configuration_directory = DeclareLaunchArgument(
        'configuration_directory',
        default_value=PathJoinSubstitution([pkg_share, 'config']),
        description='Directory for cartographer configuration files.',
    )
    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='all_wheel_2d.lua',
        description='Cartographer configuration filename.',
    )

    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'cartographer.rviz'])

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan'), ('odom', '/odom')],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_resolution,
        declare_publish_period_sec,
        declare_configuration_directory,
        declare_configuration_basename,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])
