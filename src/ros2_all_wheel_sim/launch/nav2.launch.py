import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

PACKAGE_NAME = "ros2_all_wheel_sim"

ARGUMENTS = [
    DeclareLaunchArgument('map',
                          default_value='maze2',
                          description='Map name in the maps directory'),
]

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution([
                get_package_share_directory("nav2_bringup"), 'launch', 'bringup_launch.py'
            ])
    map_name = LaunchConfiguration("map")
    map_path = [PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'maps', map_name]), '.yaml']
    param_file_path =  os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={
            "map": map_path,
            "params_file": param_file_path,
            "use_sim_time": 'true'}.items()
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_launch)
    return ld
