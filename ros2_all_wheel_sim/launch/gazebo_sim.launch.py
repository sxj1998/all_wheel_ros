import os  # 读取环境变量
from ament_index_python.packages import get_package_share_directory  # 获取包 share 路径
from launch import LaunchDescription  # launch 描述容器
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription  # 传入 launch 参数、设置环境变量、引入其他 launch
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 引入 Python launch 文件
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution  # 运行期参数取值、命令替换、拼路径
from launch_ros.actions import Node  # 启动 ROS2 节点
from launch_ros.parameter_descriptions import ParameterValue  # 强制参数类型
from pathlib import Path  # 路径处理

PACKAGE_NAME = "ros2_all_wheel_sim"

ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        default_value="maze2",
        description='Gazebo World',
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true',
    ),
]


def generate_launch_description():
    # Step 0: 读取模型与参数
    robot_model = os.environ.get("OMNI_ROBOT_MODEL", "all_wheel")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Step 1: 设置 Gazebo 资源路径（world/mesh）
    pkg_path = get_package_share_directory(PACKAGE_NAME)
    resource_paths = [
        str(Path(pkg_path).parent.resolve()), ":",  # share/ 上一级，保证 package:// 可解析
        os.path.join(pkg_path, 'worlds'),  # world 文件目录
    ]
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_paths
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths
    )

    # Step 2: 生成 robot_description 并启动 robot_state_publisher
    xacro_file = os.path.join(pkg_path, 'urdf', 'all_wheel', 'ALL_WHEEL.urdf')
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.08', '0', '0.08',
            '0', '0', '3.1415926',
            'base_link',
            'all_wheel/base_footprint/lidar_sensor',
        ],
        output='screen'
    )

    # Step 3: 启动 Gazebo
    gazebo_launch_path = PathJoinSubstitution([
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4'
            ])
        ]
    )

    # Step 4: 生成机器人实体
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_model,
            '-z', '0.1'
        ],
        output='screen'
    )

    # Step 5: 启动 ros_gz_bridge
    bridge_params = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'gz_bridge',
        'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_params}],
    )

    # Step 6: 启动控制器
    spawn_wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'wheel1_controller',
            'wheel2_controller',
            'wheel3_controller',
        ],
        output='screen'
    )


    # Step 7: 启动运动学节点
    kinematics = Node(
        package=PACKAGE_NAME,
        executable='kinematics',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Step 8: 可选 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Step 9: 把所有动作加入 LaunchDescription 并返回
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(gz_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(static_lidar_tf)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(spawn_wheel_controller)
    ld.add_action(kinematics)
    ld.add_action(rviz_node)
    return ld
