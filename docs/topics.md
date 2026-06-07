# ROS 2 Topic 清单

本文档记录当前工程显式配置或在 RViz 中使用到的 ROS 2 topic。清单主要来自 `src/kinematics.cpp`、`config/gz_bridge/gz_bridge.yaml`、`config/nav2_params.yaml`、`config/slam_toolbox.yaml`、launch 文件和 RViz 配置。

Topic 名称以运行时常见的全局名称书写；源码或参数中写成相对名时，默认会解析到节点命名空间下。本工程 launch 当前未设置额外 namespace，因此通常表现为下面的全局 topic。

## 仿真与机器人状态

| Topic | 消息类型 | 方向 / 节点 | 用途 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS | 仿真时钟，所有 `use_sim_time: true` 节点使用。 |
| `/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` 发布；`ros_gz_sim create` 和 RViz 订阅 | 机器人 URDF 描述，用于生成 Gazebo 实体和 RViz RobotModel。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher`、`omni_kinematics`、Nav2/SLAM 发布或订阅 | 动态 TF，例如 `odom -> base_footprint`。 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher`、`static_transform_publisher` 发布；Nav2/SLAM/RViz 订阅 | 静态 TF，例如机器人固定关节和雷达静态变换。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` 发布；`omni_kinematics` 和 `robot_state_publisher` 订阅 | 轮子关节状态，用于里程计积分和机器人状态发布。 |

## Gazebo 传感器桥接

这些 topic 在 `config/gz_bridge/gz_bridge.yaml` 中配置，方向都是 `GZ_TO_ROS`。

| Topic | 消息类型 | 发布者 | 主要订阅者 / 用途 |
| --- | --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo IMU 传感器经 bridge 发布 | `omni_kinematics` 读取 yaw 和角速度。 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo 雷达经 bridge 发布 | SLAM Toolbox、AMCL、Nav2 costmap、collision monitor、RViz 使用。 |
| `/camera` | `sensor_msgs/msg/Image` | Gazebo 左相机经 bridge 发布 | RViz 图像显示或视觉调试。 |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo 左相机信息经 bridge 发布 | 相机标定元数据。 |
| `/camera_right` | `sensor_msgs/msg/Image` | Gazebo 右相机经 bridge 发布 | RViz 图像显示或双目调试。 |
| `/camera_right/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo 右相机信息经 bridge 发布 | 相机标定元数据。 |
| `/depth` | `sensor_msgs/msg/Image` | Gazebo 深度图像经 bridge 发布 | 深度图像流，当前用于后续订阅者或调试。 |

## 运动学与轮子控制

| Topic | 消息类型 | 方向 / 节点 | 用途 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2、collision monitor 或 teleop 发布；`omni_kinematics` 订阅 | 机器人速度指令，`omni_kinematics` 将其转换为轮子速度指令。 |
| `/odom` | `nav_msgs/msg/Odometry` | `omni_kinematics` 发布；Nav2、SLAM Toolbox、velocity smoother、RViz 订阅 | `odom` 坐标系下的机器人里程计。 |
| `/wheel1_controller/commands` | `std_msgs/msg/Float64MultiArray` | `omni_kinematics` 发布；`wheel1_controller` 订阅 | 前轮速度指令。 |
| `/wheel2_controller/commands` | `std_msgs/msg/Float64MultiArray` | `omni_kinematics` 发布；`wheel2_controller` 订阅 | 左轮速度指令。 |
| `/wheel3_controller/commands` | `std_msgs/msg/Float64MultiArray` | `omni_kinematics` 发布；`wheel3_controller` 订阅 | 右轮速度指令。 |

## SLAM

| Topic | 消息类型 | 方向 / 节点 | 用途 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | SLAM Toolbox 订阅 | 雷达输入，由 `scan_topic: /scan` 配置。 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox 或 Nav2 map server 发布；RViz/Nav2 订阅 | 栅格地图。 |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | SLAM Toolbox 激活时发布；RViz 订阅 | 增量地图更新。 |

## Nav2 与导航调试

这些 topic 直接配置在 `config/nav2_params.yaml` 中，或出现在 `rviz/navigation.rviz` 里。

| Topic | 消息类型 | 发布者 / 订阅者 | 用途 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | AMCL、local/global costmap、collision monitor 订阅 | 定位和障碍物观测来源。 |
| `/odom` | `nav_msgs/msg/Odometry` | Velocity smoother 订阅；Nav2 也通过 TF/参数使用 odom | 速度反馈和导航状态。 |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Nav2 velocity smoother 发布；collision monitor 订阅 | 安全过滤前的平滑速度指令。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Collision monitor 发布；`omni_kinematics` 订阅 | 发送给机器人的最终速度指令。 |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | Collision monitor 发布 | Collision monitor 运行状态。 |
| `/polygon_stop` | `geometry_msgs/msg/PolygonStamped` | Collision monitor 发布 | 停止区域可视化。 |
| `/polygon_slowdown` | `geometry_msgs/msg/PolygonStamped` | Collision monitor 发布 | 减速区域可视化。 |
| `/polygon_limit` | `geometry_msgs/msg/PolygonStamped` | Collision monitor 启用时发布 | 速度限制区域可视化。 |
| `/velocity_polygon_stop` | `geometry_msgs/msg/PolygonStamped` | 启用 `VelocityPolygonStop` 时由 collision monitor 发布 | 与速度相关的停止区域可视化。 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | Local costmap 发布；behavior server、collision monitor 和 RViz 使用 | Local costmap 中的机器人 footprint。 |
| `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | Global costmap 发布；behavior server 和 RViz 使用 | Global costmap 中的机器人 footprint。 |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Local costmap 发布；behavior server 订阅 | Nav2 behavior 使用的原始 local costmap。 |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Global costmap 发布；behavior server 订阅 | Nav2 behavior 使用的原始 global costmap。 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Local costmap 发布；RViz 显示 | Local costmap 可视化。 |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | Local costmap 发布；RViz 显示 | Local costmap 增量更新。 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Global costmap 发布；RViz 显示 | Global costmap 可视化。 |
| `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | Global costmap 发布；RViz 显示 | Global costmap 增量更新。 |
| `/local_costmap/voxel_marked_cloud` | `sensor_msgs/msg/PointCloud2` | Local voxel layer 启用时发布；RViz 显示 | Local voxel 障碍物标记点云。 |
| `/global_costmap/voxel_marked_cloud` | `sensor_msgs/msg/PointCloud2` | Global voxel layer 启用时发布；RViz 显示 | Global voxel 障碍物标记点云。 |
| `/particle_cloud` | `geometry_msgs/msg/PoseArray` | AMCL 发布；RViz 显示 | 粒子滤波点云。 |
| `/plan` | `nav_msgs/msg/Path` | Planner server 发布；RViz 显示 | 全局路径。 |
| `/local_plan` | `nav_msgs/msg/Path` | Controller server 发布；RViz 显示 | 局部控制路径。 |
| `/marker` | `visualization_msgs/msg/Marker` | Nav2 或 controller 调试发布者激活时发布；RViz 显示 | 导航调试 marker。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 发布；AMCL/Nav2 订阅 | RViz 设置的初始位姿。 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz goal 工具发布；Nav2 订阅 | 导航目标位姿。 |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz 发布 | RViz 点击点调试工具。 |
| `/waypoints` | 通常为 `nav_msgs/msg/Path` 或插件相关 waypoint topic | RViz/Nav2 waypoint 工具使用 | waypoint 可视化或输入，取决于启用的 RViz 插件。 |

## 仅 RViz 使用或可选显示

这些 topic 出现在 RViz 配置中，但当前 launch 组合不一定会发布它们。

| Topic | 消息类型 | 用途 |
| --- | --- | --- |
| `/mobile_base/sensors/bumper_pointcloud` | `sensor_msgs/msg/PointCloud2` | RViz 可选点云显示。 |
| `/downsampled_costmap` | `nav_msgs/msg/OccupancyGrid` | RViz 可选 downsampled planner costmap 显示。 |
| `/downsampled_costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | RViz 可选 downsampled costmap 增量更新显示。 |

Nav2 的 action 接口，例如 NavigateToPose，会在运行时创建额外的 action 传输 topic。这些 topic 由 ROS 2 action 机制生成，不是工程里作为普通 topic 显式配置的内容，所以没有列入上面的清单。调试 action 传输时，可以在 launch 运行后使用 `ros2 action list` 或 `ros2 topic list` 查看。
