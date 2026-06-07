# Navigation And Camera Direction Notes

本文档记录小车摄像头朝向、Nav2 目标位姿朝向、以及导航运动方式的相关修改。

## 目标

期望行为：

- 两个摄像头正对方向为小车前方。
- RViz 中设置 `Nav2 Goal` 时，目标箭头方向应作为最终车头/摄像头朝向。
- 小车导航时不再优先横移，而是先原地自旋对准路径方向，再沿车头方向向前行驶。

## 修改 1：摄像头安装方向

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/urdf/all_wheel/ALL_WHEEL.urdf
```

修改位置：

- `stereo_left_joint`
- `stereo_right_joint`

修改前：

```xml
xyz="-0.05 0.05 0.06"
rpy="0 0 3.1415926"

xyz="-0.05 -0.05 0.06"
rpy="0 0 3.1415926"
```

问题：

- 摄像头安装在车体后侧。
- yaw 为 `3.1415926`，相当于绕 Z 轴旋转 180 度，摄像头朝向与车体前方相反。

修改后：

```xml
xyz="0.08 0.05 0.06"
rpy="0 0 0"

xyz="0.08 -0.05 0.06"
rpy="0 0 0"
```

效果：

- 左右摄像头位于车体前方。
- 摄像头局部正方向与 `base_link` 前方一致。

验证结果：

```text
base_link -> stereo_left_link:
  Translation: [0.080, 0.050, 0.060]
  RPY: [0.000, 0.000, 0.000]

base_link -> stereo_right_link:
  Translation: [0.080, -0.050, 0.060]
  RPY: [0.000, 0.000, 0.000]
```

摄像头话题正常：

```text
/camera
/camera_info
/camera_right
/camera_right/camera_info
```

## 修改 2：目标位姿方向对齐

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/config/nav2_params.yaml
```

问题：

原先小车到达目标位置后，摄像头没有严格指向 RViz 中设置的目标箭头方向。

原因：

- 之前控制器更关注到达目标点位置。
- yaw 容差较大时，小车可能在角度误差仍然明显时就判定到达。

当前配置：

```yaml
goal_checker:
  stateful: true
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 0.05
```

效果：

- 位置允许 0.25 m 容差，适合仿真地图和墙角目标。
- 朝向容差约为 0.05 rad，约 2.9 度。
- 到点后会继续调整车头方向，使摄像头接近目标箭头方向。

验证结果：

目标朝向 90 度：

```text
Goal finished with status: SUCCEEDED
final yaw ~= 88.2 deg
```

目标朝向 0 度：

```text
Goal finished with status: SUCCEEDED
final yaw ~= 2.14 deg
```

## 修改 3：从横移优先改成先自旋再直走

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/config/nav2_params.yaml
```

问题：

全向轮具备横移能力，DWB 控制器在某些目标下会选择先横着走，再到点旋转。用户期望更像普通前进车：

```text
先自旋对准方向 -> 再沿车头方向直走 -> 到点后对齐最终目标方向
```

解决方案：

将 `FollowPath` 控制器从 DWB 切换为 Nav2 的 Regulated Pure Pursuit Controller。

当前配置：

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.45
  lookahead_dist: 0.5
  min_lookahead_dist: 0.25
  max_lookahead_dist: 0.8
  lookahead_time: 1.5
  transform_tolerance: 0.2
  use_velocity_scaled_lookahead_dist: false
  min_approach_linear_velocity: 0.05
  approach_velocity_scaling_dist: 0.6
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: false
  regulated_linear_scaling_min_radius: 0.6
  regulated_linear_scaling_min_speed: 0.08
  use_rotate_to_heading: true
  rotate_to_heading_min_angle: 0.15
  rotate_to_heading_angular_vel: 0.8
  max_angular_accel: 2.5
  allow_reversing: false
  use_interpolation: true
```

关键参数说明：

```yaml
use_rotate_to_heading: true
```

启用先旋转到路径方向。

```yaml
allow_reversing: false
```

禁止倒车，减少反向走或姿态奇怪的路径跟踪行为。

```yaml
desired_linear_vel: 0.45
```

正常前进行驶速度。

```yaml
rotate_to_heading_angular_vel: 0.8
```

原地旋转时的目标角速度。

效果：

- RPP 控制器不会使用 `linear.y` 横移。
- 初始朝向不对时，先输出角速度原地自旋。
- 朝向接近路径方向后，再输出 `linear.x` 前进。

验证速度序列：

第一阶段，纯自旋：

```yaml
linear:
  x: 0.0
  y: 0.0
angular:
  z: 0.8
```

第二阶段，向前直走：

```yaml
linear:
  x: 0.45
  y: 0.0
angular:
  z: small correction
```

验证结果：

```text
Goal finished with status: SUCCEEDED
```

## 运行方式

编译：

```bash
cd ~/work/ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动导航仿真：

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=my_slam_map
```

无界面验证：

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=my_slam_map rviz:=false headless:=true
```

## 注意事项

- 当前运动策略刻意限制横移，优先满足“先自旋再直走”的视觉效果。
- 机器人仍是全向轮底盘，但 Nav2 局部控制器不会主动输出横向速度。
- 如果后续又希望在狭窄环境中使用横移能力，可以切回 DWB，并恢复 `linear.y` 相关参数。
- 在 RViz 中设置目标时，箭头方向就是最终摄像头指向方向。
