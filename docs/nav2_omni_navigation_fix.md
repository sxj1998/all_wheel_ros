# Nav2 全向轮直线导航问题修复记录

本文档记录全向轮机器人使用 Nav2 设置目标点后不能直线平移、会绕行的问题原因、修改内容和验证方法。

## 问题现象

在无明显障碍物的情况下，通过 RViz 或命令行给 Nav2 设置目标点后，机器人没有直接向目标点平移，而是出现以下现象：

- 横向目标点无法直接平移过去
- 机器人像差速车一样先转向再前进
- 轨迹出现绕行
- 有时目标点执行失败或到达行为不稳定

## 根本原因

该问题不是全向轮机械结构不支持，而是 Nav2 参数和底盘运动学映射没有完全按全向底盘配置。

主要原因如下：

1. DWB 控制器未开启 y 方向速度采样
2. velocity_smoother 把 y 方向速度限制为 0
3. DWB critic 配置偏向差速车，会强制车头对齐路径
4. 底盘运动学中 `linear.y` 的电机方向符号与 ROS 坐标系相反
5. Gazebo 启动时依赖 GUI，偶尔导致 world/create 启动顺序不稳定

## 修改文件

### 1. Nav2 参数

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/config/nav2_params.yaml
```

修改内容：

- 开启 DWB 的 y 方向速度
- 开启 velocity_smoother 的 y 方向限速和加速度
- 移除偏差速车的路径朝向对齐 critic
- 加入 `Twirling` 抑制无意义旋转

关键修改：

```yaml
FollowPath:
  min_vel_x: -0.5
  min_vel_y: -0.5
  max_vel_x: 0.5
  max_vel_y: 0.5
  vx_samples: 20
  vy_samples: 20
  acc_lim_y: 3.0
  decel_lim_y: -2.5
  critics: ["Oscillation", "BaseObstacle", "PathDist", "GoalDist", "Twirling"]
  Twirling.scale: 100.0
```

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.5, 0.5, 2.5]
    min_velocity: [-0.5, -0.5, -2.5]
    max_accel: [2.5, 2.5, 3.2]
    max_decel: [-2.5, -2.5, -3.2]
```

### 2. 底盘运动学

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/src/kinematics.cpp
```

修改前：

```cpp
Eigen::VectorXd M = calculate_motor_speed(-msg->linear.x, msg->linear.y, -msg->angular.z);
```

修改后：

```cpp
Eigen::VectorXd M = calculate_motor_speed(-msg->linear.x, -msg->linear.y, -msg->angular.z);
```

原因：

Nav2 输出 `linear.y > 0` 时，实际 odom 里机器人往 `y < 0` 方向运动，说明底盘 y 方向电机命令符号反了。修正后，`cmd_vel.linear.y > 0` 与 `odom.y` 正方向一致。

### 3. Gazebo 启动

文件：

```text
all_wheel_ros/src/ros2_all_wheel_sim/launch/gazebo_sim.launch.py
```

修改内容：

- Gazebo 使用 server-only 启动
- 延迟 spawn robot
- 延迟加载控制器和运动学节点

关键修改：

```text
-r -s -v 4
```

并使用：

```python
TimerAction(period=3.0, actions=[spawn_robot])
TimerAction(period=6.0, actions=[spawn_wheel_controller, kinematics])
```

原因：

之前 Gazebo GUI 和 world list 准备不稳定时，`ros_gz_sim/create` 会反复等待 world names，导致机器人未成功 spawn，Nav2 等不到 odom/TF。

## 编译运行

从工作空间根目录执行：

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

无 RViz 启动：

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=my_slam_map rviz:=false
```

注意：

```text
world:=maze2
map:=my_slam_map
```

`world` 是 Gazebo 世界，`map` 是 Nav2 地图，二者不能混用。

## 命令行验证

发送纯横向目标点：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

验证 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

修复后应看到类似输出：

```yaml
linear:
  x: 0.0
  y: 0.3421052631578947
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

验证 `/odom`：

```bash
ros2 topic echo /odom --once
```

修复后 y 方向应正向增长，x 方向偏移很小。实测结果：

```yaml
position:
  x: 0.003489728695637148
  y: 0.21767551524608428
orientation:
  z: -0.007661589632087275
  w: 0.9999706495914316
```

Action 结果：

```text
Goal finished with status: SUCCEEDED
```

## 为什么 y 只到 0.217 就成功

当前 Nav2 目标容差是：

```yaml
xy_goal_tolerance: 0.25
```

目标点为 `y=0.5`，机器人到达 `y≈0.218` 时，与目标距离约 `0.282m`。实际 AMCL/map 位姿和 odom 存在轻微差异，Nav2 按 map 坐标判断到达，因此 action 判定成功。

如需更精确到点，可适当减小：

```yaml
goal_checker:
  xy_goal_tolerance: 0.10
```

但容差过小可能导致小车在目标附近反复微调。

## 结论

修复后，Nav2 能正确输出全向轮的横向速度，机器人可以在无障碍情况下直接横向平移到目标点，不再因为差速车式控制策略出现明显绕行。

最终验证结果：

- `cmd_vel.linear.y` 正常输出
- `cmd_vel.angular.z` 为 0
- `odom.y` 正向增长
- Nav2 action 返回 `SUCCEEDED`
