# all_wheel_ros

ROS2 workspace for the all-wheel robot simulation package.

## Build

Install runtime dependencies:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

From this directory:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

From the parent `ROS2` directory:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Start Gazebo simulation:

```bash
ros2 launch ros2_all_wheel_sim gazebo_sim.launch.py world:=maze2
```

Start simulation with SLAM:

```bash
ros2 launch ros2_all_wheel_sim slam_gazebo_sim.launch.py world:=maze2
```

Start simulation with Nav2:

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=maze2
```

Useful display fallback for software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
```
