# Decoupled DWA Planner

The DWA implementation is split into two layers:

- `ros2_all_wheel_sim/local_planner/dwa_core.hpp`: pure C++ planner data types and algorithm. It does not include ROS, Nav2, TF, or costmap headers.
- `ros2_all_wheel_sim/local_planner/omni_local_planner.hpp`: Nav2 controller plugin adapter. It converts ROS messages to `dwa_core` data and provides a costmap query callback.

## Build

```bash
cd /home/shexingju/work/ROS2/all_wheel_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_all_wheel_sim --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Runtime Trace From Nav2

Normal Nav2 navigation can write real DWA replay data from `OmniLocalPlanner::computeVelocityCommands()`.
The trace contains the real robot pose, current velocity, transformed local reference path, selected prediction,
sampled valid candidate trajectories, and a downsampled local costmap obstacle snapshot for each recorded frame.

Enable it in `config/nav2_params.yaml` under `controller_server.ros__parameters.FollowPath`:

```yaml
trace_enabled: true
trace_directory: "/tmp/ros2_all_wheel_dwa_trace"
trace_every_n: 1
trace_candidate_stride: 8
trace_candidate_state_stride: 2
trace_costmap_stride: 3
trace_cost_threshold: 253
```

After running navigation, replay the real trace:

```bash
cd /home/shexingju/work/ROS2/all_wheel_ros
python3 src/ros2_all_wheel_sim/scripts/visualize_dwa_trace.py \
  --world /tmp/ros2_all_wheel_dwa_trace/dwa_world.csv \
  --trace /tmp/ros2_all_wheel_dwa_trace/dwa_trace.csv
```

Save one frame without opening a GUI:

```bash
MPLBACKEND=Agg python3 src/ros2_all_wheel_sim/scripts/visualize_dwa_trace.py \
  --world /tmp/ros2_all_wheel_dwa_trace/dwa_world.csv \
  --trace /tmp/ros2_all_wheel_dwa_trace/dwa_trace.csv \
  --frame 180 \
  --save /tmp/ros2_all_wheel_dwa_trace/dwa_frame_180.png
```
