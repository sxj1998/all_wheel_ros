export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
ros2 launch ros2_all_wheel_sim gazebo_sim.launch.py \
  world:=/home/shexingju/code/ROS/ros2_all_wheel_sim-jazzy/worlds/maze2
