#!/usr/bin/env bash
set -euo pipefail

show_help() {
  cat <<'EOF'
Usage: clean_ros_processes.sh [--dry-run] [--keep-daemon]

Clean ROS/Gazebo/RViz processes started by the all-wheel simulation stack.

Options:
  --dry-run      Print matched processes without killing them.
  --keep-daemon  Do not stop the ROS 2 daemon after process cleanup.
  -h, --help     Show this help.
EOF
}

dry_run=false
stop_daemon=true

for arg in "$@"; do
  case "$arg" in
    --dry-run)
      dry_run=true
      ;;
    --keep-daemon)
      stop_daemon=false
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      show_help >&2
      exit 2
      ;;
  esac
done

patterns=(
  '/opt/ros/.*/bin/ros2 launch ros2_all_wheel_sim'
  'ros2 launch ros2_all_wheel_sim'
  'ruby /usr/bin/ign gazebo'
  '(^| )ign gazebo( |$)'
  '/opt/ros/.*/lib/rviz2/rviz2'
  '/opt/ros/.*/lib/ros_gz_bridge/parameter_bridge'
  '/opt/ros/.*/lib/ros_gz_sim/create'
  '/opt/ros/.*/lib/controller_manager/spawner'
  '/opt/ros/.*/lib/slam_toolbox/sync_slam_toolbox_node'
  '/opt/ros/.*/lib/nav2_controller/controller_server'
  '/opt/ros/.*/lib/nav2_planner/planner_server'
  '/opt/ros/.*/lib/nav2_bt_navigator/bt_navigator'
  '/opt/ros/.*/lib/nav2_smoother/smoother_server'
  '/opt/ros/.*/lib/nav2_behaviors/behavior_server'
  '/opt/ros/.*/lib/nav2_waypoint_follower/waypoint_follower'
  '/opt/ros/.*/lib/nav2_velocity_smoother/velocity_smoother'
  '/opt/ros/.*/lib/nav2_map_server/map_saver_server'
  '/opt/ros/.*/lib/nav2_lifecycle_manager/lifecycle_manager'
  '/opt/ros/.*/lib/robot_state_publisher/robot_state_publisher'
  '/opt/ros/.*/lib/tf2_ros/static_transform_publisher'
  '/ros2_all_wheel_sim/kinematics'
  '/ros2_all_wheel_sim/frontier_explorer'
)

collect_pids() {
  local pattern
  for pattern in "${patterns[@]}"; do
    pgrep -af "$pattern" 2>/dev/null || true
  done |
    awk -v self="$$" -v parent="${PPID:-}" '
      $1 != self && $1 != parent && !seen[$1]++ { print }
    '
}

matched="$(collect_pids)"

if [[ -z "$matched" ]]; then
  echo "No matching ROS/Gazebo/RViz processes found."
else
  echo "Matched processes:"
  echo "$matched"

  if [[ "$dry_run" == true ]]; then
    echo "Dry run only; no processes were killed."
  else
    pids="$(awk '{ print $1 }' <<<"$matched")"
    echo "Sending SIGTERM..."
    # shellcheck disable=SC2086
    kill -TERM $pids 2>/dev/null || true
    sleep 3

    remaining="$(collect_pids)"
    if [[ -n "$remaining" ]]; then
      echo "Sending SIGKILL to remaining processes:"
      echo "$remaining"
      # shellcheck disable=SC2046
      kill -KILL $(awk '{ print $1 }' <<<"$remaining") 2>/dev/null || true
    fi
  fi
fi

if [[ "$dry_run" == false && "$stop_daemon" == true ]]; then
  if command -v ros2 >/dev/null 2>&1; then
    echo "Stopping ROS 2 daemon..."
    ros2 daemon stop >/dev/null 2>&1 || true
  else
    echo "ros2 command not found; skipped ROS 2 daemon cleanup."
  fi
fi

echo "Cleanup complete."
