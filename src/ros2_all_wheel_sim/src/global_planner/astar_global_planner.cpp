#include "ros2_all_wheel_sim/global_planner/astar_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_all_wheel_sim
{
namespace global_planner
{

void AStarGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in AStarGlobalPlanner::configure");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_8_connected", rclcpp::ParameterValue(use_8_connected_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_penalty", rclcpp::ParameterValue(cost_penalty_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(interpolation_resolution_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_threshold",
    rclcpp::ParameterValue(static_cast<int>(obstacle_threshold_)));

  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".use_8_connected", use_8_connected_);
  node->get_parameter(name_ + ".cost_penalty", cost_penalty_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  int obstacle_threshold = obstacle_threshold_;
  node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold);
  obstacle_threshold_ = static_cast<unsigned char>(
    std::clamp(obstacle_threshold, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  interpolation_resolution_ = std::max(interpolation_resolution_, costmap_->getResolution());

  RCLCPP_INFO(
    logger_,
    "Configured %s with frame=%s tolerance=%.2f allow_unknown=%s use_8_connected=%s",
    name_.c_str(), global_frame_.c_str(), tolerance_, allow_unknown_ ? "true" : "false",
    use_8_connected_ ? "true" : "false");
}

void AStarGlobalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

void AStarGlobalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void AStarGlobalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path AStarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = clock_->now();
  empty_path.header.frame_id = global_frame_;

  if (costmap_ == nullptr) {
    RCLCPP_ERROR(logger_, "Costmap is not available");
    return empty_path;
  }

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    RCLCPP_WARN(logger_, "Start pose is outside the global costmap");
    return empty_path;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal pose is outside the global costmap");
    return empty_path;
  }

  GridCell start_cell{start_x, start_y};
  GridCell goal_cell{goal_x, goal_y};
  if (!isCellTraversable(start_x, start_y) &&
    !findNearestTraversableCell(start_x, start_y, tolerance_, start_cell))
  {
    RCLCPP_WARN(logger_, "No traversable start cell found within %.2f m", tolerance_);
    return empty_path;
  }
  if (!isCellTraversable(goal_x, goal_y) &&
    !findNearestTraversableCell(goal_x, goal_y, tolerance_, goal_cell))
  {
    RCLCPP_WARN(logger_, "No traversable goal cell found within %.2f m", tolerance_);
    return empty_path;
  }

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int cell_count = size_x * size_y;
  const unsigned int start_index = toIndex(start_cell.x, start_cell.y);
  const unsigned int goal_index = toIndex(goal_cell.x, goal_cell.y);

  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parents(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  using QueueEntry = std::pair<double, unsigned int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  g_score[start_index] = 0.0;
  parents[start_index] = static_cast<int>(start_index);
  open.emplace(heuristic(start_cell.x, start_cell.y, goal_cell.x, goal_cell.y), start_index);

  const int dx4[4] = {1, 0, -1, 0};
  const int dy4[4] = {0, 1, 0, -1};
  const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const int neighbor_count = use_8_connected_ ? 8 : 4;

  bool found = false;
  while (!open.empty()) {
    const auto [unused_f_score, current] = open.top();
    (void)unused_f_score;
    open.pop();

    if (closed[current]) {
      continue;
    }
    closed[current] = true;

    if (current == goal_index) {
      found = true;
      break;
    }

    const unsigned int cx = current % size_x;
    const unsigned int cy = current / size_x;
    for (int i = 0; i < neighbor_count; ++i) {
      const int nx_i = static_cast<int>(cx) + (use_8_connected_ ? dx8[i] : dx4[i]);
      const int ny_i = static_cast<int>(cy) + (use_8_connected_ ? dy8[i] : dy4[i]);
      if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<int>(size_x) ||
        ny_i >= static_cast<int>(size_y))
      {
        continue;
      }

      const unsigned int nx = static_cast<unsigned int>(nx_i);
      const unsigned int ny = static_cast<unsigned int>(ny_i);
      if (!isCellTraversable(nx, ny)) {
        continue;
      }

      const unsigned int neighbor_index = toIndex(nx, ny);
      if (closed[neighbor_index]) {
        continue;
      }

      const double step = std::hypot(
        static_cast<double>(nx_i) - static_cast<double>(cx),
        static_cast<double>(ny_i) - static_cast<double>(cy));
      const unsigned char cost = costmap_->getCost(nx, ny);
      const double normalized_cost =
        cost == nav2_costmap_2d::NO_INFORMATION ? 0.0 : static_cast<double>(cost) / 252.0;
      const double tentative_g = g_score[current] + step * (1.0 + cost_penalty_ * normalized_cost);

      if (tentative_g < g_score[neighbor_index]) {
        parents[neighbor_index] = static_cast<int>(current);
        g_score[neighbor_index] = tentative_g;
        const double f_score = tentative_g + heuristic(nx, ny, goal_cell.x, goal_cell.y);
        open.emplace(f_score, neighbor_index);
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* could not find a path to the goal");
    return empty_path;
  }

  auto path = buildPath(parents, start_index, goal_index, start, goal);
  RCLCPP_DEBUG(logger_, "A* generated path with %zu poses", path.poses.size());
  return path;
}

unsigned int AStarGlobalPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return y * costmap_->getSizeInCellsX() + x;
}

bool AStarGlobalPlanner::isCellTraversable(unsigned int x, unsigned int y) const
{
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < obstacle_threshold_;
}

bool AStarGlobalPlanner::findNearestTraversableCell(
  unsigned int seed_x,
  unsigned int seed_y,
  double tolerance,
  GridCell & cell) const
{
  const int max_radius = static_cast<int>(std::ceil(tolerance / costmap_->getResolution()));
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const int sx = static_cast<int>(seed_x);
  const int sy = static_cast<int>(seed_y);

  for (int radius = 0; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }

        const int nx = sx + dx;
        const int ny = sy + dy;
        if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y) {
          continue;
        }
        if (std::hypot(dx, dy) * costmap_->getResolution() > tolerance) {
          continue;
        }
        if (isCellTraversable(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
          cell.x = static_cast<unsigned int>(nx);
          cell.y = static_cast<unsigned int>(ny);
          return true;
        }
      }
    }
  }

  return false;
}

double AStarGlobalPlanner::heuristic(
  unsigned int x,
  unsigned int y,
  unsigned int goal_x,
  unsigned int goal_y) const
{
  return std::hypot(
    static_cast<double>(goal_x) - static_cast<double>(x),
    static_cast<double>(goal_y) - static_cast<double>(y));
}

bool AStarGlobalPlanner::hasLineOfSight(const GridCell & from, const GridCell & to) const
{
  int x0 = static_cast<int>(from.x);
  int y0 = static_cast<int>(from.y);
  const int x1 = static_cast<int>(to.x);
  const int y1 = static_cast<int>(to.y);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (!isCellTraversable(static_cast<unsigned int>(x0), static_cast<unsigned int>(y0))) {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }

    const int err2 = 2 * err;
    if (err2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (err2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

std::vector<AStarGlobalPlanner::GridCell> AStarGlobalPlanner::simplifyGridPath(
  const std::vector<GridCell> & grid_path) const
{
  if (grid_path.size() <= 2) {
    return grid_path;
  }

  std::vector<GridCell> simplified;
  simplified.push_back(grid_path.front());

  size_t anchor = 0;
  while (anchor < grid_path.size() - 1) {
    size_t next = grid_path.size() - 1;
    while (next > anchor + 1 && !hasLineOfSight(grid_path[anchor], grid_path[next])) {
      --next;
    }

    simplified.push_back(grid_path[next]);
    anchor = next;
  }

  return simplified;
}

nav_msgs::msg::Path AStarGlobalPlanner::buildPath(
  const std::vector<int> & parents,
  unsigned int start_index,
  unsigned int goal_index,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  std::vector<GridCell> grid_path;
  unsigned int current = goal_index;
  const unsigned int size_x = costmap_->getSizeInCellsX();
  while (current != start_index) {
    grid_path.push_back(GridCell{current % size_x, current / size_x});
    current = static_cast<unsigned int>(parents[current]);
  }
  grid_path.push_back(GridCell{start_index % size_x, start_index / size_x});
  std::reverse(grid_path.begin(), grid_path.end());
  const auto simplified_path = simplifyGridPath(grid_path);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  auto add_pose = [this, &path](double x, double y, double yaw) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(pose);
    };

  std::vector<std::pair<double, double>> world_points;
  world_points.emplace_back(start.pose.position.x, start.pose.position.y);
  for (size_t i = 1; i + 1 < simplified_path.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(simplified_path[i].x, simplified_path[i].y, wx, wy);
    world_points.emplace_back(wx, wy);
  }
  world_points.emplace_back(goal.pose.position.x, goal.pose.position.y);

  for (size_t i = 0; i + 1 < world_points.size(); ++i) {
    const auto [x0, y0] = world_points[i];
    const auto [x1, y1] = world_points[i + 1];
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double distance = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / interpolation_resolution_)));

    for (int step = 0; step < steps; ++step) {
      if (i > 0 && step == 0) {
        continue;
      }
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      add_pose(x0 + ratio * dx, y0 + ratio * dy, yaw);
    }
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);

  return path;
}

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

PLUGINLIB_EXPORT_CLASS(
  ros2_all_wheel_sim::global_planner::AStarGlobalPlanner,
  nav2_core::GlobalPlanner)
