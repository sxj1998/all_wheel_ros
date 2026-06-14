#include "ros2_all_wheel_sim/global_planner/astar_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace ros2_all_wheel_sim
{
namespace global_planner
{
namespace astar
{

namespace
{
constexpr unsigned char kNoInformation = 255;
constexpr unsigned char kLethalObstacle = 254;
constexpr double kCostNormalization = 252.0;
}

Planner::Planner(Config config)
: config_(config)
{
  normalizeConfig();
}

void Planner::setConfig(Config config)
{
  config_ = config;
  normalizeConfig();
}

const Config & Planner::config() const
{
  return config_;
}

/**
 * @brief 执行纯栅格 A* 全局规划。
 *
 * @param input 规划输入。
 * @return 规划结果。
 */
PlanResult Planner::plan(const PlanInput & input) const
{
  PlanResult result;
  if (!isMapValid(input.map)) {
    result.status = PlanResult::Status::kInvalidMap;
    return result;
  }

  GridCell start_cell;
  GridCell goal_cell;
  if (!worldToGrid(input.map, input.start, start_cell)) {
    result.status = PlanResult::Status::kStartOutsideMap;
    return result;
  }
  if (!worldToGrid(input.map, input.goal, goal_cell)) {
    result.status = PlanResult::Status::kGoalOutsideMap;
    return result;
  }
  if (!normalizeEndpoint(input.map, start_cell, start_cell)) {
    result.status = PlanResult::Status::kNoTraversableStart;
    return result;
  }
  if (!normalizeEndpoint(input.map, goal_cell, goal_cell)) {
    result.status = PlanResult::Status::kNoTraversableGoal;
    return result;
  }

  const unsigned int start_index = toIndex(input.map, start_cell.x, start_cell.y);
  const unsigned int goal_index = toIndex(input.map, goal_cell.x, goal_cell.y);
  const SearchResult search_result = searchPath(input.map, start_cell, goal_cell);
  if (!search_result.found) {
    result.status = PlanResult::Status::kNoPath;
    return result;
  }

  result.raw_grid_path = reconstructGridPath(input.map, search_result.parents, start_index, goal_index);
  result.simplified_grid_path = simplifyGridPath(input.map, result.raw_grid_path);
  const auto key_points = toWorldPoints(input.map, result.simplified_grid_path, input.start, input.goal);
  for (std::size_t i = 0; i + 1 < key_points.size(); ++i) {
    appendInterpolatedSegment(result.path, key_points[i], key_points[i + 1], i > 0);
  }
  result.path.push_back(input.goal);
  result.found = true;
  result.status = PlanResult::Status::kSuccess;
  return result;
}

bool Planner::isMapValid(const GridMap & map) const
{
  return map.width > 0 && map.height > 0 && map.resolution > 0.0 &&
    map.costs.size() == static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
}

bool Planner::worldToGrid(const GridMap & map, const Point2D & point, GridCell & cell) const
{
  if (point.x < map.origin_x || point.y < map.origin_y) {
    return false;
  }
  const auto mx = static_cast<int>((point.x - map.origin_x) / map.resolution);
  const auto my = static_cast<int>((point.y - map.origin_y) / map.resolution);
  if (mx < 0 || my < 0 || mx >= static_cast<int>(map.width) || my >= static_cast<int>(map.height)) {
    return false;
  }
  cell.x = static_cast<unsigned int>(mx);
  cell.y = static_cast<unsigned int>(my);
  return true;
}

Point2D Planner::gridToWorld(const GridMap & map, const GridCell & cell) const
{
  return Point2D{
    map.origin_x + (static_cast<double>(cell.x) + 0.5) * map.resolution,
    map.origin_y + (static_cast<double>(cell.y) + 0.5) * map.resolution,
    0.0};
}

bool Planner::normalizeEndpoint(
  const GridMap & map,
  const GridCell & original_cell,
  GridCell & normalized_cell) const
{
  if (isCellTraversable(map, original_cell.x, original_cell.y)) {
    normalized_cell = original_cell;
    return true;
  }
  return findNearestTraversableCell(map, original_cell.x, original_cell.y, normalized_cell);
}

Planner::SearchResult Planner::searchPath(
  const GridMap & map,
  const GridCell & start_cell,
  const GridCell & goal_cell) const
{
  struct QueueNode
  {
    double f_score{0.0};
    unsigned int index{0};

    bool operator>(const QueueNode & other) const
    {
      return f_score > other.f_score;
    }
  };

  const unsigned int cell_count = map.width * map.height;
  const unsigned int start_index = toIndex(map, start_cell.x, start_cell.y);
  const unsigned int goal_index = toIndex(map, goal_cell.x, goal_cell.y);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parents(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  g_score[start_index] = 0.0;
  parents[start_index] = static_cast<int>(start_index);
  open.push(QueueNode{heuristic(start_cell.x, start_cell.y, goal_cell.x, goal_cell.y), start_index});

  while (!open.empty()) {
    const QueueNode queue_node = open.top();
    open.pop();
    const unsigned int current_index = queue_node.index;
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;
    if (current_index == goal_index) {
      break;
    }

    const GridCell current_cell{current_index % map.width, current_index / map.width};
    for (const auto & offset : neighborOffsets()) {
      const int nx_i = static_cast<int>(current_cell.x) + offset.dx;
      const int ny_i = static_cast<int>(current_cell.y) + offset.dy;
      if (nx_i < 0 || ny_i < 0 ||
        nx_i >= static_cast<int>(map.width) || ny_i >= static_cast<int>(map.height))
      {
        continue;
      }

      const GridCell neighbor_cell{
        static_cast<unsigned int>(nx_i),
        static_cast<unsigned int>(ny_i)};
      if (!isCellTraversable(map, neighbor_cell.x, neighbor_cell.y)) {
        continue;
      }

      const unsigned int neighbor_index = toIndex(map, neighbor_cell.x, neighbor_cell.y);
      if (closed[neighbor_index]) {
        continue;
      }

      const double tentative_g =
        g_score[current_index] + traversalCost(map, current_cell, neighbor_cell);
      if (tentative_g < g_score[neighbor_index]) {
        parents[neighbor_index] = static_cast<int>(current_index);
        g_score[neighbor_index] = tentative_g;
        const double h_score = heuristic(neighbor_cell.x, neighbor_cell.y, goal_cell.x, goal_cell.y);
        open.push(QueueNode{tentative_g + h_score, neighbor_index});
      }
    }
  }

  return SearchResult{closed[goal_index], std::move(parents)};
}

const std::vector<Planner::GridOffset> & Planner::neighborOffsets() const
{
  static const std::vector<GridOffset> kFourConnectedOffsets{
    {1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  static const std::vector<GridOffset> kEightConnectedOffsets{
    {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

  return config_.use_8_connected ? kEightConnectedOffsets : kFourConnectedOffsets;
}

double Planner::traversalCost(
  const GridMap & map,
  const GridCell & current_cell,
  const GridCell & neighbor_cell) const
{
  const double step = std::hypot(
    static_cast<double>(neighbor_cell.x) - static_cast<double>(current_cell.x),
    static_cast<double>(neighbor_cell.y) - static_cast<double>(current_cell.y));
  const unsigned char cost = costAt(map, neighbor_cell.x, neighbor_cell.y);
  const double normalized_cost =
    cost == kNoInformation ? 0.0 : static_cast<double>(cost) / kCostNormalization;
  return step * (1.0 + config_.cost_penalty * normalized_cost);
}

std::vector<GridCell> Planner::reconstructGridPath(
  const GridMap & map,
  const std::vector<int> & parents,
  unsigned int start_index,
  unsigned int goal_index) const
{
  std::vector<GridCell> grid_path;
  unsigned int current = goal_index;
  while (current != start_index) {
    grid_path.push_back(GridCell{current % map.width, current / map.width});
    current = static_cast<unsigned int>(parents[current]);
  }
  grid_path.push_back(GridCell{start_index % map.width, start_index / map.width});
  std::reverse(grid_path.begin(), grid_path.end());
  return grid_path;
}

std::vector<GridCell> Planner::simplifyGridPath(
  const GridMap & map,
  const std::vector<GridCell> & grid_path) const
{
  if (grid_path.size() <= 2) {
    return grid_path;
  }

  std::vector<GridCell> simplified;
  simplified.push_back(grid_path.front());
  std::size_t anchor = 0;
  while (anchor < grid_path.size() - 1) {
    std::size_t next = grid_path.size() - 1;
    while (next > anchor + 1 && !hasLineOfSight(map, grid_path[anchor], grid_path[next])) {
      --next;
    }
    simplified.push_back(grid_path[next]);
    anchor = next;
  }
  return simplified;
}

std::vector<Point2D> Planner::toWorldPoints(
  const GridMap & map,
  const std::vector<GridCell> & grid_path,
  const Point2D & start,
  const Point2D & goal) const
{
  std::vector<Point2D> world_points;
  world_points.reserve(grid_path.size());
  world_points.push_back(start);
  for (std::size_t i = 1; i + 1 < grid_path.size(); ++i) {
    world_points.push_back(gridToWorld(map, grid_path[i]));
  }
  world_points.push_back(goal);
  return world_points;
}

void Planner::appendInterpolatedSegment(
  std::vector<Point2D> & path,
  const Point2D & from,
  const Point2D & to,
  bool skip_first_point) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / config_.interpolation_resolution)));
  for (int step = 0; step < steps; ++step) {
    if (skip_first_point && step == 0) {
      continue;
    }
    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    path.push_back(Point2D{from.x + ratio * dx, from.y + ratio * dy, yaw});
  }
}

bool Planner::isCellTraversable(const GridMap & map, unsigned int x, unsigned int y) const
{
  const unsigned char cost = costAt(map, x, y);
  if (cost == kNoInformation) {
    return config_.allow_unknown;
  }
  return cost < config_.obstacle_threshold;
}

bool Planner::findNearestTraversableCell(
  const GridMap & map,
  unsigned int seed_x,
  unsigned int seed_y,
  GridCell & cell) const
{
  const int max_radius = static_cast<int>(std::ceil(config_.tolerance / map.resolution));
  const int size_x = static_cast<int>(map.width);
  const int size_y = static_cast<int>(map.height);
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
        if (std::hypot(dx, dy) * map.resolution > config_.tolerance) {
          continue;
        }
        if (isCellTraversable(map, static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
          cell.x = static_cast<unsigned int>(nx);
          cell.y = static_cast<unsigned int>(ny);
          return true;
        }
      }
    }
  }
  return false;
}

double Planner::heuristic(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) const
{
  return std::hypot(
    static_cast<double>(goal_x) - static_cast<double>(x),
    static_cast<double>(goal_y) - static_cast<double>(y));
}

bool Planner::hasLineOfSight(const GridMap & map, const GridCell & from, const GridCell & to) const
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
    if (!isCellTraversable(map, static_cast<unsigned int>(x0), static_cast<unsigned int>(y0))) {
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

unsigned int Planner::toIndex(const GridMap & map, unsigned int x, unsigned int y) const
{
  return y * map.width + x;
}

unsigned char Planner::costAt(const GridMap & map, unsigned int x, unsigned int y) const
{
  return map.costs[toIndex(map, x, y)];
}

void Planner::normalizeConfig()
{
  config_.tolerance = std::max(0.0, config_.tolerance);
  config_.cost_penalty = std::max(0.0, config_.cost_penalty);
  config_.interpolation_resolution = std::max(1e-3, config_.interpolation_resolution);
  config_.obstacle_threshold = std::clamp(
    config_.obstacle_threshold, static_cast<unsigned char>(1), kLethalObstacle);
}

}  // namespace astar
}  // namespace global_planner
}  // namespace ros2_all_wheel_sim
