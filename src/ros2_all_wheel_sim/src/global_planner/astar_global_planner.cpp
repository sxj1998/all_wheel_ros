#include "ros2_all_wheel_sim/global_planner/astar_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#ifndef ASTAR_TRACE_POINTS
#define ASTAR_TRACE_POINTS 0
#endif

#if ASTAR_TRACE_POINTS
#define ASTAR_TRACE(logger, ...) RCLCPP_INFO(logger, __VA_ARGS__)
#else
#define ASTAR_TRACE(logger, ...) \
  do { \
  } while (0)
#endif

namespace ros2_all_wheel_sim
{
namespace global_planner
{

/**
 * @brief 配置 A* 全局规划器插件。
 *
 * 该函数由 Nav2 planner_server 在 lifecycle configure 阶段调用，负责保存 node、TF、
 * costmap 等运行上下文，并从参数服务器读取 A* 规划相关参数。完成后插件即可被
 * planner_server 调用执行全局路径规划。
 *
 * @param parent Nav2 lifecycle node 弱引用。
 * @param name 插件实例名，当前通常为 GridBased。
 * @param tf TF buffer，保留给接口和后续坐标转换扩展使用。
 * @param costmap_ros Nav2 全局代价地图封装对象。
 */
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

  // 参数服务器以 int 保存阈值，读取后再裁剪到 costmap 合法 cost 范围。
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
  ASTAR_TRACE(
    logger_,
    "[AStarTrace] trace enabled: cost_penalty=%.3f interpolation_resolution=%.3f "
    "obstacle_threshold=%u",
    cost_penalty_, interpolation_resolution_, static_cast<unsigned int>(obstacle_threshold_));
}

/**
 * @brief 清理规划器资源。
 *
 * 当前实现没有额外动态资源需要释放，函数保留用于符合 Nav2 lifecycle 插件接口。
 */
void AStarGlobalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

/**
 * @brief 激活规划器。
 *
 * 当前实现没有 publisher、timer 等需要单独激活的对象，仅记录生命周期状态。
 */
void AStarGlobalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

/**
 * @brief 停用规划器。
 *
 * 当前实现没有 publisher、timer 等需要单独停用的对象，仅记录生命周期状态。
 */
void AStarGlobalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

/**
 * @brief 根据起点和目标点创建全局路径。
 *
 * 该函数是 Nav2 GlobalPlanner 接口的核心入口。它只负责组织主流程：
 * 坐标转换、端点可通行修正、A* 搜索、路径回溯与后处理。具体算法细节下沉到
 * searchPath()、buildPath() 等私有函数中，保持主入口简洁。
 *
 * @param start 当前机器人起点位姿，通常位于 map 坐标系。
 * @param goal 用户在 RViz 或 action 中指定的目标位姿。
 * @return nav_msgs::msg::Path 可供 Nav2 controller 跟踪的全局路径；失败时返回空路径。
 */
nav_msgs::msg::Path AStarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto empty_path = makeEmptyPath();
  ASTAR_TRACE(
    logger_,
    "[AStarTrace] createPlan start_world=(%.3f, %.3f) goal_world=(%.3f, %.3f) frame=%s",
    start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y,
    global_frame_.c_str());

  if (costmap_ == nullptr) {
    RCLCPP_ERROR(logger_, "Costmap is not available");
    return empty_path;
  }

  GridCell start_cell{0, 0};
  GridCell goal_cell{0, 0};
  if (!poseToGridCell(start, start_cell)) {
    RCLCPP_WARN(logger_, "Start pose is outside the global costmap");
    return empty_path;
  }
  if (!poseToGridCell(goal, goal_cell)) {
    RCLCPP_WARN(logger_, "Goal pose is outside the global costmap");
    return empty_path;
  }
  traceGridCell("raw_start_cell", start_cell);
  traceGridCell("raw_goal_cell", goal_cell);

  if (!normalizeEndpoint(start_cell, start_cell)) {
    RCLCPP_WARN(logger_, "No traversable start cell found within %.2f m", tolerance_);
    return empty_path;
  }
  if (!normalizeEndpoint(goal_cell, goal_cell)) {
    RCLCPP_WARN(logger_, "No traversable goal cell found within %.2f m", tolerance_);
    return empty_path;
  }
  traceGridCell("normalized_start_cell", start_cell);
  traceGridCell("normalized_goal_cell", goal_cell);

  const unsigned int start_index = toIndex(start_cell.x, start_cell.y);
  const unsigned int goal_index = toIndex(goal_cell.x, goal_cell.y);
  ASTAR_TRACE(
    logger_, "[AStarTrace] start_index=%u goal_index=%u", start_index, goal_index);
  const auto search_result = searchPath(start_cell, goal_cell);

  if (!search_result.found) {
    RCLCPP_WARN(logger_, "A* could not find a path to the goal");
    return empty_path;
  }

  auto path = buildPath(search_result.parents, start_index, goal_index, start, goal);
  RCLCPP_DEBUG(logger_, "A* generated path with %zu poses", path.poses.size());
  return path;
}

/**
 * @brief 创建带时间戳和全局坐标系的空路径。
 *
 * 统一封装空路径创建逻辑，保证成功和失败返回的 Path header 一致。
 *
 * @return nav_msgs::msg::Path 已设置 header 的空路径。
 */
nav_msgs::msg::Path AStarGlobalPlanner::makeEmptyPath() const
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  return path;
}

/**
 * @brief 将世界坐标位姿转换为 costmap 栅格坐标。
 *
 * A* 搜索在离散栅格地图上执行，因此需要先把 map 坐标系下的米制坐标转换为
 * costmap 中的 cell 坐标。
 *
 * @param pose 输入位姿。
 * @param cell 输出栅格坐标。
 * @return true 转换成功；false 表示位姿在 costmap 范围外。
 */
bool AStarGlobalPlanner::poseToGridCell(
  const geometry_msgs::msg::PoseStamped & pose,
  GridCell & cell) const
{
  return costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, cell.x, cell.y);
}

/**
 * @brief 归一化规划端点，确保端点落在可通行栅格上。
 *
 * 如果原始端点可通行，则直接使用；如果不可通行，则在 tolerance_ 半径内寻找最近
 * 可通行栅格。该逻辑用于处理用户目标点点到墙边、膨胀层或未知区域的情况。
 *
 * @param original_cell 原始端点栅格。
 * @param normalized_cell 输出可通行端点栅格。
 * @return true 找到可用端点；false 表示端点附近没有可通行栅格。
 */
bool AStarGlobalPlanner::normalizeEndpoint(
  const GridCell & original_cell,
  GridCell & normalized_cell) const
{
  if (isCellTraversable(original_cell.x, original_cell.y)) {
    normalized_cell = original_cell;
    ASTAR_TRACE(
      logger_, "[AStarTrace] endpoint already traversable cell=(%u,%u)",
      original_cell.x, original_cell.y);
    return true;
  }

  const bool found = findNearestTraversableCell(
    original_cell.x, original_cell.y, tolerance_, normalized_cell);
  ASTAR_TRACE(
    logger_, "[AStarTrace] endpoint normalized original=(%u,%u) found=%s normalized=(%u,%u)",
    original_cell.x, original_cell.y, found ? "true" : "false",
    normalized_cell.x, normalized_cell.y);
  return found;
}

/**
 * @brief 在 costmap 上执行 A* 搜索。
 *
 * 函数维护开放列表 open、关闭列表 closed、累计代价 g_score 和父节点 parents。
 * 每次从 open 中取 f_score 最小的栅格扩展，直到到达目标或 open 为空。
 *
 * @param start_cell 搜索起点栅格。
 * @param goal_cell 搜索终点栅格。
 * @return SearchResult found 表示是否成功，parents 用于后续路径回溯。
 */
AStarGlobalPlanner::SearchResult AStarGlobalPlanner::searchPath(
  const GridCell & start_cell,
  const GridCell & goal_cell) const
{
  // 搜索状态按 costmap 一维索引存储，避免在循环中反复分配复杂对象。
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int cell_count = size_x * size_y;
  const unsigned int start_index = toIndex(start_cell.x, start_cell.y);
  const unsigned int goal_index = toIndex(goal_cell.x, goal_cell.y);

  // g_score 记录从起点到每个栅格的当前最小已知代价。
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());

  // parents 用于搜索成功后从目标点回溯完整路径。
  std::vector<int> parents(cell_count, -1);

  // closed 标记已经完成扩展的栅格，防止重复处理。
  std::vector<bool> closed(cell_count, false);

  // open 是 A* 的开放列表，始终优先扩展 f_score 最小的节点。
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  g_score[start_index] = 0.0;
  parents[start_index] = static_cast<int>(start_index);
  open.push(QueueNode{heuristic(start_cell.x, start_cell.y, goal_cell.x, goal_cell.y), start_index});
  ASTAR_TRACE(
    logger_, "[AStarTrace] search begin size=(%u,%u) cells=%u start=(%u,%u) goal=(%u,%u)",
    size_x, size_y, cell_count, start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);

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

    const GridCell current_cell{current_index % size_x, current_index / size_x};
    ASTAR_TRACE(
      logger_, "[AStarTrace] expand cell=(%u,%u) index=%u f=%.3f g=%.3f open_size=%zu",
      current_cell.x, current_cell.y, current_index, queue_node.f_score,
      g_score[current_index], open.size());

    for (const auto & offset : neighborOffsets()) {
      const int nx_i = static_cast<int>(current_cell.x) + offset.dx;
      const int ny_i = static_cast<int>(current_cell.y) + offset.dy;
      if (nx_i < 0 || ny_i < 0 ||
        nx_i >= static_cast<int>(size_x) || ny_i >= static_cast<int>(size_y))
      {
        continue;
      }

      const GridCell neighbor_cell{
        static_cast<unsigned int>(nx_i),
        static_cast<unsigned int>(ny_i)};
      if (!isCellTraversable(neighbor_cell.x, neighbor_cell.y)) {
        continue;
      }

      const unsigned int neighbor_index = toIndex(neighbor_cell.x, neighbor_cell.y);
      if (closed[neighbor_index]) {
        continue;
      }

      const double tentative_g =
        g_score[current_index] + traversalCost(current_cell, neighbor_cell);

      if (tentative_g < g_score[neighbor_index]) {
        parents[neighbor_index] = static_cast<int>(current_index);
        g_score[neighbor_index] = tentative_g;
        const double h_score = heuristic(neighbor_cell.x, neighbor_cell.y, goal_cell.x, goal_cell.y);
        const double f_score =
          tentative_g + h_score;
        ASTAR_TRACE(
          logger_,
          "[AStarTrace] update neighbor=(%u,%u) parent=(%u,%u) index=%u "
          "cost=%u g=%.3f h=%.3f f=%.3f",
          neighbor_cell.x, neighbor_cell.y, current_cell.x, current_cell.y, neighbor_index,
          static_cast<unsigned int>(costmap_->getCost(neighbor_cell.x, neighbor_cell.y)),
          tentative_g, h_score, f_score);
        open.push(QueueNode{f_score, neighbor_index});
      }
    }
  }

  SearchResult result;
  result.found = closed[goal_index];
  result.parents = std::move(parents);
  ASTAR_TRACE(
    logger_, "[AStarTrace] search end found=%s goal_index=%u",
    result.found ? "true" : "false", goal_index);
  return result;
}

/**
 * @brief 获取邻接搜索方向。
 *
 * 根据 use_8_connected_ 参数返回 4 邻接或 8 邻接偏移表。偏移表使用静态常量，
 * 避免每次规划重复分配。
 *
 * @return 当前配置对应的邻接方向数组引用。
 */
const std::vector<AStarGlobalPlanner::GridOffset> & AStarGlobalPlanner::neighborOffsets() const
{
  static const std::vector<GridOffset> kFourConnectedOffsets{
    {1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  static const std::vector<GridOffset> kEightConnectedOffsets{
    {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

  return use_8_connected_ ? kEightConnectedOffsets : kFourConnectedOffsets;
}

/**
 * @brief 计算两个相邻栅格之间的移动代价。
 *
 * 移动代价由几何步长和 costmap 代价共同决定。直行步长约为 1，斜向步长约为
 * 1.414；cost 越高，惩罚越大，路径越倾向远离障碍物和膨胀区。
 *
 * @param current_cell 当前栅格。
 * @param neighbor_cell 候选相邻栅格。
 * @return double 从当前栅格移动到相邻栅格的代价。
 */
double AStarGlobalPlanner::traversalCost(
  const GridCell & current_cell,
  const GridCell & neighbor_cell) const
{
  // step 是栅格移动距离：直行约 1.0，斜向约 1.414。
  const double step = std::hypot(
    static_cast<double>(neighbor_cell.x) - static_cast<double>(current_cell.x),
    static_cast<double>(neighbor_cell.y) - static_cast<double>(current_cell.y));

  // cost 越高越靠近障碍，惩罚项会让路径主动远离膨胀区。
  const unsigned char cost = costmap_->getCost(neighbor_cell.x, neighbor_cell.y);
  const double normalized_cost =
    cost == nav2_costmap_2d::NO_INFORMATION ? 0.0 : static_cast<double>(cost) / 252.0;

  return step * (1.0 + cost_penalty_ * normalized_cost);
}

/**
 * @brief 将二维栅格坐标转换为一维数组索引。
 *
 * 搜索状态使用一维 vector 保存，索引公式为 index = y * size_x + x。
 *
 * @param x 栅格 x 坐标。
 * @param y 栅格 y 坐标。
 * @return unsigned int 一维数组索引。
 */
unsigned int AStarGlobalPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return y * costmap_->getSizeInCellsX() + x;
}

/**
 * @brief 判断栅格是否可通行。
 *
 * 未知区域由 allow_unknown_ 决定是否允许通过；其他栅格只有 cost 小于
 * obstacle_threshold_ 时才认为可通行。
 *
 * @param x 栅格 x 坐标。
 * @param y 栅格 y 坐标。
 * @return true 栅格可通行；false 栅格不可通行。
 */
bool AStarGlobalPlanner::isCellTraversable(unsigned int x, unsigned int y) const
{
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < obstacle_threshold_;
}

/**
 * @brief 在指定端点附近寻找最近可通行栅格。
 *
 * 以 seed 为中心逐圈扩展搜索，并用真实距离限制在 tolerance 半径内。找到第一个
 * 可通行栅格后立即返回。
 *
 * @param seed_x 搜索中心 x 坐标。
 * @param seed_y 搜索中心 y 坐标。
 * @param tolerance 最大搜索半径，单位 m。
 * @param cell 输出找到的可通行栅格。
 * @return true 找到可通行栅格；false 未找到。
 */
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
          ASTAR_TRACE(
            logger_,
            "[AStarTrace] nearest traversable found seed=(%u,%u) cell=(%u,%u) radius=%d",
            seed_x, seed_y, cell.x, cell.y, radius);
          return true;
        }
      }
    }
  }

  return false;
}

/**
 * @brief A* 启发函数。
 *
 * 当前使用欧氏距离估计从当前栅格到目标栅格的剩余距离，适合二维栅格地图和
 * 8 邻接搜索场景。
 *
 * @param x 当前栅格 x 坐标。
 * @param y 当前栅格 y 坐标。
 * @param goal_x 目标栅格 x 坐标。
 * @param goal_y 目标栅格 y 坐标。
 * @return double 当前栅格到目标栅格的启发式距离。
 */
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

/**
 * @brief 判断两个栅格之间是否存在无障碍直线视线。
 *
 * 使用 Bresenham 类似的栅格遍历方法检查 from 到 to 之间的每个栅格。如果沿途
 * 任意栅格不可通行，则说明不能直接连线。
 *
 * @param from 起始栅格。
 * @param to 终止栅格。
 * @return true 两点之间可直连；false 中间存在不可通行栅格。
 */
bool AStarGlobalPlanner::hasLineOfSight(const GridCell & from, const GridCell & to) const
{
  // 使用 Bresenham 思路沿直线遍历栅格，只要中途遇到不可通行 cell 就判定不可直连。
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

/**
 * @brief 简化原始栅格路径。
 *
 * 从路径锚点开始尽量向后寻找最远的可直连点，从而跳过中间锯齿点。该处理可以
 * 降低局部控制器频繁转向的概率，让全局路径更平滑。
 *
 * @param grid_path A* 回溯得到的原始栅格路径。
 * @return std::vector<GridCell> 简化后的栅格路径。
 */
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

/**
 * @brief 根据父节点表回溯原始栅格路径。
 *
 * A* 搜索成功后，parents 中记录了每个栅格的来源栅格。本函数从 goal_index 反向
 * 追溯到 start_index，再反转得到从起点到终点的正向路径。
 *
 * @param parents A* 搜索生成的父节点表。
 * @param start_index 起点一维索引。
 * @param goal_index 终点一维索引。
 * @return std::vector<GridCell> 从起点到终点的原始栅格路径。
 */
std::vector<AStarGlobalPlanner::GridCell> AStarGlobalPlanner::reconstructGridPath(
  const std::vector<int> & parents,
  unsigned int start_index,
  unsigned int goal_index) const
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
  traceGridPath("reconstructed_grid_path", grid_path);
  return grid_path;
}

/**
 * @brief 将栅格路径转换为世界坐标路径点。
 *
 * 中间点使用 costmap_->mapToWorld() 转换为 map 坐标系下的米制坐标；首尾点保留
 * 原始 start/goal pose，避免 cell 中心点带来的端点偏移。
 *
 * @param grid_path 简化后的栅格路径。
 * @param start 原始起点位姿。
 * @param goal 原始目标位姿。
 * @return std::vector<WorldPoint> 世界坐标路径点。
 */
std::vector<AStarGlobalPlanner::WorldPoint> AStarGlobalPlanner::toWorldPoints(
  const std::vector<GridCell> & grid_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  std::vector<WorldPoint> world_points;
  world_points.reserve(grid_path.size());

  // 首尾使用真实 pose，避免 costmap cell 中心点造成 RViz 里路径端点轻微偏移。
  world_points.push_back(WorldPoint{start.pose.position.x, start.pose.position.y});

  for (size_t i = 1; i + 1 < grid_path.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(grid_path[i].x, grid_path[i].y, wx, wy);
    world_points.push_back(WorldPoint{wx, wy});
  }

  world_points.push_back(WorldPoint{goal.pose.position.x, goal.pose.position.y});
  traceWorldPoints("world_key_points", world_points);
  return world_points;
}

/**
 * @brief 向 Path 追加一段线性插值路径。
 *
 * 按 interpolation_resolution_ 将 from 到 to 的直线段拆成若干 PoseStamped，
 * 并根据线段方向设置 yaw。skip_first_point 用于避免相邻线段连接点重复。
 *
 * @param path 输出路径对象。
 * @param from 线段起点。
 * @param to 线段终点。
 * @param skip_first_point 是否跳过该段第一个点。
 */
void AStarGlobalPlanner::appendInterpolatedSegment(
  nav_msgs::msg::Path & path,
  const WorldPoint & from,
  const WorldPoint & to,
  bool skip_first_point) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / interpolation_resolution_)));
  ASTAR_TRACE(
    logger_,
    "[AStarTrace] interpolate segment from=(%.3f,%.3f) to=(%.3f,%.3f) "
    "distance=%.3f yaw=%.3f steps=%d skip_first=%s",
    from.x, from.y, to.x, to.y, distance, yaw, steps, skip_first_point ? "true" : "false");

  for (int step = 0; step < steps; ++step) {
    if (skip_first_point && step == 0) {
      continue;
    }

    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    auto pose = makePose(from.x + ratio * dx, from.y + ratio * dy, yaw);
    pose.header = path.header;
    path.poses.push_back(pose);
  }
}

/**
 * @brief 创建指定位置和朝向的 PoseStamped。
 *
 * yaw 以弧度表示，函数内部转换为四元数。header 默认使用当前规划器的全局坐标系
 * 和当前时钟，调用方也可以按需要覆盖 header。
 *
 * @param x map 坐标系下 x 坐标。
 * @param y map 坐标系下 y 坐标。
 * @param yaw 路径点朝向，单位 rad。
 * @return geometry_msgs::msg::PoseStamped 路径点位姿。
 */
geometry_msgs::msg::PoseStamped AStarGlobalPlanner::makePose(
  double x,
  double y,
  double yaw) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = clock_->now();
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

/**
 * @brief 根据 A* 搜索结果构建最终 Path。
 *
 * 构建流程包括：父节点回溯、line-of-sight 路径简化、栅格坐标转世界坐标、分段
 * 插值以及追加用户原始目标 pose。输出 Path 可直接交给 Nav2 controller 跟踪。
 *
 * @param parents A* 搜索生成的父节点表。
 * @param start_index 起点一维索引。
 * @param goal_index 终点一维索引。
 * @param start 原始起点位姿。
 * @param goal 原始目标位姿。
 * @return nav_msgs::msg::Path 最终全局路径。
 */
nav_msgs::msg::Path AStarGlobalPlanner::buildPath(
  const std::vector<int> & parents,
  unsigned int start_index,
  unsigned int goal_index,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const auto grid_path = reconstructGridPath(parents, start_index, goal_index);
  const auto simplified_path = simplifyGridPath(grid_path);
  traceGridPath("simplified_grid_path", simplified_path);
  const auto world_points = toWorldPoints(simplified_path, start, goal);

  nav_msgs::msg::Path path = makeEmptyPath();
  for (size_t i = 0; i + 1 < world_points.size(); ++i) {
    appendInterpolatedSegment(path, world_points[i], world_points[i + 1], i > 0);
  }

  // 最后一个点保留用户指定的目标朝向，方便 Nav2 controller 执行最终姿态调整。
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);
  ASTAR_TRACE(logger_, "[AStarTrace] final path poses=%zu", path.poses.size());

  return path;
}

/**
 * @brief 打印单个栅格点。
 *
 * 仅在 ASTAR_TRACE_POINTS 宏开启时编译，用于观察端点转换和归一化结果。
 *
 * @param label 日志标签。
 * @param cell 需要打印的栅格点。
 */
void AStarGlobalPlanner::traceGridCell(const char * label, const GridCell & cell) const
{
#if ASTAR_TRACE_POINTS
  RCLCPP_INFO(logger_, "[AStarTrace] %s=(%u,%u)", label, cell.x, cell.y);
#else
  (void)label;
  (void)cell;
#endif
}

/**
 * @brief 打印栅格路径点序列。
 *
 * 仅在 ASTAR_TRACE_POINTS 宏开启时编译，用于观察 A* 回溯路径和简化后路径。
 *
 * @param label 日志标签。
 * @param path 需要打印的栅格路径。
 */
void AStarGlobalPlanner::traceGridPath(
  const char * label,
  const std::vector<GridCell> & path) const
{
#if ASTAR_TRACE_POINTS
  std::ostringstream stream;
  stream << "[AStarTrace] " << label << " size=" << path.size() << " ";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      stream << " -> ";
    }
    stream << "(" << path[i].x << "," << path[i].y << ")";
  }
  RCLCPP_INFO(logger_, "%s", stream.str().c_str());
#else
  (void)label;
  (void)path;
#endif
}

/**
 * @brief 打印世界坐标路径关键点序列。
 *
 * 仅在 ASTAR_TRACE_POINTS 宏开启时编译，用于观察插值前的 map 坐标路径。
 *
 * @param label 日志标签。
 * @param points 需要打印的世界坐标点。
 */
void AStarGlobalPlanner::traceWorldPoints(
  const char * label,
  const std::vector<WorldPoint> & points) const
{
#if ASTAR_TRACE_POINTS
  std::ostringstream stream;
  stream << "[AStarTrace] " << label << " size=" << points.size() << " ";
  stream.setf(std::ios::fixed);
  stream.precision(3);
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      stream << " -> ";
    }
    stream << "(" << points[i].x << "," << points[i].y << ")";
  }
  RCLCPP_INFO(logger_, "%s", stream.str().c_str());
#else
  (void)label;
  (void)points;
#endif
}

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

PLUGINLIB_EXPORT_CLASS(
  ros2_all_wheel_sim::global_planner::AStarGlobalPlanner,
  nav2_core::GlobalPlanner)
