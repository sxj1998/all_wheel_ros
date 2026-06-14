#ifndef ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_CORE_HPP_
#define ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_CORE_HPP_

#include <cstddef>
#include <vector>

namespace ros2_all_wheel_sim
{
namespace global_planner
{
namespace astar
{

/**
 * @brief 二维栅格坐标。
 */
struct GridCell
{
  /**
   * @brief 栅格横坐标。
   */
  unsigned int x{0};

  /**
   * @brief 栅格纵坐标。
   */
  unsigned int y{0};
};

/**
 * @brief 二维世界坐标点。
 */
struct Point2D
{
  /**
   * @brief x 坐标，单位 m。
   */
  double x{0.0};

  /**
   * @brief y 坐标，单位 m。
   */
  double y{0.0};

  /**
   * @brief 路径点朝向，单位 rad。
   */
  double yaw{0.0};
};

/**
 * @brief 与 ROS/Nav2 解耦的栅格地图。
 */
struct GridMap
{
  /**
   * @brief 地图宽度，单位 cell。
   */
  unsigned int width{0};

  /**
   * @brief 地图高度，单位 cell。
   */
  unsigned int height{0};

  /**
   * @brief 地图分辨率，单位 m/cell。
   */
  double resolution{0.05};

  /**
   * @brief 地图原点 x 坐标，单位 m。
   */
  double origin_x{0.0};

  /**
   * @brief 地图原点 y 坐标，单位 m。
   */
  double origin_y{0.0};

  /**
   * @brief 栅格代价数组，按 y * width + x 展开。
   */
  std::vector<unsigned char> costs;
};

/**
 * @brief A* 纯算法配置。
 */
struct Config
{
  /**
   * @brief 端点不可通行时搜索替代栅格的半径，单位 m。
   */
  double tolerance{0.5};

  /**
   * @brief costmap 代价对移动代价的惩罚系数。
   */
  double cost_penalty{2.0};

  /**
   * @brief 输出路径插值间距，单位 m。
   */
  double interpolation_resolution{0.10};

  /**
   * @brief 是否允许路径穿过未知区域。
   */
  bool allow_unknown{false};

  /**
   * @brief 是否使用 8 邻接搜索。
   */
  bool use_8_connected{true};

  /**
   * @brief cost 小于该阈值时认为可通行。
   */
  unsigned char obstacle_threshold{253};
};

/**
 * @brief A* 规划输入。
 */
struct PlanInput
{
  /**
   * @brief 当前栅格地图。
   */
  GridMap map;

  /**
   * @brief 起点世界坐标。
   */
  Point2D start;

  /**
   * @brief 目标世界坐标。
   */
  Point2D goal;
};

/**
 * @brief A* 规划结果。
 */
struct PlanResult
{
  /**
   * @brief true 表示成功找到路径。
   */
  bool found{false};

  /**
   * @brief 失败或修正端点时的状态码。
   */
  enum class Status
  {
    kSuccess,
    kInvalidMap,
    kStartOutsideMap,
    kGoalOutsideMap,
    kNoTraversableStart,
    kNoTraversableGoal,
    kNoPath
  } status{Status::kInvalidMap};

  /**
   * @brief 最终输出的世界坐标路径。
   */
  std::vector<Point2D> path;

  /**
   * @brief A* 回溯得到的原始栅格路径。
   */
  std::vector<GridCell> raw_grid_path;

  /**
   * @brief line-of-sight 简化后的栅格路径。
   */
  std::vector<GridCell> simplified_grid_path;
};

/**
 * @brief 与 ROS/Nav2 解耦的 A* 全局规划器。
 */
class Planner
{
public:
  explicit Planner(Config config = Config{});

  /**
   * @brief 更新规划器配置。
   *
   * @param config 新配置。
   */
  void setConfig(Config config);

  /**
   * @brief 获取当前配置。
   *
   * @return 当前配置。
   */
  const Config & config() const;

  /**
   * @brief 执行 A* 全局规划。
   *
   * @param input 规划输入。
   * @return 规划结果。
   */
  PlanResult plan(const PlanInput & input) const;

private:
  struct GridOffset
  {
    int dx{0};
    int dy{0};
  };

  struct SearchResult
  {
    bool found{false};
    std::vector<int> parents;
  };

  /**
   * @brief 判断地图数据是否完整有效。
   *
   * @param map 栅格地图。
   * @return true 表示地图可用于规划。
   */
  bool isMapValid(const GridMap & map) const;

  /**
   * @brief 将世界坐标转换为栅格坐标。
   *
   * @param map 栅格地图。
   * @param point 世界坐标点。
   * @param cell 输出栅格坐标。
   * @return true 表示转换成功。
   */
  bool worldToGrid(const GridMap & map, const Point2D & point, GridCell & cell) const;

  /**
   * @brief 将栅格坐标转换为世界坐标。
   *
   * @param map 栅格地图。
   * @param cell 栅格坐标。
   * @return 世界坐标点。
   */
  Point2D gridToWorld(const GridMap & map, const GridCell & cell) const;

  /**
   * @brief 将端点修正到附近可通行栅格。
   *
   * @param map 栅格地图。
   * @param original_cell 原始端点栅格。
   * @param normalized_cell 输出修正后的端点栅格。
   * @return true 表示找到可通行端点。
   */
  bool normalizeEndpoint(
    const GridMap & map,
    const GridCell & original_cell,
    GridCell & normalized_cell) const;

  /**
   * @brief 在栅格地图上执行 A* 搜索。
   *
   * @param map 栅格地图。
   * @param start_cell 起点栅格。
   * @param goal_cell 目标栅格。
   * @return 搜索结果。
   */
  SearchResult searchPath(
    const GridMap & map,
    const GridCell & start_cell,
    const GridCell & goal_cell) const;

  /**
   * @brief 获取当前配置下的邻接方向。
   *
   * @return 4 邻接或 8 邻接偏移表。
   */
  const std::vector<GridOffset> & neighborOffsets() const;

  /**
   * @brief 计算相邻栅格移动代价。
   *
   * @param map 栅格地图。
   * @param current_cell 当前栅格。
   * @param neighbor_cell 相邻栅格。
   * @return 移动代价。
   */
  double traversalCost(
    const GridMap & map,
    const GridCell & current_cell,
    const GridCell & neighbor_cell) const;

  /**
   * @brief 根据父节点表回溯栅格路径。
   *
   * @param map 栅格地图。
   * @param parents 父节点表。
   * @param start_index 起点一维索引。
   * @param goal_index 目标一维索引。
   * @return 原始栅格路径。
   */
  std::vector<GridCell> reconstructGridPath(
    const GridMap & map,
    const std::vector<int> & parents,
    unsigned int start_index,
    unsigned int goal_index) const;

  /**
   * @brief 使用 line-of-sight 简化栅格路径。
   *
   * @param map 栅格地图。
   * @param grid_path 原始栅格路径。
   * @return 简化后的栅格路径。
   */
  std::vector<GridCell> simplifyGridPath(
    const GridMap & map,
    const std::vector<GridCell> & grid_path) const;

  /**
   * @brief 将栅格关键点转换为世界坐标关键点。
   *
   * @param map 栅格地图。
   * @param grid_path 栅格路径。
   * @param start 原始起点。
   * @param goal 原始目标点。
   * @return 世界坐标关键点。
   */
  std::vector<Point2D> toWorldPoints(
    const GridMap & map,
    const std::vector<GridCell> & grid_path,
    const Point2D & start,
    const Point2D & goal) const;

  /**
   * @brief 向路径追加线性插值段。
   *
   * @param path 输出路径。
   * @param from 线段起点。
   * @param to 线段终点。
   * @param skip_first_point true 表示跳过该段第一个点。
   */
  void appendInterpolatedSegment(
    std::vector<Point2D> & path,
    const Point2D & from,
    const Point2D & to,
    bool skip_first_point) const;

  /**
   * @brief 判断栅格是否可通行。
   *
   * @param map 栅格地图。
   * @param x 栅格 x 坐标。
   * @param y 栅格 y 坐标。
   * @return true 表示可通行。
   */
  bool isCellTraversable(const GridMap & map, unsigned int x, unsigned int y) const;

  /**
   * @brief 在端点附近寻找最近可通行栅格。
   *
   * @param map 栅格地图。
   * @param seed_x 搜索中心 x 坐标。
   * @param seed_y 搜索中心 y 坐标。
   * @param cell 输出可通行栅格。
   * @return true 表示找到可通行栅格。
   */
  bool findNearestTraversableCell(
    const GridMap & map,
    unsigned int seed_x,
    unsigned int seed_y,
    GridCell & cell) const;

  /**
   * @brief A* 启发函数。
   *
   * @param x 当前栅格 x 坐标。
   * @param y 当前栅格 y 坐标。
   * @param goal_x 目标栅格 x 坐标。
   * @param goal_y 目标栅格 y 坐标。
   * @return 启发式距离。
   */
  double heuristic(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) const;

  /**
   * @brief 判断两个栅格是否可以直连。
   *
   * @param map 栅格地图。
   * @param from 起始栅格。
   * @param to 终止栅格。
   * @return true 表示中间没有障碍。
   */
  bool hasLineOfSight(const GridMap & map, const GridCell & from, const GridCell & to) const;

  /**
   * @brief 将二维栅格坐标转换为一维索引。
   *
   * @param map 栅格地图。
   * @param x 栅格 x 坐标。
   * @param y 栅格 y 坐标。
   * @return 一维索引。
   */
  unsigned int toIndex(const GridMap & map, unsigned int x, unsigned int y) const;

  /**
   * @brief 读取指定栅格代价。
   *
   * @param map 栅格地图。
   * @param x 栅格 x 坐标。
   * @param y 栅格 y 坐标。
   * @return 栅格代价。
   */
  unsigned char costAt(const GridMap & map, unsigned int x, unsigned int y) const;

  /**
   * @brief 规整配置参数边界。
   */
  void normalizeConfig();

  /**
   * @brief 已规整的 A* 配置。
   */
  Config config_;
};

}  // namespace astar
}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_CORE_HPP_
