#ifndef ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
#define ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#ifndef ASTAR_TRACE_POINTS
#define ASTAR_TRACE_POINTS 0
#endif

namespace ros2_all_wheel_sim
{
namespace global_planner
{

class AStarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  /// 构造函数：插件由 Nav2 planner_server 通过 pluginlib 创建。
  AStarGlobalPlanner() = default;

  /// 析构函数：使用默认析构即可，资源由智能指针和 Nav2 生命周期管理。
  ~AStarGlobalPlanner() override = default;

  /// Nav2 生命周期 configure 阶段入口：读取参数并保存 costmap、TF、node 等上下文。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// Nav2 生命周期 cleanup 阶段入口：当前无需手动释放额外资源。
  void cleanup() override;

  /// Nav2 生命周期 activate 阶段入口：规划器激活时记录状态。
  void activate() override;

  /// Nav2 生命周期 deactivate 阶段入口：规划器停用时记录状态。
  void deactivate() override;

  /// Nav2 全局规划入口：根据起点、终点和全局代价地图生成 nav_msgs::msg::Path。
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /// 代价地图中的二维栅格坐标，单位是 cell。
  struct GridCell
  {
    /// 栅格横坐标。
    unsigned int x;

    /// 栅格纵坐标。
    unsigned int y;
  };

  /// 世界坐标路径点，单位是 m。
  struct WorldPoint
  {
    /// map 坐标系下的 x 坐标。
    double x;

    /// map 坐标系下的 y 坐标。
    double y;
  };

  /// 相邻栅格方向偏移，允许负数。
  struct GridOffset
  {
    /// x 方向偏移。
    int dx;

    /// y 方向偏移。
    int dy;
  };

  /// A* 搜索队列中的候选节点。
  struct QueueNode
  {
    /// f = g + h，值越小越优先扩展。
    double f_score;

    /// 当前栅格的一维索引。
    unsigned int index;

    /// priority_queue 默认大顶堆；这里反转比较，得到 f_score 最小优先。
    bool operator>(const QueueNode & other) const
    {
      return f_score > other.f_score;
    }
  };

  /// A* 搜索结果，包含是否成功以及用于回溯的父节点表。
  struct SearchResult
  {
    /// true 表示已经找到从起点到终点的连通路径。
    bool found{false};

    /// parents[i] 记录索引 i 的父节点索引，用于从终点回溯路径。
    std::vector<int> parents;
  };

  /// 创建带有正确时间戳和 frame_id 的空路径。
  nav_msgs::msg::Path makeEmptyPath() const;

  /// 将 PoseStamped 的世界坐标转换为 costmap 栅格坐标。
  bool poseToGridCell(const geometry_msgs::msg::PoseStamped & pose, GridCell & cell) const;

  /// 如果端点落在不可通行区域，则在 tolerance 范围内寻找最近可通行栅格。
  bool normalizeEndpoint(const GridCell & original_cell, GridCell & normalized_cell) const;

  /// 在全局代价地图上执行 A* 搜索，返回搜索结果和父节点表。
  SearchResult searchPath(const GridCell & start_cell, const GridCell & goal_cell) const;

  /// 获取当前配置下的邻接偏移；支持 4 邻接或 8 邻接。
  const std::vector<GridOffset> & neighborOffsets() const;

  /// 计算从当前栅格移动到相邻栅格的实际代价。
  double traversalCost(const GridCell & current_cell, const GridCell & neighbor_cell) const;

  /// 根据父节点表，从终点回溯出原始栅格路径。
  std::vector<GridCell> reconstructGridPath(
    const std::vector<int> & parents,
    unsigned int start_index,
    unsigned int goal_index) const;

  /// 将简化后的栅格路径转换为世界坐标点，并保留真实起点和目标点。
  std::vector<WorldPoint> toWorldPoints(
    const std::vector<GridCell> & grid_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// 向 Path 中追加一段线性插值后的路径点。
  void appendInterpolatedSegment(
    nav_msgs::msg::Path & path,
    const WorldPoint & from,
    const WorldPoint & to,
    bool skip_first_point) const;

  /// 创建带 yaw 朝向的 PoseStamped。
  geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw) const;

  /// 将二维栅格坐标转换为一维数组索引。
  unsigned int toIndex(unsigned int x, unsigned int y) const;

  /// 判断指定栅格是否允许规划器通行。
  bool isCellTraversable(unsigned int x, unsigned int y) const;

  /// 在指定栅格附近搜索最近的可通行栅格。
  bool findNearestTraversableCell(
    unsigned int seed_x,
    unsigned int seed_y,
    double tolerance,
    GridCell & cell) const;

  /// A* 启发函数：估计当前栅格到目标栅格的剩余距离。
  double heuristic(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) const;

  /// 判断两个栅格之间是否可以用一条不穿越障碍的直线连接。
  bool hasLineOfSight(const GridCell & from, const GridCell & to) const;

  /// 使用 line-of-sight 简化原始栅格路径，减少锯齿点。
  std::vector<GridCell> simplifyGridPath(const std::vector<GridCell> & grid_path) const;

  /// 将 A* 父节点表转换为最终可发布的 nav_msgs::msg::Path。
  nav_msgs::msg::Path buildPath(
    const std::vector<int> & parents,
    unsigned int start_index,
    unsigned int goal_index,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// 打印单个栅格点，用于跟踪 A* 输入、端点归一化和搜索过程。
  void traceGridCell(const char * label, const GridCell & cell) const;

  /// 打印栅格路径点序列，用于观察回溯路径和简化后路径。
  void traceGridPath(const char * label, const std::vector<GridCell> & path) const;

  /// 打印世界坐标点序列，用于观察输出 Path 插值前的关键点。
  void traceWorldPoints(const char * label, const std::vector<WorldPoint> & points) const;

  /// Nav2 lifecycle node 的弱引用，避免插件持有 node 生命周期。
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  /// TF buffer，保留给 Nav2 插件接口和后续需要 frame 转换的扩展。
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /// Nav2 costmap ROS 封装对象。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /// 底层二维代价地图，A* 搜索直接读取该对象。
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  /// 日志器，使用 planner_server 的 logger 便于统一输出。
  rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};

  /// ROS 时钟，用于给 Path 和 PoseStamped 打时间戳。
  rclcpp::Clock::SharedPtr clock_;

  /// 插件在 planner_server 中的实例名，当前配置通常是 GridBased。
  std::string name_;

  /// 全局规划坐标系，通常是 map。
  std::string global_frame_;

  /// 起点或终点不可通行时，允许向周围搜索替代栅格的半径，单位 m。
  double tolerance_{0.5};

  /// 代价地图 cost 对移动代价的惩罚系数，越大越倾向远离障碍。
  double cost_penalty_{2.0};

  /// 输出路径插值间距，单位 m。
  double interpolation_resolution_{0.10};

  /// 是否允许路径穿过未知区域。
  bool allow_unknown_{false};

  /// 是否使用 8 邻接；false 时使用 4 邻接。
  bool use_8_connected_{true};

  /// cost 小于该阈值时认为可通行。
  unsigned char obstacle_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
