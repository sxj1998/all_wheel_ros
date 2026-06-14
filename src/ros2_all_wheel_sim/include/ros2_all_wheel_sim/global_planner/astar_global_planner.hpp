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
#include "ros2_all_wheel_sim/global_planner/astar_core.hpp"

#ifndef ASTAR_TRACE_POINTS
#define ASTAR_TRACE_POINTS 0
#endif

namespace ros2_all_wheel_sim
{
namespace global_planner
{

/**
 * @brief A* 全局规划器 Nav2 插件适配层。
 *
 * 该类只负责 Nav2 生命周期、参数读取、costmap 转换和 Path 消息封装。
 * 真正的 A* 搜索逻辑位于 astar::Planner，可脱离 Nav2 单独移植。
 */
class AStarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarGlobalPlanner() = default;
  ~AStarGlobalPlanner() override = default;

  /**
   * @brief 配置全局规划器插件。
   *
   * @param parent Nav2 生命周期节点。
   * @param name 插件实例名。
   * @param tf TF 缓冲区。
   * @param costmap_ros Nav2 全局代价地图封装对象。
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理规划器生命周期资源。
   */
  void cleanup() override;

  /**
   * @brief 激活规划器。
   */
  void activate() override;

  /**
   * @brief 停用规划器。
   */
  void deactivate() override;

  /**
   * @brief 创建全局路径。
   *
   * @param start 起点位姿。
   * @param goal 目标位姿。
   * @return 可供 Nav2 controller 跟踪的全局路径。
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief 创建带有正确时间戳和 frame_id 的空路径。
   *
   * @return 空路径。
   */
  nav_msgs::msg::Path makeEmptyPath() const;

  /**
   * @brief 从 Nav2 costmap 拷贝出与 ROS 解耦的栅格地图。
   *
   * @return A* core 可直接使用的栅格地图。
   */
  astar::GridMap makeCoreMap() const;

  /**
   * @brief 将 ROS 位姿转换为 A* core 的二维点。
   *
   * @param pose ROS 位姿。
   * @return A* core 二维点。
   */
  astar::Point2D toCorePoint(const geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief 将 A* core 点转换为 ROS PoseStamped。
   *
   * @param point A* core 二维点。
   * @return ROS 位姿。
   */
  geometry_msgs::msg::PoseStamped toRosPose(const astar::Point2D & point) const;

  /**
   * @brief 将 A* core 结果转换为 Nav2 Path 消息。
   *
   * @param result A* core 规划结果。
   * @param goal 原始目标位姿。
   * @return Nav2 Path 消息。
   */
  nav_msgs::msg::Path toRosPath(
    const astar::PlanResult & result,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /**
   * @brief 从插件成员变量构造 A* core 配置。
   *
   * @return A* core 配置。
   */
  astar::Config plannerConfigFromParameters() const;

  /**
   * @brief 记录 A* core 规划状态。
   *
   * @param result A* core 规划结果。
   */
  void logPlanFailure(const astar::PlanResult & result) const;

  /**
   * @brief Nav2 lifecycle node 的弱引用。
   */
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  /**
   * @brief TF buffer。
   */
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * @brief Nav2 costmap ROS 封装对象。
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief 底层二维代价地图。
   */
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  /**
   * @brief 日志器。
   */
  rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};

  /**
   * @brief ROS 时钟。
   */
  rclcpp::Clock::SharedPtr clock_;

  /**
   * @brief 插件实例名。
   */
  std::string name_;

  /**
   * @brief 全局规划坐标系。
   */
  std::string global_frame_;

  /**
   * @brief 与 Nav2 解耦的 A* 纯算法对象。
   */
  astar::Planner planner_;

  /**
   * @brief 起点或终点不可通行时的替代栅格搜索半径，单位 m。
   */
  double tolerance_{0.5};

  /**
   * @brief costmap cost 对移动代价的惩罚系数。
   */
  double cost_penalty_{2.0};

  /**
   * @brief 输出路径插值间距，单位 m。
   */
  double interpolation_resolution_{0.10};

  /**
   * @brief 是否允许路径穿过未知区域。
   */
  bool allow_unknown_{false};

  /**
   * @brief 是否使用 8 邻接。
   */
  bool use_8_connected_{true};

  /**
   * @brief cost 小于该阈值时认为可通行。
   */
  unsigned char obstacle_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
