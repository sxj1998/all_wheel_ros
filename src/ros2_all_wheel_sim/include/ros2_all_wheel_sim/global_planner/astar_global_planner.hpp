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

namespace ros2_all_wheel_sim
{
namespace global_planner
{

class AStarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarGlobalPlanner() = default;
  ~AStarGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct GridCell
  {
    unsigned int x;
    unsigned int y;
  };

  unsigned int toIndex(unsigned int x, unsigned int y) const;
  bool isCellTraversable(unsigned int x, unsigned int y) const;
  bool findNearestTraversableCell(
    unsigned int seed_x,
    unsigned int seed_y,
    double tolerance,
    GridCell & cell) const;
  double heuristic(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) const;
  bool hasLineOfSight(const GridCell & from, const GridCell & to) const;
  std::vector<GridCell> simplifyGridPath(const std::vector<GridCell> & grid_path) const;
  nav_msgs::msg::Path buildPath(
    const std::vector<int> & parents,
    unsigned int start_index,
    unsigned int goal_index,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  std::string name_;
  std::string global_frame_;
  double tolerance_{0.5};
  double cost_penalty_{2.0};
  double interpolation_resolution_{0.10};
  bool allow_unknown_{false};
  bool use_8_connected_{true};
  unsigned char obstacle_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__GLOBAL_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
