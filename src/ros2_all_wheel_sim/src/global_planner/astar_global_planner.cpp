#include "ros2_all_wheel_sim/global_planner/astar_global_planner.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_all_wheel_sim
{
namespace global_planner
{

/**
 * @brief 配置 A* 全局规划器插件。
 *
 * @param parent Nav2 lifecycle node 弱引用。
 * @param name 插件实例名。
 * @param tf TF buffer。
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

  int obstacle_threshold = obstacle_threshold_;
  node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold);
  obstacle_threshold_ = static_cast<unsigned char>(
    std::clamp(obstacle_threshold, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  interpolation_resolution_ = std::max(interpolation_resolution_, costmap_->getResolution());
  planner_.setConfig(plannerConfigFromParameters());

  RCLCPP_INFO(
    logger_,
    "Configured %s decoupled A* planner frame=%s tolerance=%.2f allow_unknown=%s "
    "use_8_connected=%s",
    name_.c_str(), global_frame_.c_str(), tolerance_, allow_unknown_ ? "true" : "false",
    use_8_connected_ ? "true" : "false");
}

/**
 * @brief 清理规划器资源。
 */
void AStarGlobalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

/**
 * @brief 激活规划器。
 */
void AStarGlobalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

/**
 * @brief 停用规划器。
 */
void AStarGlobalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

/**
 * @brief 根据起点和目标点创建全局路径。
 *
 * @param start 当前机器人起点位姿。
 * @param goal 用户指定目标位姿。
 * @return 可供 Nav2 controller 跟踪的全局路径。
 */
nav_msgs::msg::Path AStarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (costmap_ == nullptr) {
    RCLCPP_ERROR(logger_, "Costmap is not available");
    return makeEmptyPath();
  }

  astar::PlanInput input;
  input.map = makeCoreMap();
  input.start = toCorePoint(start);
  input.goal = toCorePoint(goal);
  const auto result = planner_.plan(input);
  if (!result.found) {
    logPlanFailure(result);
    return makeEmptyPath();
  }

  auto path = toRosPath(result, goal);
  RCLCPP_DEBUG(logger_, "A* generated path with %zu poses", path.poses.size());
  return path;
}

/**
 * @brief 创建带时间戳和全局坐标系的空路径。
 *
 * @return 已设置 header 的空路径。
 */
nav_msgs::msg::Path AStarGlobalPlanner::makeEmptyPath() const
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  return path;
}

/**
 * @brief 从 Nav2 costmap 拷贝纯栅格地图。
 *
 * @return A* core 可直接使用的栅格地图。
 */
astar::GridMap AStarGlobalPlanner::makeCoreMap() const
{
  astar::GridMap map;
  map.width = costmap_->getSizeInCellsX();
  map.height = costmap_->getSizeInCellsY();
  map.resolution = costmap_->getResolution();
  map.origin_x = costmap_->getOriginX();
  map.origin_y = costmap_->getOriginY();
  map.costs.reserve(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height));

  for (unsigned int y = 0; y < map.height; ++y) {
    for (unsigned int x = 0; x < map.width; ++x) {
      map.costs.push_back(costmap_->getCost(x, y));
    }
  }
  return map;
}

/**
 * @brief 将 ROS 位姿转换为 A* core 二维点。
 *
 * @param pose ROS 位姿。
 * @return A* core 二维点。
 */
astar::Point2D AStarGlobalPlanner::toCorePoint(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Quaternion quaternion(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return astar::Point2D{pose.pose.position.x, pose.pose.position.y, yaw};
}

/**
 * @brief 将 A* core 点转换为 ROS PoseStamped。
 *
 * @param point A* core 二维点。
 * @return ROS 位姿。
 */
geometry_msgs::msg::PoseStamped AStarGlobalPlanner::toRosPose(
  const astar::Point2D & point) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = clock_->now();
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, point.yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

/**
 * @brief 将 A* core 结果转换为 Nav2 Path 消息。
 *
 * @param result A* core 规划结果。
 * @param goal 原始目标位姿。
 * @return Nav2 Path 消息。
 */
nav_msgs::msg::Path AStarGlobalPlanner::toRosPath(
  const astar::PlanResult & result,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path = makeEmptyPath();
  path.poses.reserve(result.path.size());
  for (const auto & point : result.path) {
    auto pose = toRosPose(point);
    pose.header = path.header;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header = path.header;
    path.poses.back() = goal_pose;
  }
  return path;
}

/**
 * @brief 从插件成员变量构造 A* core 配置。
 *
 * @return A* core 配置。
 */
astar::Config AStarGlobalPlanner::plannerConfigFromParameters() const
{
  astar::Config config;
  config.tolerance = tolerance_;
  config.cost_penalty = cost_penalty_;
  config.interpolation_resolution = interpolation_resolution_;
  config.allow_unknown = allow_unknown_;
  config.use_8_connected = use_8_connected_;
  config.obstacle_threshold = obstacle_threshold_;
  return config;
}

/**
 * @brief 记录 A* core 规划失败原因。
 *
 * @param result A* core 规划结果。
 */
void AStarGlobalPlanner::logPlanFailure(const astar::PlanResult & result) const
{
  switch (result.status) {
    case astar::PlanResult::Status::kInvalidMap:
      RCLCPP_WARN(logger_, "A* input map is invalid");
      break;
    case astar::PlanResult::Status::kStartOutsideMap:
      RCLCPP_WARN(logger_, "Start pose is outside the global costmap");
      break;
    case astar::PlanResult::Status::kGoalOutsideMap:
      RCLCPP_WARN(logger_, "Goal pose is outside the global costmap");
      break;
    case astar::PlanResult::Status::kNoTraversableStart:
      RCLCPP_WARN(logger_, "No traversable start cell found within %.2f m", tolerance_);
      break;
    case astar::PlanResult::Status::kNoTraversableGoal:
      RCLCPP_WARN(logger_, "No traversable goal cell found within %.2f m", tolerance_);
      break;
    case astar::PlanResult::Status::kNoPath:
      RCLCPP_WARN(logger_, "A* could not find a path to the goal");
      break;
    case astar::PlanResult::Status::kSuccess:
      break;
  }
}

}  // namespace global_planner
}  // namespace ros2_all_wheel_sim

PLUGINLIB_EXPORT_CLASS(
  ros2_all_wheel_sim::global_planner::AStarGlobalPlanner,
  nav2_core::GlobalPlanner)
