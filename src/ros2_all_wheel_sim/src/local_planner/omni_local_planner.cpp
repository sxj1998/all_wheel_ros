#include "ros2_all_wheel_sim/local_planner/omni_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_all_wheel_sim
{
namespace local_planner
{

/**
 * @brief 配置 Nav2 控制器插件并初始化 DWA core、costmap 和 trace logger。
 *
 * @param parent Nav2 生命周期节点。
 * @param name 插件实例名。
 * @param tf TF 缓冲区。
 * @param costmap_ros Nav2 costmap 封装对象。
 */
void OmniLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in OmniLocalPlanner::configure");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  costmap_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".desired_linear_vel", rclcpp::ParameterValue(desired_linear_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_linear_vel", rclcpp::ParameterValue(max_linear_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_lateral_vel", rclcpp::ParameterValue(max_lateral_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_vel", rclcpp::ParameterValue(max_angular_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_linear_vel", rclcpp::ParameterValue(min_linear_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_lateral_vel", rclcpp::ParameterValue(min_lateral_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_angular_vel", rclcpp::ParameterValue(min_angular_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".acc_lim_x", rclcpp::ParameterValue(acc_lim_x_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".acc_lim_y", rclcpp::ParameterValue(acc_lim_y_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(acc_lim_theta_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sim_time", rclcpp::ParameterValue(sim_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sim_step", rclcpp::ParameterValue(sim_step_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".controller_period", rclcpp::ParameterValue(controller_period_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_approach_linear_vel", rclcpp::ParameterValue(min_approach_linear_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_dist", rclcpp::ParameterValue(lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".approach_dist", rclcpp::ParameterValue(approach_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(xy_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(yaw_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_distance_weight", rclcpp::ParameterValue(path_distance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".target_distance_weight", rclcpp::ParameterValue(target_distance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_distance_weight", rclcpp::ParameterValue(goal_distance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_weight", rclcpp::ParameterValue(obstacle_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_weight", rclcpp::ParameterValue(heading_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_weight", rclcpp::ParameterValue(velocity_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_trans_vel", rclcpp::ParameterValue(min_trans_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_heading_min_angle",
    rclcpp::ParameterValue(rotate_to_heading_min_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_heading_angular_vel",
    rclcpp::ParameterValue(rotate_to_heading_angular_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_forward_only", rclcpp::ParameterValue(use_forward_only_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vx_samples", rclcpp::ParameterValue(vx_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vy_samples", rclcpp::ParameterValue(vy_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vtheta_samples", rclcpp::ParameterValue(vtheta_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_threshold",
    rclcpp::ParameterValue(static_cast<int>(obstacle_threshold_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_enabled", rclcpp::ParameterValue(trace_options_.enabled));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_directory", rclcpp::ParameterValue(trace_options_.directory));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_every_n", rclcpp::ParameterValue(trace_options_.every_n));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_candidate_stride", rclcpp::ParameterValue(trace_options_.candidate_stride));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_candidate_state_stride",
    rclcpp::ParameterValue(trace_options_.candidate_state_stride));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_costmap_stride", rclcpp::ParameterValue(trace_options_.costmap_stride));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trace_cost_threshold", rclcpp::ParameterValue(trace_options_.cost_threshold));

  node->get_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(name_ + ".max_lateral_vel", max_lateral_vel_);
  node->get_parameter(name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(name_ + ".min_lateral_vel", min_lateral_vel_);
  node->get_parameter(name_ + ".min_angular_vel", min_angular_vel_);
  node->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(name_ + ".acc_lim_y", acc_lim_y_);
  node->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(name_ + ".sim_time", sim_time_);
  node->get_parameter(name_ + ".sim_step", sim_step_);
  node->get_parameter(name_ + ".controller_period", controller_period_);
  node->get_parameter(name_ + ".min_approach_linear_vel", min_approach_linear_vel_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name_ + ".approach_dist", approach_dist_);
  node->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(name_ + ".path_distance_weight", path_distance_weight_);
  node->get_parameter(name_ + ".target_distance_weight", target_distance_weight_);
  node->get_parameter(name_ + ".goal_distance_weight", goal_distance_weight_);
  node->get_parameter(name_ + ".obstacle_weight", obstacle_weight_);
  node->get_parameter(name_ + ".heading_weight", heading_weight_);
  node->get_parameter(name_ + ".velocity_weight", velocity_weight_);
  node->get_parameter(name_ + ".min_trans_vel", min_trans_vel_);
  node->get_parameter(name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(name_ + ".use_forward_only", use_forward_only_);
  node->get_parameter(name_ + ".vx_samples", vx_samples_);
  node->get_parameter(name_ + ".vy_samples", vy_samples_);
  node->get_parameter(name_ + ".vtheta_samples", vtheta_samples_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".trace_enabled", trace_options_.enabled);
  node->get_parameter(name_ + ".trace_directory", trace_options_.directory);
  node->get_parameter(name_ + ".trace_every_n", trace_options_.every_n);
  node->get_parameter(name_ + ".trace_candidate_stride", trace_options_.candidate_stride);
  node->get_parameter(name_ + ".trace_candidate_state_stride", trace_options_.candidate_state_stride);
  node->get_parameter(name_ + ".trace_costmap_stride", trace_options_.costmap_stride);
  node->get_parameter(name_ + ".trace_cost_threshold", trace_options_.cost_threshold);

  int obstacle_threshold = obstacle_threshold_;
  node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold);
  obstacle_threshold_ = static_cast<unsigned char>(
    std::clamp(obstacle_threshold, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  /**
   * DWA core 只接收纯算法配置，ROS 参数读取集中在插件适配层。
   */
  planner_.setConfig(plannerConfigFromParameters());
  trace_logger_.configure(trace_options_);
  if (trace_logger_.open(costmap_)) {
    RCLCPP_INFO(logger_, "DWA runtime trace enabled: %s", trace_options_.directory.c_str());
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s decoupled DWA planner frame=%s sim_time=%.2f samples=(%d,%d,%d)",
    name_.c_str(), costmap_frame_.c_str(), planner_.config().sim_time,
    planner_.config().vx_samples, planner_.config().vy_samples, planner_.config().vtheta_samples);
}

/**
 * @brief 清理生命周期资源。
 */
void OmniLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  trace_logger_.close();
  global_plan_.poses.clear();
}

/**
 * @brief 激活插件。
 */
void OmniLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

/**
 * @brief 停用插件。
 */
void OmniLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

/**
 * @brief 接收 Nav2 全局路径并标记 trace 中的参考路径需要刷新。
 *
 * @param path Nav2 下发的全局路径。
 */
void OmniLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  trace_logger_.markPathDirty();
}

/**
 * @brief Nav2 控制器主入口：转换 ROS 数据、调用 DWA core、返回速度命令。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 当前机器人速度。
 * @param goal_checker Nav2 目标检查器。
 * @return 本周期速度命令。
 */
geometry_msgs::msg::TwistStamped OmniLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "No path set for %s", name_.c_str());
    return zeroCommand(pose.header);
  }

  const std::size_t nearest_index = nearestPathIndex(pose);
  const auto transformed_plan = transformPlan(pose, nearest_index);
  if (transformed_plan.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Failed to transform local plan");
    return zeroCommand(pose.header);
  }

  /**
   * 输入对象是 core 与 ROS 解耦的边界，后续算法不再依赖 ROS 消息。
   */
  dwa::PlanInput input;
  input.pose = toDwaPose(pose);
  input.velocity = dwa::Velocity{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  input.speed_limit = active_speed_limit_;
  input.speed_limit_is_percentage = speed_limit_is_percentage_;
  input.goal_reached = isGoalReached(pose, velocity, goal_checker);
  input.path.reserve(transformed_plan.size());
  for (const auto & path_pose : transformed_plan) {
    input.path.push_back(toDwaPose(path_pose));
  }

  const auto result = planner_.plan(input, [this](const dwa::Pose2D & state) {
      return queryCostmap(state);
    });
  trace_logger_.recordStep(input, result, input.path);

  if (result.goal_reached) {
    return zeroCommand(pose.header);
  }
  if (!result.command_valid) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "DWA found no valid local trajectory");
    return zeroCommand(pose.header);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = result.command.vx;
  cmd.twist.linear.y = result.command.vy;
  cmd.twist.angular.z = result.command.wz;
  return cmd;
}

/**
 * @brief 接收 Nav2 speed filter 或上层模块的速度限制。
 *
 * @param speed_limit 速度限制值。
 * @param percentage true 表示 speed_limit 为百分比。
 */
void OmniLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  active_speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

/**
 * @brief 查找全局路径中离当前位姿最近的点。
 *
 * @param pose 当前机器人位姿。
 * @return 最近路径点索引。
 */
std::size_t OmniLocalPlanner::nearestPathIndex(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (!transformPose(global_plan_.poses[i], pose.header.frame_id, transformed_pose)) {
      continue;
    }
    const double distance = distance2D(pose, transformed_pose);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  return nearest_index;
}

/**
 * @brief 使用 TF 将位姿转换到目标坐标系。
 *
 * @param input 输入位姿。
 * @param target_frame 目标坐标系。
 * @param output 输出位姿。
 * @return true 表示转换成功。
 */
bool OmniLocalPlanner::transformPose(
  const geometry_msgs::msg::PoseStamped & input,
  const std::string & target_frame,
  geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  try {
    output = tf_->transform(input, target_frame, tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(logger_, "Transform failed from %s to %s: %s",
      input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

/**
 * @brief 判断目标是否到达，优先复用 Nav2 GoalChecker。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 当前机器人速度。
 * @param goal_checker Nav2 目标检查器。
 * @return true 表示目标已到达。
 */
bool OmniLocalPlanner::isGoalReached(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) const
{
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!transformPose(global_plan_.poses.back(), pose.header.frame_id, goal_pose)) {
    return false;
  }

  if (goal_checker != nullptr && goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
    return true;
  }

  const double yaw_error = std::abs(dwa::normalizeAngle(poseYaw(goal_pose) - poseYaw(pose)));
  return distance2D(pose, goal_pose) <= xy_goal_tolerance_ && yaw_error <= yaw_goal_tolerance_;
}

/**
 * @brief 将全局路径裁剪并转换成当前控制坐标系下的局部路径。
 *
 * @param pose 当前机器人位姿。
 * @param nearest_index 最近路径点索引。
 * @return 当前控制坐标系下的局部路径。
 */
std::vector<geometry_msgs::msg::PoseStamped> OmniLocalPlanner::transformPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  std::size_t nearest_index) const
{
  std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  local_plan.reserve(global_plan_.poses.size() - nearest_index);

  for (std::size_t i = nearest_index; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (transformPose(global_plan_.poses[i], pose.header.frame_id, transformed_pose)) {
      local_plan.push_back(transformed_pose);
    }
  }

  return local_plan;
}

/**
 * @brief 查询 Nav2 costmap，并转成 DWA core 的碰撞/障碍代价。
 *
 * @param state 待检查的二维状态。
 * @return DWA core 可使用的代价查询结果。
 */
dwa::CostQuery OmniLocalPlanner::queryCostmap(const dwa::Pose2D & state) const
{
  dwa::CostQuery result;
  if (costmap_ == nullptr) {
    return result;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(state.x, state.y, mx, my)) {
    result.collision = true;
    return result;
  }

  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    result.collision = !allow_unknown_;
    return result;
  }
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    result.collision = true;
    return result;
  }

  const double normalized_cost = static_cast<double>(cost) /
    static_cast<double>(std::max(1, static_cast<int>(obstacle_threshold_)));
  result.obstacle_score = std::clamp(normalized_cost, 0.0, 1.0);
  return result;
}

/**
 * @brief 从插件成员变量构造 DWA core 配置。
 *
 * @return DWA core 配置。
 */
dwa::Config OmniLocalPlanner::plannerConfigFromParameters() const
{
  dwa::Config config;
  config.desired_linear_vel = desired_linear_vel_;
  config.max_linear_vel = max_linear_vel_;
  config.max_lateral_vel = max_lateral_vel_;
  config.max_angular_vel = max_angular_vel_;
  config.min_linear_vel = min_linear_vel_;
  config.min_lateral_vel = min_lateral_vel_;
  config.min_angular_vel = min_angular_vel_;
  config.acc_lim_x = acc_lim_x_;
  config.acc_lim_y = acc_lim_y_;
  config.acc_lim_theta = acc_lim_theta_;
  config.sim_time = sim_time_;
  config.sim_step = sim_step_;
  config.controller_period = controller_period_;
  config.min_approach_linear_vel = min_approach_linear_vel_;
  config.lookahead_dist = lookahead_dist_;
  config.approach_dist = approach_dist_;
  config.xy_goal_tolerance = xy_goal_tolerance_;
  config.yaw_goal_tolerance = yaw_goal_tolerance_;
  config.path_distance_weight = path_distance_weight_;
  config.target_distance_weight = target_distance_weight_;
  config.goal_distance_weight = goal_distance_weight_;
  config.obstacle_weight = obstacle_weight_;
  config.heading_weight = heading_weight_;
  config.velocity_weight = velocity_weight_;
  config.min_trans_vel = min_trans_vel_;
  config.rotate_to_heading_min_angle = rotate_to_heading_min_angle_;
  config.rotate_to_heading_angular_vel = rotate_to_heading_angular_vel_;
  config.vx_samples = vx_samples_;
  config.vy_samples = vy_samples_;
  config.vtheta_samples = vtheta_samples_;
  config.use_forward_only = use_forward_only_;
  return config;
}

/**
 * @brief 将 ROS PoseStamped 转换成纯二维位姿。
 *
 * @param pose ROS 位姿。
 * @return DWA 二维位姿。
 */
dwa::Pose2D OmniLocalPlanner::toDwaPose(const geometry_msgs::msg::PoseStamped & pose) const
{
  return dwa::Pose2D{pose.pose.position.x, pose.pose.position.y, poseYaw(pose)};
}

/**
 * @brief 将 DWA core 的旋转命令包装成 Nav2 TwistStamped。
 *
 * @param header 输出消息头。
 * @param heading_error 目标朝向与当前朝向误差。
 * @param velocity 当前机器人速度。
 * @return Nav2 速度命令。
 */
geometry_msgs::msg::TwistStamped OmniLocalPlanner::rotateCommand(
  const std_msgs::msg::Header & header,
  double heading_error,
  const geometry_msgs::msg::Twist & velocity) const
{
  const auto command = planner_.rotateCommand(
    heading_error, dwa::Velocity{velocity.linear.x, velocity.linear.y, velocity.angular.z});
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = command.vx;
  cmd.twist.linear.y = command.vy;
  cmd.twist.angular.z = command.wz;
  return cmd;
}

/**
 * @brief 返回带时间戳的零速度命令。
 *
 * @param header 输出消息头。
 * @return 零速度命令。
 */
geometry_msgs::msg::TwistStamped OmniLocalPlanner::zeroCommand(
  const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.header.stamp = clock_->now();
  return cmd;
}

/**
 * @brief 从四元数姿态中提取 yaw。
 *
 * @param pose ROS 位姿。
 * @return yaw 角，单位 rad。
 */
double OmniLocalPlanner::poseYaw(const geometry_msgs::msg::PoseStamped & pose) const
{
  tf2::Quaternion quaternion(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

/**
 * @brief 角度归一化代理函数。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
double OmniLocalPlanner::normalizedAngle(double angle) const
{
  return dwa::normalizeAngle(angle);
}

/**
 * @brief 计算两个 ROS 位姿的二维距离。
 *
 * @param a 第一个位姿。
 * @param b 第二个位姿。
 * @return 二维欧氏距离。
 */
double OmniLocalPlanner::distance2D(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b) const
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

}  // namespace local_planner
}  // namespace ros2_all_wheel_sim

PLUGINLIB_EXPORT_CLASS(
  ros2_all_wheel_sim::local_planner::OmniLocalPlanner,
  nav2_core::Controller)
