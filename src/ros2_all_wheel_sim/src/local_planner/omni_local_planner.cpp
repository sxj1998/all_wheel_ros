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
 * @brief 配置全向局部规划器插件。
 *
 * 该函数由 Nav2 controller_server 在 lifecycle configure 阶段调用，负责保存 node、
 * TF、局部代价地图等运行上下文，并声明、读取、校验 DWA 采样规划相关参数。
 *
 * @param parent Nav2 lifecycle node 弱引用。
 * @param name 插件实例名，当前通常为 FollowPath。
 * @param tf TF buffer，用于把全局路径点转换到控制器当前坐标系。
 * @param costmap_ros Nav2 局部代价地图封装对象。
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

  int obstacle_threshold = obstacle_threshold_;
  node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold);
  obstacle_threshold_ = static_cast<unsigned char>(
    std::clamp(obstacle_threshold, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  desired_linear_vel_ = std::max(0.0, desired_linear_vel_);
  max_linear_vel_ = std::max(0.0, max_linear_vel_);
  max_lateral_vel_ = std::max(0.0, max_lateral_vel_);
  max_angular_vel_ = std::max(0.01, max_angular_vel_);
  min_linear_vel_ = std::clamp(min_linear_vel_, -max_linear_vel_, max_linear_vel_);
  min_lateral_vel_ = std::clamp(min_lateral_vel_, -max_lateral_vel_, max_lateral_vel_);
  min_angular_vel_ = std::clamp(min_angular_vel_, -max_angular_vel_, max_angular_vel_);
  acc_lim_x_ = std::max(0.01, acc_lim_x_);
  acc_lim_y_ = std::max(0.01, acc_lim_y_);
  acc_lim_theta_ = std::max(0.01, acc_lim_theta_);
  min_approach_linear_vel_ = std::clamp(min_approach_linear_vel_, 0.0, max_linear_vel_);
  sim_time_ = std::max(0.2, sim_time_);
  sim_step_ = std::clamp(sim_step_, 0.02, sim_time_);
  controller_period_ = std::max(0.01, controller_period_);
  lookahead_dist_ = std::max(0.05, lookahead_dist_);
  approach_dist_ = std::max(0.05, approach_dist_);
  xy_goal_tolerance_ = std::max(0.0, xy_goal_tolerance_);
  yaw_goal_tolerance_ = std::max(0.0, yaw_goal_tolerance_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);
  path_distance_weight_ = std::max(0.0, path_distance_weight_);
  target_distance_weight_ = std::max(0.0, target_distance_weight_);
  goal_distance_weight_ = std::max(0.0, goal_distance_weight_);
  obstacle_weight_ = std::max(0.0, obstacle_weight_);
  heading_weight_ = std::max(0.0, heading_weight_);
  velocity_weight_ = std::max(0.0, velocity_weight_);
  min_trans_vel_ = std::max(0.0, min_trans_vel_);
  rotate_to_heading_min_angle_ = std::max(0.0, rotate_to_heading_min_angle_);
  rotate_to_heading_angular_vel_ = std::clamp(
    rotate_to_heading_angular_vel_, 0.01, max_angular_vel_);
  vx_samples_ = std::max(2, vx_samples_);
  vy_samples_ = std::max(1, vy_samples_);
  vtheta_samples_ = std::max(3, vtheta_samples_);

  RCLCPP_INFO(
    logger_,
    "Configured %s DWA local planner frame=%s sim_time=%.2f samples=(%d,%d,%d)",
    name_.c_str(), costmap_frame_.c_str(), sim_time_, vx_samples_, vy_samples_, vtheta_samples_);
}

/**
 * @brief 清理局部规划器资源。
 *
 * 当前插件没有额外动态资源需要释放，仅清空缓存的全局路径以避免停用后继续使用旧路径。
 */
void OmniLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  global_plan_.poses.clear();
}

/**
 * @brief 激活局部规划器。
 *
 * 当前实现没有 publisher、timer 等 lifecycle 资源，仅记录状态切换日志。
 */
void OmniLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

/**
 * @brief 停用局部规划器。
 *
 * 当前实现没有 publisher、timer 等 lifecycle 资源，仅记录状态切换日志。
 */
void OmniLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

/**
 * @brief 接收并缓存全局路径。
 *
 * Nav2 每次获得新的全局路径后会调用该函数，computeVelocityCommands() 后续会围绕
 * 这条路径寻找最近点、前瞻点并生成局部速度命令。
 *
 * @param path 全局规划器生成的路径。
 */
void OmniLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

/**
 * @brief 计算下一周期速度命令。
 *
 * 这是 Nav2 Controller 插件的核心入口。函数采用轻量 DWA 思路：在当前速度附近按照
 * 加速度约束采样 vx、vy、wz，对每组速度做短时前向仿真，过滤碰撞轨迹，并根据贴近
 * 路径、接近前瞻点、接近终点、远离障碍、朝向误差和速度目标综合打分。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 当前机器人速度。
 * @param goal_checker Nav2 目标检查器，可为空。
 * @return geometry_msgs::msg::TwistStamped 本周期输出的底盘速度命令。
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

  if (isGoalReached(pose, velocity, goal_checker)) {
    return zeroCommand(pose.header);
  }

  const std::size_t nearest_index = nearestPathIndex(pose);
  const std::size_t target_index = lookaheadPathIndex(nearest_index);

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!transformPose(global_plan_.poses.back(), pose.header.frame_id, goal_pose)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Failed to transform goal pose");
    return zeroCommand(pose.header);
  }

  const double distance_to_goal = distance2D(pose, goal_pose);
  double target_speed = limitedSpeed();
  if (distance_to_goal < approach_dist_) {
    const double scale = std::clamp(distance_to_goal / approach_dist_, 0.0, 1.0);
    target_speed = std::max(min_approach_linear_vel_, target_speed * scale);
  }

  double target_yaw = poseYaw(goal_pose);
  geometry_msgs::msg::PoseStamped target_pose = goal_pose;
  (void)transformPose(global_plan_.poses[target_index], pose.header.frame_id, target_pose);
  if (target_index + 1 < global_plan_.poses.size()) {
    geometry_msgs::msg::PoseStamped next_pose;
    if (transformPose(global_plan_.poses[target_index + 1], pose.header.frame_id, next_pose)) {
      const double path_dx = next_pose.pose.position.x - target_pose.pose.position.x;
      const double path_dy = next_pose.pose.position.y - target_pose.pose.position.y;
      if (std::hypot(path_dx, path_dy) > 1e-3) {
        target_yaw = std::atan2(path_dy, path_dx);
      }
    }
  }

  // 前进模式下先让车头对准路径切线，避免大角度侧滑或背向跟踪。
  const double heading_error = normalizedAngle(target_yaw - poseYaw(pose));
  if (use_forward_only_ &&
    distance_to_goal > xy_goal_tolerance_ &&
    std::abs(heading_error) > rotate_to_heading_min_angle_)
  {
    return rotateCommand(pose.header, heading_error, velocity);
  }

  const auto local_plan = transformPlan(pose, nearest_index);
  const double min_vx = use_forward_only_ ? std::max(
      0.0,
      velocity.linear.x - acc_lim_x_ * controller_period_) :
    std::max(min_linear_vel_, velocity.linear.x - acc_lim_x_ * controller_period_);
  const double max_vx = std::min(max_linear_vel_, velocity.linear.x + acc_lim_x_ * controller_period_);
  const double min_vy = std::max(
    min_lateral_vel_,
    velocity.linear.y - acc_lim_y_ * controller_period_);
  const double max_vy = std::min(
    max_lateral_vel_,
    velocity.linear.y + acc_lim_y_ * controller_period_);
  const double min_wz = std::max(min_angular_vel_, velocity.angular.z - acc_lim_theta_ * controller_period_);
  const double max_wz = std::min(max_angular_vel_, velocity.angular.z + acc_lim_theta_ * controller_period_);
  const auto vx_values = sampleRange(min_vx, max_vx, vx_samples_);
  const auto vy_values = sampleRange(min_vy, max_vy, vy_samples_);
  const auto wz_values = sampleRange(min_wz, max_wz, vtheta_samples_);

  TrajectoryScore best;
  best.score = std::numeric_limits<double>::infinity();

  for (const double vx : vx_values) {
    for (const double vy : vy_values) {
      for (const double wz : wz_values) {
        VelocitySample sample{vx, vy, wz};
        if (distance_to_goal <= xy_goal_tolerance_) {
          // 已进入位置容差时只保留旋转自由度，用于最终姿态对齐。
          sample.vx = 0.0;
          sample.vy = 0.0;
        } else if (std::hypot(sample.vx, sample.vy) < min_trans_vel_) {
          continue;
        }
        const auto score = scoreTrajectory(
          pose, sample, local_plan, target_pose, goal_pose, target_yaw, target_speed);
        if (score.valid && score.score < best.score) {
          best = score;
        }
      }
    }
  }

  if (!best.valid) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "DWA found no valid local trajectory");
    if (use_forward_only_ && distance_to_goal > xy_goal_tolerance_) {
      return rotateCommand(pose.header, heading_error, velocity);
    }
    return zeroCommand(pose.header);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = best.velocity.vx;
  cmd.twist.linear.y = best.velocity.vy;
  cmd.twist.angular.z = best.velocity.wz;
  return cmd;
}

/**
 * @brief 设置外部速度限制。
 *
 * Nav2 speed filter 或其他上层模块可通过该接口限制局部规划器输出速度。
 *
 * @param speed_limit 速度上限；percentage 为 true 时表示百分比。
 * @param percentage true 表示 speed_limit 是 max_linear_vel_ 的百分比。
 */
void OmniLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  active_speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

/**
 * @brief 查找当前位姿在全局路径上的最近点索引。
 *
 * 路径点会先转换到当前 pose 的坐标系，再计算二维距离，避免不同 frame 下直接比较。
 *
 * @param pose 当前机器人位姿。
 * @return 最近路径点索引；若转换全部失败则返回 0。
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
 * @brief 从最近点开始沿路径寻找前瞻点索引。
 *
 * 前瞻点用于引导短期局部轨迹，不直接盯最终目标，能让机器人沿路径逐段跟踪。
 *
 * @param nearest_index 当前最近路径点索引。
 * @return 距离最近点约 lookahead_dist_ 的路径点索引。
 */
std::size_t OmniLocalPlanner::lookaheadPathIndex(std::size_t nearest_index) const
{
  double accumulated_distance = 0.0;
  std::size_t index = nearest_index;

  while (index + 1 < global_plan_.poses.size() && accumulated_distance < lookahead_dist_) {
    accumulated_distance += distance2D(global_plan_.poses[index], global_plan_.poses[index + 1]);
    ++index;
  }

  return index;
}

/**
 * @brief 将位姿转换到指定坐标系。
 *
 * 如果输入已经在目标坐标系中则直接复制；否则使用 TF buffer 进行转换。
 *
 * @param input 输入位姿。
 * @param target_frame 目标坐标系。
 * @param output 输出位姿。
 * @return true 转换成功；false 转换失败。
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
 * @brief 判断机器人是否已经到达目标。
 *
 * 优先使用 Nav2 提供的 GoalChecker；若 GoalChecker 不可用或未判定到达，则使用本插件
 * 的 xy/yaw 容差作为兜底判断。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 当前机器人速度。
 * @param goal_checker Nav2 目标检查器，可为空。
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

  const double yaw_error = std::abs(normalizedAngle(poseYaw(goal_pose) - poseYaw(pose)));
  return distance2D(pose, goal_pose) <= xy_goal_tolerance_ && yaw_error <= yaw_goal_tolerance_;
}

/**
 * @brief 将从最近点到终点的全局路径转换到当前控制坐标系。
 *
 * 转换后的局部路径仅用于轨迹打分中的贴线距离计算。
 *
 * @param pose 当前机器人位姿，用于提供目标 frame。
 * @param nearest_index 当前最近路径点索引。
 * @return 已转换的局部路径点集合。
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
 * @brief 在闭区间 [min_value, max_value] 上均匀采样。
 *
 * 当采样数不足或区间极小时返回区间中点，避免后续循环处理空数组。
 *
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @param samples 采样数量。
 * @return 采样结果。
 */
std::vector<double> OmniLocalPlanner::sampleRange(
  double min_value,
  double max_value,
  int samples) const
{
  if (samples <= 1 || std::abs(max_value - min_value) < 1e-6) {
    return {0.5 * (min_value + max_value)};
  }

  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(samples));
  const double step = (max_value - min_value) / static_cast<double>(samples - 1);
  for (int i = 0; i < samples; ++i) {
    values.push_back(min_value + static_cast<double>(i) * step);
  }

  return values;
}

/**
 * @brief 前向仿真并计算候选速度轨迹得分。
 *
 * 函数会从当前位姿出发，以固定速度采样值积分 sim_time_ 秒。仿真过程中任一点发生
 * 碰撞则轨迹无效；否则按照路径、目标、障碍、朝向和速度目标加权得到总分。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 待评估的速度采样。
 * @param local_plan 当前坐标系下的局部路径。
 * @param target_pose 前瞻目标点。
 * @param goal_pose 最终目标点。
 * @param target_yaw 期望朝向，通常为路径切线方向。
 * @param target_speed 期望平移速度，接近终点时会降低。
 * @return TrajectoryScore 轨迹有效性、总分和对应速度。
 */
OmniLocalPlanner::TrajectoryScore OmniLocalPlanner::scoreTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  const VelocitySample & velocity,
  const std::vector<geometry_msgs::msg::PoseStamped> & local_plan,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  double target_yaw,
  double target_speed) const
{
  TrajectoryScore result;
  result.velocity = velocity;

  SimState state{pose.pose.position.x, pose.pose.position.y, poseYaw(pose)};
  double max_obstacle_score = 0.0;

  for (double time = 0.0; time < sim_time_; time += sim_step_) {
    state = simulateStep(state, velocity, sim_step_);
    double obstacle_score = 0.0;
    if (stateInCollision(state, obstacle_score)) {
      return result;
    }
    max_obstacle_score = std::max(max_obstacle_score, obstacle_score);
  }

  result.valid = true;
  result.obstacle_score = max_obstacle_score;
  const double path_distance = distanceToPlan(state, local_plan);
  const double target_distance = std::hypot(
    state.x - target_pose.pose.position.x,
    state.y - target_pose.pose.position.y);
  const double goal_distance = std::hypot(
    state.x - goal_pose.pose.position.x,
    state.y - goal_pose.pose.position.y);
  const double heading_error = std::abs(normalizedAngle(target_yaw - state.yaw));
  const double velocity_score = scoreVelocity(velocity.vx, velocity.vy, target_speed);

  result.score =
    path_distance_weight_ * path_distance +
    target_distance_weight_ * target_distance +
    goal_distance_weight_ * goal_distance +
    obstacle_weight_ * max_obstacle_score +
    heading_weight_ * heading_error +
    velocity_weight_ * velocity_score;

  return result;
}

/**
 * @brief 使用全向底盘速度模型积分一个仿真步。
 *
 * vx、vy 表示机器人自身坐标系下的前向和横向速度，函数根据当前 yaw 将其转换到世界
 * 坐标系并更新 x、y，同时积分角速度 wz。
 *
 * @param state 当前仿真状态。
 * @param velocity 当前速度采样。
 * @param dt 仿真步长，单位秒。
 * @return 下一仿真状态。
 */
OmniLocalPlanner::SimState OmniLocalPlanner::simulateStep(
  const SimState & state,
  const VelocitySample & velocity,
  double dt) const
{
  SimState next = state;
  next.x += (std::cos(state.yaw) * velocity.vx - std::sin(state.yaw) * velocity.vy) * dt;
  next.y += (std::sin(state.yaw) * velocity.vx + std::cos(state.yaw) * velocity.vy) * dt;
  next.yaw = normalizedAngle(state.yaw + velocity.wz * dt);
  return next;
}

/**
 * @brief 检查仿真状态是否与局部代价地图发生碰撞。
 *
 * 该实现检查机器人中心点所在栅格，并依赖 costmap inflation layer 提供安全缓冲。
 * 如果状态落在 costmap 外部，会按碰撞处理，避免规划器驶出局部地图范围。
 *
 * 注意：膨胀层中的高代价区域不应全部视为硬碰撞。否则机器人离墙仍有一段距离时，
 * 轨迹会因为 cost 超过 obstacle_threshold_ 被整体丢弃，表现为“贴墙前卡住”。
 * 因此这里仅把 INSCRIBED/LETHAL 级别视为碰撞，普通膨胀代价只进入 obstacle_score，
 * 交给轨迹评分去偏好更远离障碍的候选速度。
 *
 * @param state 待检查的仿真状态。
 * @param obstacle_score 输出归一化障碍代价，范围约为 [0, 1]。
 * @return true 表示碰撞或不可通行；false 表示可通行。
 */
bool OmniLocalPlanner::stateInCollision(const SimState & state, double & obstacle_score) const
{
  obstacle_score = 0.0;
  if (costmap_ == nullptr) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(state.x, state.y, mx, my)) {
    return true;
  }

  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown_;
  }
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return true;
  }

  const double normalized_cost = static_cast<double>(cost) /
    static_cast<double>(std::max(1, static_cast<int>(obstacle_threshold_)));
  obstacle_score = std::clamp(normalized_cost, 0.0, 1.0);
  return false;
}

/**
 * @brief 计算仿真状态到局部路径的最近二维距离。
 *
 * 该距离用于鼓励候选轨迹贴近全局路径。如果局部路径为空，则返回 0，避免因 TF 转换
 * 短时失败导致所有轨迹被额外惩罚。
 *
 * @param state 仿真状态。
 * @param local_plan 当前坐标系下的局部路径。
 * @return 到局部路径最近点的距离。
 */
double OmniLocalPlanner::distanceToPlan(
  const SimState & state,
  const std::vector<geometry_msgs::msg::PoseStamped> & local_plan) const
{
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & pose : local_plan) {
    nearest_distance = std::min(
      nearest_distance,
      std::hypot(state.x - pose.pose.position.x, state.y - pose.pose.position.y));
  }

  if (!std::isfinite(nearest_distance)) {
    return 0.0;
  }
  return nearest_distance;
}

/**
 * @brief 计算速度目标代价。
 *
 * 候选速度模长越接近目标速度，代价越小。接近终点时 target_speed 会降低，从而鼓励
 * 规划器自然减速。
 *
 * @param vx 机器人坐标系 x 方向速度。
 * @param vy 机器人坐标系 y 方向速度。
 * @param target_speed 期望平移速度。
 * @return 速度偏差绝对值。
 */
double OmniLocalPlanner::scoreVelocity(double vx, double vy, double target_speed) const
{
  return std::abs(target_speed - std::hypot(vx, vy));
}

/**
 * @brief 计算当前生效的期望速度。
 *
 * 基础速度由 desired_linear_vel_ 和 max_linear_vel_ 共同限制；若外部设置了 speed limit，
 * 则继续按绝对值或百分比限制。
 *
 * @return 当前速度上限下的期望平移速度。
 */
double OmniLocalPlanner::limitedSpeed() const
{
  double speed = std::min(desired_linear_vel_, max_linear_vel_);
  if (active_speed_limit_ > 0.0) {
    const double limit = speed_limit_is_percentage_ ?
      max_linear_vel_ * active_speed_limit_ / 100.0 : active_speed_limit_;
    speed = std::min(speed, std::max(0.0, limit));
  }
  return speed;
}

/**
 * @brief 生成原地旋转速度命令。
 *
 * 当前进模式下车头与路径方向偏差过大时，先输出角速度对齐路径方向。角速度同样受
 * 当前速度和角加速度限制约束，避免控制命令突变。
 *
 * @param header 输出速度命令使用的 header。
 * @param heading_error 目标朝向与当前朝向的误差。
 * @param velocity 当前机器人速度。
 * @return 原地旋转速度命令。
 */
geometry_msgs::msg::TwistStamped OmniLocalPlanner::rotateCommand(
  const std_msgs::msg::Header & header,
  double heading_error,
  const geometry_msgs::msg::Twist & velocity) const
{
  const double sign = heading_error >= 0.0 ? 1.0 : -1.0;
  const double target_wz = sign * std::min(
    rotate_to_heading_angular_vel_,
    std::max(0.2, std::abs(heading_error)));
  const double min_wz = std::max(
    min_angular_vel_,
    velocity.angular.z - acc_lim_theta_ * controller_period_);
  const double max_wz = std::min(
    max_angular_vel_,
    velocity.angular.z + acc_lim_theta_ * controller_period_);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.header.stamp = clock_->now();
  cmd.twist.angular.z = std::clamp(target_wz, min_wz, max_wz);
  return cmd;
}

/**
 * @brief 生成零速度命令。
 *
 * 用于无路径、已到达目标、TF 失败或无有效轨迹等需要安全停车的场景。
 *
 * @param header 输出速度命令使用的 header。
 * @return 线速度和角速度均为 0 的速度命令。
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
 * @brief 从 PoseStamped 的四元数姿态中提取 yaw。
 *
 * @param pose 输入位姿。
 * @return yaw 角，单位弧度。
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
 * @brief 将角度归一化到 [-pi, pi]。
 *
 * @param angle 输入角度，单位弧度。
 * @return 归一化后的角度。
 */
double OmniLocalPlanner::normalizedAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief 计算两个位姿之间的二维欧氏距离。
 *
 * @param a 第一个位姿。
 * @param b 第二个位姿。
 * @return x-y 平面距离。
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
