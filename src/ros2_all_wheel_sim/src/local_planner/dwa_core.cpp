#include "ros2_all_wheel_sim/local_planner/dwa_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ros2_all_wheel_sim
{
namespace local_planner
{
namespace dwa
{

/**
 * @brief 将角度归一化到 [-pi, pi]。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
double normalizeAngle(double angle)
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
 * @brief 计算二维欧氏距离。
 *
 * @param a 第一个二维位姿。
 * @param b 第二个二维位姿。
 * @return 两个位姿之间的二维距离，单位 m。
 */
double distance2D(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
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
 * @brief 执行一个控制周期的 DWA 搜索。
 *
 * 函数只处理纯算法数据：路径裁剪、动态窗口采样、前向仿真和加权评分。
 *
 * @param input 当前控制周期输入。
 * @param cost_query 外部环境代价查询回调。
 * @return 当前控制周期的 DWA 规划结果。
 */
PlanResult Planner::plan(const PlanInput & input, const CostQueryCallback & cost_query) const
{
  PlanResult result;
  result.command = Velocity{};
  result.target_speed = limitedSpeed(input.speed_limit, input.speed_limit_is_percentage);

  if (input.path.empty()) {
    return result;
  }

  /**
   * 终点和前瞻点共同决定短期轨迹方向，避免只盯最终目标导致切弯。
   */
  result.goal_pose = input.path.back();
  const double distance_to_goal = distance2D(input.pose, result.goal_pose);
  const double yaw_error = std::abs(normalizeAngle(result.goal_pose.yaw - input.pose.yaw));
  result.goal_reached =
    input.goal_reached ||
    (distance_to_goal <= config_.xy_goal_tolerance && yaw_error <= config_.yaw_goal_tolerance);
  if (result.goal_reached) {
    result.command_valid = true;
    return result;
  }

  result.nearest_index = nearestPathIndex(input.pose, input.path);
  result.target_index = lookaheadPathIndex(input.path, result.nearest_index);
  result.target_pose = input.path[result.target_index];

  if (distance_to_goal < config_.approach_dist) {
    const double scale = std::clamp(distance_to_goal / config_.approach_dist, 0.0, 1.0);
    result.target_speed = std::max(config_.min_approach_linear_vel, result.target_speed * scale);
  }

  result.target_yaw = result.goal_pose.yaw;
  if (result.target_index + 1 < input.path.size()) {
    const auto & next_pose = input.path[result.target_index + 1];
    const double path_dx = next_pose.x - result.target_pose.x;
    const double path_dy = next_pose.y - result.target_pose.y;
    if (std::hypot(path_dx, path_dy) > 1e-3) {
      result.target_yaw = std::atan2(path_dy, path_dx);
    }
  }

  /**
   * 朝向偏差过大时先原地旋转对齐路径切线。
   * 这让全向底盘在窄通道转向时少做贴墙平移，转向也更干脆。
   */
  const double heading_error = normalizeAngle(result.target_yaw - input.pose.yaw);
  if (config_.rotate_to_heading_enabled &&
    distance_to_goal > config_.xy_goal_tolerance &&
    std::abs(heading_error) > config_.rotate_to_heading_min_angle)
  {
    result.rotate_to_heading = true;
    result.command_valid = true;
    result.command = rotateCommand(heading_error, input.velocity);
    return result;
  }

  std::vector<Pose2D> local_plan;
  local_plan.reserve(input.path.size() - result.nearest_index);
  for (std::size_t i = result.nearest_index; i < input.path.size(); ++i) {
    local_plan.push_back(input.path[i]);
  }

  /**
   * 动态窗口：速度上下界同时受机械极限和当前速度加速度约束。
   */
  const double min_vx = config_.use_forward_only ?
    std::max(0.0, input.velocity.vx - config_.acc_lim_x * config_.controller_period) :
    std::max(config_.min_linear_vel, input.velocity.vx - config_.acc_lim_x * config_.controller_period);
  const double max_vx = std::min(
    config_.max_linear_vel, input.velocity.vx + config_.acc_lim_x * config_.controller_period);
  const double min_vy = std::max(
    config_.min_lateral_vel, input.velocity.vy - config_.acc_lim_y * config_.controller_period);
  const double max_vy = std::min(
    config_.max_lateral_vel, input.velocity.vy + config_.acc_lim_y * config_.controller_period);
  const double min_wz = std::max(
    config_.min_angular_vel, input.velocity.wz - config_.acc_lim_theta * config_.controller_period);
  const double max_wz = std::min(
    config_.max_angular_vel, input.velocity.wz + config_.acc_lim_theta * config_.controller_period);

  const auto vx_values = sampleRange(min_vx, max_vx, config_.vx_samples);
  const auto vy_values = sampleRange(min_vy, max_vy, config_.vy_samples);
  const auto wz_values = sampleRange(min_wz, max_wz, config_.vtheta_samples);

  /**
   * 穷举采样空间，保留综合代价最小的无碰撞轨迹。
   */
  result.best.score = std::numeric_limits<double>::infinity();
  for (const double vx : vx_values) {
    for (const double vy : vy_values) {
      for (const double wz : wz_values) {
        Velocity sample{vx, vy, wz};
        if (distance_to_goal <= config_.xy_goal_tolerance) {
          sample.vx = 0.0;
          sample.vy = 0.0;
        } else if (std::hypot(sample.vx, sample.vy) < config_.min_trans_vel) {
          continue;
        }

        auto trajectory = scoreTrajectory(
          input.pose, sample, local_plan, result.target_pose, result.goal_pose,
          result.target_yaw, result.target_speed, cost_query);
        result.candidates.push_back(trajectory);
        if (trajectory.valid && trajectory.score < result.best.score) {
          result.best = trajectory;
        }
      }
    }
  }

  if (!result.best.valid) {
    result.no_valid_trajectory = true;
    if (config_.use_forward_only && distance_to_goal > config_.xy_goal_tolerance) {
      result.rotate_to_heading = true;
      result.command_valid = true;
      result.command = rotateCommand(heading_error, input.velocity);
    }
    return result;
  }

  result.command_valid = true;
  result.command = result.best.velocity;
  return result;
}

/**
 * @brief 生成受角加速度约束的原地旋转速度。
 *
 * @param heading_error 目标朝向与当前朝向误差。
 * @param current_velocity 当前机器人速度。
 * @return 原地旋转速度命令。
 */
Velocity Planner::rotateCommand(double heading_error, const Velocity & current_velocity) const
{
  const double sign = heading_error >= 0.0 ? 1.0 : -1.0;
  const double target_wz = sign * std::min(
    config_.rotate_to_heading_angular_vel,
    std::max(0.2, std::abs(heading_error)));
  const double min_wz = std::max(
    config_.min_angular_vel,
    current_velocity.wz - config_.acc_lim_theta * config_.controller_period);
  const double max_wz = std::min(
    config_.max_angular_vel,
    current_velocity.wz + config_.acc_lim_theta * config_.controller_period);
  return Velocity{0.0, 0.0, std::clamp(target_wz, min_wz, max_wz)};
}

/**
 * @brief 合并基础巡航速度和外部限速。
 *
 * @param speed_limit 外部速度限制。
 * @param percentage true 表示 speed_limit 为百分比。
 * @return 限速后的目标巡航速度。
 */
double Planner::limitedSpeed(double speed_limit, bool percentage) const
{
  double speed = std::min(config_.desired_linear_vel, config_.max_linear_vel);
  if (speed_limit > 0.0) {
    const double limit = percentage ? config_.max_linear_vel * speed_limit / 100.0 : speed_limit;
    speed = std::min(speed, std::max(0.0, limit));
  }
  return speed;
}

/**
 * @brief 找到路径上距离机器人最近的点。
 *
 * @param pose 当前机器人位姿。
 * @param path 当前参考路径。
 * @return 最近路径点索引。
 */
std::size_t Planner::nearestPathIndex(const Pose2D & pose, const std::vector<Pose2D> & path) const
{
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = distance2D(pose, path[i]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  return nearest_index;
}

/**
 * @brief 从最近点沿路径累计距离，选取前瞻目标点。
 *
 * @param path 当前参考路径。
 * @param nearest_index 最近路径点索引。
 * @return 前瞻目标点索引。
 */
std::size_t Planner::lookaheadPathIndex(
  const std::vector<Pose2D> & path,
  std::size_t nearest_index) const
{
  double accumulated_distance = 0.0;
  std::size_t index = nearest_index;
  while (index + 1 < path.size() && accumulated_distance < config_.lookahead_dist) {
    accumulated_distance += distance2D(path[index], path[index + 1]);
    ++index;
  }
  return index;
}

/**
 * @brief 在闭区间上均匀采样。
 *
 * @param min_value 采样下界。
 * @param max_value 采样上界。
 * @param samples 采样数量。
 * @return 均匀采样结果。
 */
std::vector<double> Planner::sampleRange(double min_value, double max_value, int samples) const
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
 * @brief 对单个速度样本做前向仿真并计算综合代价。
 *
 * @param pose 当前机器人位姿。
 * @param velocity 待评估速度样本。
 * @param local_plan 当前局部参考路径。
 * @param target_pose 前瞻目标点。
 * @param goal_pose 最终目标点。
 * @param target_yaw 目标朝向。
 * @param target_speed 目标平移速度。
 * @param cost_query 外部环境代价查询回调。
 * @return 候选轨迹及评分。
 */
Trajectory Planner::scoreTrajectory(
  const Pose2D & pose,
  const Velocity & velocity,
  const std::vector<Pose2D> & local_plan,
  const Pose2D & target_pose,
  const Pose2D & goal_pose,
  double target_yaw,
  double target_speed,
  const CostQueryCallback & cost_query) const
{
  Trajectory trajectory;
  trajectory.velocity = velocity;
  trajectory.score = std::numeric_limits<double>::infinity();
  trajectory.states.push_back(pose);

  Pose2D state = pose;
  /**
   * 轨迹全程遇到的最高障碍代价。
   */
  double max_obstacle_score = 0.0;
  for (double time = 0.0; time < config_.sim_time; time += config_.sim_step) {
    state = simulateStep(state, velocity, config_.sim_step);
    trajectory.states.push_back(state);
    const CostQuery query = cost_query ? cost_query(state) : CostQuery{};
    if (query.collision) {
      return trajectory;
    }
    max_obstacle_score = std::max(max_obstacle_score, query.obstacle_score);
  }

  trajectory.valid = true;
  trajectory.obstacle_score = max_obstacle_score;
  trajectory.path_distance = distanceToPlan(state, local_plan);
  trajectory.target_distance = distance2D(state, target_pose);
  trajectory.goal_distance = distance2D(state, goal_pose);
  trajectory.heading_error = std::abs(normalizeAngle(target_yaw - state.yaw));
  trajectory.velocity_error = scoreVelocity(velocity.vx, velocity.vy, target_speed);
  trajectory.score =
    config_.path_distance_weight * trajectory.path_distance +
    config_.target_distance_weight * trajectory.target_distance +
    config_.goal_distance_weight * trajectory.goal_distance +
    config_.obstacle_weight * trajectory.obstacle_score +
    config_.heading_weight * trajectory.heading_error +
    config_.velocity_weight * trajectory.velocity_error;
  return trajectory;
}

/**
 * @brief 全向底盘运动模型积分一步。
 *
 * @param state 当前仿真状态。
 * @param velocity 当前速度样本。
 * @param dt 积分步长，单位 s。
 * @return 下一仿真状态。
 */
Pose2D Planner::simulateStep(const Pose2D & state, const Velocity & velocity, double dt) const
{
  Pose2D next = state;
  next.x += (std::cos(state.yaw) * velocity.vx - std::sin(state.yaw) * velocity.vy) * dt;
  next.y += (std::sin(state.yaw) * velocity.vx + std::cos(state.yaw) * velocity.vy) * dt;
  next.yaw = normalizeAngle(state.yaw + velocity.wz * dt);
  return next;
}

/**
 * @brief 计算状态到局部参考路径的最近距离。
 *
 * @param state 待评估状态。
 * @param local_plan 当前局部参考路径。
 * @return 状态到路径的最近距离，单位 m。
 */
double Planner::distanceToPlan(const Pose2D & state, const std::vector<Pose2D> & local_plan) const
{
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & pose : local_plan) {
    nearest_distance = std::min(nearest_distance, distance2D(state, pose));
  }
  return std::isfinite(nearest_distance) ? nearest_distance : 0.0;
}

/**
 * @brief 计算候选平移速度与目标速度的偏差。
 *
 * @param vx 机器人坐标系前向速度。
 * @param vy 机器人坐标系横向速度。
 * @param target_speed 目标平移速度。
 * @return 速度模长误差。
 */
double Planner::scoreVelocity(double vx, double vy, double target_speed) const
{
  return std::abs(target_speed - std::hypot(vx, vy));
}

/**
 * @brief 规整参数边界，避免非法采样范围进入规划过程。
 */
void Planner::normalizeConfig()
{
  config_.desired_linear_vel = std::max(0.0, config_.desired_linear_vel);
  config_.max_linear_vel = std::max(0.0, config_.max_linear_vel);
  config_.max_lateral_vel = std::max(0.0, config_.max_lateral_vel);
  config_.max_angular_vel = std::max(0.01, config_.max_angular_vel);
  config_.min_linear_vel = std::clamp(
    config_.min_linear_vel, -config_.max_linear_vel, config_.max_linear_vel);
  config_.min_lateral_vel = std::clamp(
    config_.min_lateral_vel, -config_.max_lateral_vel, config_.max_lateral_vel);
  config_.min_angular_vel = std::clamp(
    config_.min_angular_vel, -config_.max_angular_vel, config_.max_angular_vel);
  config_.acc_lim_x = std::max(0.01, config_.acc_lim_x);
  config_.acc_lim_y = std::max(0.01, config_.acc_lim_y);
  config_.acc_lim_theta = std::max(0.01, config_.acc_lim_theta);
  config_.min_approach_linear_vel = std::clamp(
    config_.min_approach_linear_vel, 0.0, config_.max_linear_vel);
  config_.sim_time = std::max(0.2, config_.sim_time);
  config_.sim_step = std::clamp(config_.sim_step, 0.02, config_.sim_time);
  config_.controller_period = std::max(0.01, config_.controller_period);
  config_.lookahead_dist = std::max(0.05, config_.lookahead_dist);
  config_.approach_dist = std::max(0.05, config_.approach_dist);
  config_.xy_goal_tolerance = std::max(0.0, config_.xy_goal_tolerance);
  config_.yaw_goal_tolerance = std::max(0.0, config_.yaw_goal_tolerance);
  config_.path_distance_weight = std::max(0.0, config_.path_distance_weight);
  config_.target_distance_weight = std::max(0.0, config_.target_distance_weight);
  config_.goal_distance_weight = std::max(0.0, config_.goal_distance_weight);
  config_.obstacle_weight = std::max(0.0, config_.obstacle_weight);
  config_.heading_weight = std::max(0.0, config_.heading_weight);
  config_.velocity_weight = std::max(0.0, config_.velocity_weight);
  config_.min_trans_vel = std::max(0.0, config_.min_trans_vel);
  config_.rotate_to_heading_min_angle = std::max(0.0, config_.rotate_to_heading_min_angle);
  config_.rotate_to_heading_angular_vel = std::clamp(
    config_.rotate_to_heading_angular_vel, 0.01, config_.max_angular_vel);
  config_.vx_samples = std::max(2, config_.vx_samples);
  config_.vy_samples = std::max(1, config_.vy_samples);
  config_.vtheta_samples = std::max(3, config_.vtheta_samples);
}

}  // namespace dwa
}  // namespace local_planner
}  // namespace ros2_all_wheel_sim
