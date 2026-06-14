#include "ros2_all_wheel_sim/local_planner/dwa_trace_logger.hpp"

#include <algorithm>
#include <filesystem>

#include "nav2_costmap_2d/cost_values.hpp"

namespace ros2_all_wheel_sim
{
namespace local_planner
{
namespace dwa
{

TraceLogger::~TraceLogger()
{
  close();
}

/**
 * @brief 设置 trace 输出参数。
 *
 * @param options 来自 Nav2 参数或离线工具的记录配置。
 */
void TraceLogger::configure(const TraceLoggerOptions & options)
{
  options_ = options;
  normalizeOptions();
}

/**
 * @brief 打开 CSV 文件并写入表头。
 *
 * @param costmap Nav2 局部代价地图；可为空，空时仅跳过 costmap 快照。
 * @return true 表示文件已成功打开，false 表示禁用或打开失败。
 */
bool TraceLogger::open(nav2_costmap_2d::Costmap2D * costmap)
{
  close();
  costmap_ = costmap;
  step_ = 0;
  path_dirty_ = true;
  if (!options_.enabled) {
    return false;
  }

  std::filesystem::create_directories(options_.directory);
  trace_file_.open(std::filesystem::path(options_.directory) / "dwa_trace.csv", std::ios::out);
  world_file_.open(std::filesystem::path(options_.directory) / "dwa_world.csv", std::ios::out);
  if (!trace_file_.is_open() || !world_file_.is_open()) {
    close();
    return false;
  }

  world_file_ << "kind,x,y,yaw,radius,width,height\n";
  trace_file_ << "step,kind,candidate_id,index,x,y,yaw,vx,vy,wz,total_score,"
              << "path_distance,target_distance,goal_distance,obstacle_score,"
              << "heading_error,velocity_error,valid\n";
  writeWorldBounds();
  return true;
}

/**
 * @brief 关闭已打开的 CSV 文件。
 */
void TraceLogger::close()
{
  if (trace_file_.is_open()) {
    trace_file_.close();
  }
  if (world_file_.is_open()) {
    world_file_.close();
  }
}

/**
 * @brief 当前记录器是否处于可写状态。
 *
 * @return true 表示 trace 文件已打开且可写。
 */
bool TraceLogger::enabled() const
{
  return options_.enabled && trace_file_.is_open();
}

/**
 * @brief 标记参考路径发生变化。
 *
 * 下一次 recordStep() 会把新的局部参考路径写入 trace。
 */
void TraceLogger::markPathDirty()
{
  path_dirty_ = true;
}

/**
 * @brief 记录一个真实控制周期的 DWA 回放帧。
 *
 * @param input 当前规划输入，来自真实 Nav2 控制周期。
 * @param result DWA core 计算结果。
 * @param transformed_path 当前控制坐标系下的参考路径。
 */
void TraceLogger::recordStep(
  const PlanInput & input,
  const PlanResult & result,
  const std::vector<Pose2D> & transformed_path)
{
  if (!enabled()) {
    ++step_;
    return;
  }
  if (step_ % options_.every_n != 0) {
    ++step_;
    return;
  }

  const int output_step = step_ / options_.every_n;
  Trajectory robot_score = result.best;
  robot_score.velocity = input.velocity;
  writeState(output_step, "robot", -1, 0, input.pose, input.velocity, robot_score);
  writePathIfNeeded(output_step, transformed_path);
  writeCandidates(output_step, result);
  writePrediction(output_step, input, result);
  writeCostmap(output_step);
  ++step_;
}

/**
 * @brief 规整 trace 配置，避免非法降采样参数。
 */
void TraceLogger::normalizeOptions()
{
  options_.every_n = std::max(1, options_.every_n);
  options_.candidate_stride = std::max(1, options_.candidate_stride);
  options_.candidate_state_stride = std::max(1, options_.candidate_state_stride);
  options_.costmap_stride = std::max(1, options_.costmap_stride);
  options_.cost_threshold = std::clamp(
    options_.cost_threshold, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE));
}

/**
 * @brief 写入当前 costmap 边界信息。
 */
void TraceLogger::writeWorldBounds()
{
  if (!world_file_.is_open() || costmap_ == nullptr) {
    return;
  }

  const double origin_x = costmap_->getOriginX();
  const double origin_y = costmap_->getOriginY();
  world_file_ << "bounds," << origin_x << ',' << origin_y << ",0,0,"
              << costmap_->getSizeInMetersX() << ',' << costmap_->getSizeInMetersY() << '\n';
}

/**
 * @brief 在路径变化后写入当前参考路径。
 *
 * @param output_step trace 输出帧编号。
 * @param transformed_path 当前控制坐标系下的参考路径。
 */
void TraceLogger::writePathIfNeeded(int output_step, const std::vector<Pose2D> & transformed_path)
{
  if (!path_dirty_) {
    return;
  }

  for (std::size_t i = 0; i < transformed_path.size(); ++i) {
    writeState(
      output_step, "path", -1, static_cast<int>(i), transformed_path[i],
      Velocity{}, Trajectory{});
  }
  path_dirty_ = false;
}

/**
 * @brief 写入降采样后的有效候选轨迹。
 *
 * @param output_step trace 输出帧编号。
 * @param result DWA 规划结果。
 */
void TraceLogger::writeCandidates(int output_step, const PlanResult & result)
{
  for (std::size_t candidate_id = 0; candidate_id < result.candidates.size(); ++candidate_id) {
    const auto & candidate = result.candidates[candidate_id];
    if (!candidate.valid ||
      candidate_id % static_cast<std::size_t>(options_.candidate_stride) != 0)
    {
      continue;
    }
    for (std::size_t i = 0; i < candidate.states.size();
      i += static_cast<std::size_t>(options_.candidate_state_stride))
    {
      writeState(
        output_step, "candidate", static_cast<int>(candidate_id), static_cast<int>(i),
        candidate.states[i], candidate.velocity, candidate);
    }
  }
}

/**
 * @brief 写入当前最优预测轨迹。
 *
 * @param output_step trace 输出帧编号。
 * @param input 当前控制周期输入。
 * @param result DWA 规划结果。
 */
void TraceLogger::writePrediction(
  int output_step,
  const PlanInput & input,
  const PlanResult & result)
{
  if (result.best.states.empty()) {
    writeState(output_step, "prediction", -1, 0, input.pose, result.command, result.best);
    return;
  }

  for (std::size_t i = 0; i < result.best.states.size(); ++i) {
    writeState(
      output_step, "prediction", -1, static_cast<int>(i), result.best.states[i],
      result.best.velocity, result.best);
  }
}

/**
 * @brief 写入降采样后的局部 costmap 障碍快照。
 *
 * @param output_step trace 输出帧编号。
 */
void TraceLogger::writeCostmap(int output_step)
{
  if (costmap_ == nullptr) {
    return;
  }

  int index = 0;
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const auto stride = static_cast<unsigned int>(options_.costmap_stride);
  for (unsigned int mx = 0; mx < size_x; mx += stride) {
    for (unsigned int my = 0; my < size_y; my += stride) {
      const unsigned char cost = costmap_->getCost(mx, my);
      if (cost < static_cast<unsigned char>(options_.cost_threshold)) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      costmap_->mapToWorld(mx, my, wx, wy);
      Trajectory cost_score;
      cost_score.valid = true;
      cost_score.score = static_cast<double>(cost);
      cost_score.obstacle_score = static_cast<double>(cost) /
        static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
      writeState(output_step, "costmap", -1, index++, Pose2D{wx, wy, 0.0}, Velocity{}, cost_score);
    }
  }
}

/**
 * @brief 写入一行统一格式的 trace 状态。
 *
 * @param step trace 输出帧编号。
 * @param kind 数据类型。
 * @param candidate_id 候选轨迹编号。
 * @param index 同一数据类型内的点索引。
 * @param pose 二维位姿。
 * @param velocity 速度。
 * @param trajectory 轨迹评分信息。
 */
void TraceLogger::writeState(
  int step,
  const std::string & kind,
  int candidate_id,
  int index,
  const Pose2D & pose,
  const Velocity & velocity,
  const Trajectory & trajectory)
{
  if (!trace_file_.is_open()) {
    return;
  }

  trace_file_ << step << ',' << kind << ',' << candidate_id << ',' << index << ','
              << pose.x << ',' << pose.y << ',' << pose.yaw << ','
              << velocity.vx << ',' << velocity.vy << ',' << velocity.wz << ','
              << trajectory.score << ',' << trajectory.path_distance << ','
              << trajectory.target_distance << ',' << trajectory.goal_distance << ','
              << trajectory.obstacle_score << ',' << trajectory.heading_error << ','
              << trajectory.velocity_error << ',' << (trajectory.valid ? 1 : 0) << '\n';
}

}  // namespace dwa
}  // namespace local_planner
}  // namespace ros2_all_wheel_sim
