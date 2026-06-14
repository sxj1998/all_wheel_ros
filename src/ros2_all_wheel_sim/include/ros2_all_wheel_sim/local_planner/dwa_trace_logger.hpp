#ifndef ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_TRACE_LOGGER_HPP_
#define ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_TRACE_LOGGER_HPP_

#include <fstream>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "ros2_all_wheel_sim/local_planner/dwa_core.hpp"

namespace ros2_all_wheel_sim
{
namespace local_planner
{
namespace dwa
{

/**
 * @brief DWA 运行时回放记录器配置。
 *
 * 该配置只影响调试数据落盘，不参与 DWA 规划打分和速度输出。
 */
struct TraceLoggerOptions
{
  /**
   * @brief 是否启用运行时 trace。
   */
  bool enabled{false};

  /**
   * @brief trace CSV 输出目录。
   */
  std::string directory{"dwa_trace"};

  /**
   * @brief 每 N 个控制周期记录一帧。
   */
  int every_n{1};

  /**
   * @brief 候选轨迹按编号降采样，降低文件体积。
   */
  int candidate_stride{8};

  /**
   * @brief 候选轨迹内部点降采样。
   */
  int candidate_state_stride{2};

  /**
   * @brief costmap 栅格降采样步长。
   */
  int costmap_stride{3};

  /**
   * @brief 只记录高于该代价的 costmap 点。
   */
  int cost_threshold{253};
};

/**
 * @brief 将真实运行中的 DWA 数据写成可离线回放的 CSV。
 *
 * 该类是一个小型策略/适配器：上层传入纯 DWA 数据和 costmap 指针，类内部负责文件格式、
 * 降采样和路径变更状态。这样 Nav2 插件不需要关心 CSV 细节。
 */
class TraceLogger
{
public:
  TraceLogger() = default;
  ~TraceLogger();

  /**
   * @brief 配置 trace 输出参数。
   *
   * @param options trace 输出配置。
   */
  void configure(const TraceLoggerOptions & options);

  /**
   * @brief 打开 trace 文件。
   *
   * @param costmap Nav2 局部代价地图。
   * @return true 表示文件打开成功。
   */
  bool open(nav2_costmap_2d::Costmap2D * costmap);

  /**
   * @brief 关闭 trace 文件。
   */
  void close();

  /**
   * @brief 判断当前是否可写 trace。
   *
   * @return true 表示 trace 已启用且文件可写。
   */
  bool enabled() const;

  /**
   * @brief 标记参考路径已变化。
   */
  void markPathDirty();

  /**
   * @brief 记录一个 DWA 控制周期。
   *
   * @param input 当前控制周期输入。
   * @param result DWA 规划结果。
   * @param transformed_path 当前控制坐标系下的参考路径。
   */
  void recordStep(
    const PlanInput & input,
    const PlanResult & result,
    const std::vector<Pose2D> & transformed_path);

private:
  void normalizeOptions();
  void writeWorldBounds();
  void writePathIfNeeded(int output_step, const std::vector<Pose2D> & transformed_path);
  void writeCandidates(int output_step, const PlanResult & result);
  void writePrediction(int output_step, const PlanInput & input, const PlanResult & result);
  void writeCostmap(int output_step);
  void writeState(
    int step,
    const std::string & kind,
    int candidate_id,
    int index,
    const Pose2D & pose,
    const Velocity & velocity,
    const Trajectory & trajectory);

  /**
   * @brief 当前 trace 输出策略。
   */
  TraceLoggerOptions options_;

  /**
   * @brief 只读 costmap 指针，由 Nav2 生命周期持有。
   */
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  /**
   * @brief 原始控制周期计数。
   */
  int step_{0};

  /**
   * @brief 路径变化标记，用于按需写入参考路径。
   */
  bool path_dirty_{true};

  /**
   * @brief 每帧轨迹/costmap 数据。
   */
  std::ofstream trace_file_;

  /**
   * @brief 世界边界等静态数据。
   */
  std::ofstream world_file_;
};

}  // namespace dwa
}  // namespace local_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_TRACE_LOGGER_HPP_
