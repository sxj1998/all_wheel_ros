#ifndef ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_CORE_HPP_
#define ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

namespace ros2_all_wheel_sim
{
namespace local_planner
{
namespace dwa
{

/**
 * @brief 与 ROS 无关的二维位姿。
 */
struct Pose2D
{
  /**
   * @brief 世界/控制坐标系 x 坐标，单位 m。
   */
  double x{0.0};

  /**
   * @brief 世界/控制坐标系 y 坐标，单位 m。
   */
  double y{0.0};

  /**
   * @brief 机器人朝向，单位 rad。
   */
  double yaw{0.0};
};

/**
 * @brief 全向底盘速度采样。
 */
struct Velocity
{
  /**
   * @brief 机器人坐标系前向速度，单位 m/s。
   */
  double vx{0.0};

  /**
   * @brief 机器人坐标系横向速度，单位 m/s。
   */
  double vy{0.0};

  /**
   * @brief 角速度，单位 rad/s。
   */
  double wz{0.0};
};

/**
 * @brief DWA 纯算法配置。
 *
 * 该结构不包含 ROS 类型，因此可被 Nav2 插件、离线回放和其他可视化工具复用。
 */
struct Config
{
  /**
   * @brief 期望巡航线速度。
   */
  double desired_linear_vel{0.35};

  /**
   * @brief 最大前向速度。
   */
  double max_linear_vel{0.45};

  /**
   * @brief 最大横向速度。
   */
  double max_lateral_vel{0.45};

  /**
   * @brief 最大角速度。
   */
  double max_angular_vel{1.2};

  double min_linear_vel{-0.25};
  double min_lateral_vel{-0.25};
  double min_angular_vel{-1.2};

  /**
   * @brief 前向加速度限制。
   */
  double acc_lim_x{2.5};

  /**
   * @brief 横向加速度限制。
   */
  double acc_lim_y{2.5};

  /**
   * @brief 角加速度限制。
   */
  double acc_lim_theta{3.2};

  /**
   * @brief 每条候选轨迹的前向仿真时长。
   */
  double sim_time{1.5};

  /**
   * @brief 仿真积分步长。
   */
  double sim_step{0.1};

  /**
   * @brief 控制周期，用于动态窗口加速度约束。
   */
  double controller_period{0.1};

  double min_approach_linear_vel{0.04};

  /**
   * @brief 沿参考路径选取前瞻目标点的距离。
   */
  double lookahead_dist{0.45};

  /**
   * @brief 接近终点时开始降速的距离。
   */
  double approach_dist{0.7};

  /**
   * @brief 位置到达容差。
   */
  double xy_goal_tolerance{0.18};

  /**
   * @brief 朝向到达容差。
   */
  double yaw_goal_tolerance{0.08};

  /**
   * @brief 贴近参考路径权重。
   */
  double path_distance_weight{8.0};

  /**
   * @brief 接近前瞻点权重。
   */
  double target_distance_weight{10.0};

  /**
   * @brief 接近最终目标权重。
   */
  double goal_distance_weight{5.0};

  /**
   * @brief 远离障碍权重。
   */
  double obstacle_weight{6.0};

  /**
   * @brief 朝向误差权重。
   */
  double heading_weight{2.0};

  /**
   * @brief 速度目标误差权重。
   */
  double velocity_weight{1.0};

  /**
   * @brief 非终点状态下最小平移速度。
   */
  double min_trans_vel{0.05};

  double rotate_to_heading_min_angle{0.35};
  double rotate_to_heading_angular_vel{0.8};

  /**
   * @brief 前向速度采样数。
   */
  int vx_samples{7};

  /**
   * @brief 横向速度采样数。
   */
  int vy_samples{7};

  /**
   * @brief 角速度采样数。
   */
  int vtheta_samples{15};

  /**
   * @brief true 时大角度先原地对齐路径切线。
   */
  bool use_forward_only{true};
};

/**
 * @brief 外部环境查询结果。
 */
struct CostQuery
{
  /**
   * @brief true 表示该状态不可通行。
   */
  bool collision{false};

  /**
   * @brief 归一化障碍代价，越大越靠近障碍。
   */
  double obstacle_score{0.0};
};

/**
 * @brief 候选轨迹和各项评分。
 */
struct Trajectory
{
  /**
   * @brief 是否通过碰撞检查。
   */
  bool valid{false};

  /**
   * @brief 综合代价，越小越优。
   */
  double score{0.0};

  double path_distance{0.0};
  double target_distance{0.0};
  double goal_distance{0.0};
  double obstacle_score{0.0};
  double heading_error{0.0};
  double velocity_error{0.0};
  Velocity velocity;
  std::vector<Pose2D> states;
};

/**
 * @brief 单个控制周期的 DWA 输入。
 */
struct PlanInput
{
  /**
   * @brief 当前机器人位姿。
   */
  Pose2D pose;

  /**
   * @brief 当前机器人速度。
   */
  Velocity velocity;

  /**
   * @brief 当前控制坐标系下的参考路径。
   */
  std::vector<Pose2D> path;

  /**
   * @brief 外部速度限制；0 表示不限制。
   */
  double speed_limit{0.0};

  bool speed_limit_is_percentage{false};

  /**
   * @brief 上层 GoalChecker 已确认到达。
   */
  bool goal_reached{false};
};

/**
 * @brief 单个控制周期的 DWA 输出。
 */
struct PlanResult
{
  /**
   * @brief true 表示 command 可直接下发。
   */
  bool command_valid{false};

  bool rotate_to_heading{false};
  bool goal_reached{false};
  bool no_valid_trajectory{false};
  std::size_t nearest_index{0};
  std::size_t target_index{0};
  Pose2D target_pose;
  Pose2D goal_pose;
  double target_yaw{0.0};
  double target_speed{0.0};
  Velocity command;
  Trajectory best;
  std::vector<Trajectory> candidates;
};

using CostQueryCallback = std::function<CostQuery(const Pose2D &)>;

class Planner
{
public:
  explicit Planner(Config config = Config{});

  /**
   * @brief 更新算法参数，并执行边界归一化。
   *
   * @param config 新的 DWA 参数。
   */
  void setConfig(Config config);

  /**
   * @brief 读取当前归一化后的算法参数。
   *
   * @return 当前 DWA 参数。
   */
  const Config & config() const;

  /**
   * @brief 计算一个控制周期的 DWA 速度命令。
   *
   * @param input 当前控制周期输入。
   * @param cost_query 外部环境代价查询回调。
   * @return DWA 规划结果。
   */
  PlanResult plan(const PlanInput & input, const CostQueryCallback & cost_query) const;

  /**
   * @brief 根据朝向误差生成原地旋转命令。
   *
   * @param heading_error 目标朝向与当前朝向的误差。
   * @param current_velocity 当前机器人速度。
   * @return 原地旋转速度。
   */
  Velocity rotateCommand(double heading_error, const Velocity & current_velocity) const;

  /**
   * @brief 计算外部限速生效后的目标巡航速度。
   *
   * @param speed_limit 外部速度限制。
   * @param percentage true 表示 speed_limit 是百分比。
   * @return 生效后的目标速度。
   */
  double limitedSpeed(double speed_limit, bool percentage) const;

private:
  /**
   * @brief 已归一化的算法配置。
   */
  Config config_;

  std::size_t nearestPathIndex(const Pose2D & pose, const std::vector<Pose2D> & path) const;
  std::size_t lookaheadPathIndex(
    const std::vector<Pose2D> & path,
    std::size_t nearest_index) const;
  std::vector<double> sampleRange(double min_value, double max_value, int samples) const;
  Trajectory scoreTrajectory(
    const Pose2D & pose,
    const Velocity & velocity,
    const std::vector<Pose2D> & local_plan,
    const Pose2D & target_pose,
    const Pose2D & goal_pose,
    double target_yaw,
    double target_speed,
    const CostQueryCallback & cost_query) const;
  Pose2D simulateStep(const Pose2D & state, const Velocity & velocity, double dt) const;
  double distanceToPlan(const Pose2D & state, const std::vector<Pose2D> & local_plan) const;
  double scoreVelocity(double vx, double vy, double target_speed) const;
  void normalizeConfig();
};

double normalizeAngle(double angle);
double distance2D(const Pose2D & a, const Pose2D & b);

}  // namespace dwa
}  // namespace local_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__DWA_CORE_HPP_
