#ifndef ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__OMNI_LOCAL_PLANNER_HPP_
#define ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__OMNI_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "ros2_all_wheel_sim/local_planner/dwa_core.hpp"
#include "ros2_all_wheel_sim/local_planner/dwa_trace_logger.hpp"

namespace ros2_all_wheel_sim
{
namespace local_planner
{

class OmniLocalPlanner : public nav2_core::Controller
{
public:
  OmniLocalPlanner() = default;
  ~OmniLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  /**
   * @brief 查找当前机器人在全局路径上的最近点。
   *
   * @param pose 当前机器人位姿。
   * @return 最近路径点索引。
   */
  std::size_t nearestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief 将路径点转换到目标坐标系。
   *
   * @param input 输入位姿。
   * @param target_frame 目标坐标系。
   * @param output 转换后的位姿。
   * @return true 表示转换成功。
   */
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & output) const;

  /**
   * @brief 判断机器人是否到达目标。
   *
   * @param pose 当前机器人位姿。
   * @param velocity 当前机器人速度。
   * @param goal_checker Nav2 目标检查器。
   * @return true 表示目标已到达。
   */
  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) const;

  /**
   * @brief 从最近点开始生成已转换的局部参考路径。
   *
   * @param pose 当前机器人位姿。
   * @param nearest_index 最近路径点索引。
   * @return 当前控制坐标系下的局部路径。
   */
  std::vector<geometry_msgs::msg::PoseStamped> transformPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    std::size_t nearest_index) const;

  /**
   * @brief 查询 Nav2 costmap 并转换为 DWA core 的代价结果。
   *
   * @param state 待检查的二维状态。
   * @return 碰撞状态和归一化障碍代价。
   */
  dwa::CostQuery queryCostmap(const dwa::Pose2D & state) const;

  /**
   * @brief 将 Nav2 参数聚合成纯算法配置。
   *
   * @return DWA core 配置。
   */
  dwa::Config plannerConfigFromParameters() const;

  /**
   * @brief 将 ROS PoseStamped 转换为与 ROS 无关的二维位姿。
   *
   * @param pose ROS 位姿。
   * @return DWA 二维位姿。
   */
  dwa::Pose2D toDwaPose(const geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief 构造原地旋转命令。
   *
   * @param header 输出消息头。
   * @param heading_error 目标朝向与当前朝向误差。
   * @param velocity 当前机器人速度。
   * @return Nav2 速度命令。
   */
  geometry_msgs::msg::TwistStamped rotateCommand(
    const std_msgs::msg::Header & header,
    double heading_error,
    const geometry_msgs::msg::Twist & velocity) const;

  /**
   * @brief 构造安全停车命令。
   *
   * @param header 输出消息头。
   * @return 零速度命令。
   */
  geometry_msgs::msg::TwistStamped zeroCommand(const std_msgs::msg::Header & header) const;

  /**
   * @brief 从四元数中提取 yaw。
   *
   * @param pose ROS 位姿。
   * @return yaw 角，单位 rad。
   */
  double poseYaw(const geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief 将角度归一化到 [-pi, pi]。
   *
   * @param angle 输入角度，单位 rad。
   * @return 归一化后的角度。
   */
  double normalizedAngle(double angle) const;

  /**
   * @brief 计算两个 ROS 位姿的二维距离。
   *
   * @param a 第一个位姿。
   * @param b 第二个位姿。
   * @return 二维欧氏距离。
   */
  double distance2D(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const;

  /**
   * @brief Nav2 生命周期节点弱引用。
   */
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  /**
   * @brief TF 缓冲区。
   */
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * @brief Nav2 costmap 封装对象。
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief Nav2 原始 costmap 指针。
   */
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  /**
   * @brief 当前插件使用的 ROS 日志器。
   */
  rclcpp::Logger logger_{rclcpp::get_logger("OmniLocalPlanner")};

  /**
   * @brief 当前插件使用的 ROS 时钟。
   */
  rclcpp::Clock::SharedPtr clock_;

  /**
   * @brief 与 ROS 解耦的 DWA 纯算法对象。
   */
  dwa::Planner planner_;

  /**
   * @brief 真实运行 DWA trace 记录器。
   */
  dwa::TraceLogger trace_logger_;

  /**
   * @brief Nav2 插件实例名。
   */
  std::string name_;

  /**
   * @brief 局部 costmap 坐标系名称。
   */
  std::string costmap_frame_;

  /**
   * @brief 最近一次 Nav2 下发的全局路径。
   */
  nav_msgs::msg::Path global_plan_;

  /**
   * @brief DWA 期望巡航线速度。
   */
  double desired_linear_vel_{0.35};

  /**
   * @brief 最大前向速度。
   */
  double max_linear_vel_{0.45};

  /**
   * @brief 最大横向速度。
   */
  double max_lateral_vel_{0.45};

  /**
   * @brief 最大角速度。
   */
  double max_angular_vel_{1.2};

  /**
   * @brief 最小前向速度。
   */
  double min_linear_vel_{-0.25};

  /**
   * @brief 最小横向速度。
   */
  double min_lateral_vel_{-0.25};

  /**
   * @brief 最小角速度。
   */
  double min_angular_vel_{-1.2};

  /**
   * @brief 前向加速度限制。
   */
  double acc_lim_x_{2.5};

  /**
   * @brief 横向加速度限制。
   */
  double acc_lim_y_{2.5};

  /**
   * @brief 角加速度限制。
   */
  double acc_lim_theta_{3.2};

  /**
   * @brief DWA 前向仿真时长。
   */
  double sim_time_{1.5};

  /**
   * @brief DWA 前向仿真步长。
   */
  double sim_step_{0.1};

  /**
   * @brief 控制周期。
   */
  double controller_period_{0.1};

  /**
   * @brief 接近目标时允许的最小前向速度。
   */
  double min_approach_linear_vel_{0.04};

  /**
   * @brief 路径前瞻距离。
   */
  double lookahead_dist_{0.45};

  /**
   * @brief 接近目标时开始降速的距离。
   */
  double approach_dist_{0.7};

  /**
   * @brief 目标位置容差。
   */
  double xy_goal_tolerance_{0.18};

  /**
   * @brief 目标朝向容差。
   */
  double yaw_goal_tolerance_{0.08};

  /**
   * @brief TF 查询容差。
   */
  double transform_tolerance_{0.2};

  /**
   * @brief 路径距离评分权重。
   */
  double path_distance_weight_{8.0};

  /**
   * @brief 前瞻点距离评分权重。
   */
  double target_distance_weight_{10.0};

  /**
   * @brief 终点距离评分权重。
   */
  double goal_distance_weight_{5.0};

  /**
   * @brief 障碍代价评分权重。
   */
  double obstacle_weight_{6.0};

  /**
   * @brief 朝向误差评分权重。
   */
  double heading_weight_{2.0};

  /**
   * @brief 速度误差评分权重。
   */
  double velocity_weight_{1.0};

  /**
   * @brief 非目标位置处的最小平移速度。
   */
  double min_trans_vel_{0.05};

  /**
   * @brief 触发原地对齐路径方向的最小角度。
   */
  double rotate_to_heading_min_angle_{0.35};

  /**
   * @brief 原地对齐路径方向时的目标角速度。
   */
  double rotate_to_heading_angular_vel_{0.8};

  /**
   * @brief 当前外部速度限制。
   */
  double active_speed_limit_{0.0};

  /**
   * @brief 前向速度采样数。
   */
  int vx_samples_{7};

  /**
   * @brief 横向速度采样数。
   */
  int vy_samples_{7};

  /**
   * @brief 角速度采样数。
   */
  int vtheta_samples_{15};

  /**
   * @brief true 表示 active_speed_limit_ 是百分比。
   */
  bool speed_limit_is_percentage_{false};

  /**
   * @brief true 表示大角度偏航时先对齐路径方向。
   */
  bool use_forward_only_{true};

  /**
   * @brief true 表示允许规划经过未知 costmap 区域。
   */
  bool allow_unknown_{false};

  /**
   * @brief 真实运行回放记录参数。
   */
  dwa::TraceLoggerOptions trace_options_;

  /**
   * @brief 归一化障碍代价时使用的 costmap 阈值。
   */
  unsigned char obstacle_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

}  // namespace local_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__OMNI_LOCAL_PLANNER_HPP_
