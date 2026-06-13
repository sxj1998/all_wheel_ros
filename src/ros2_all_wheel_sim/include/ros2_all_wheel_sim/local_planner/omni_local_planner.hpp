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
  struct SimState
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct VelocitySample
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct TrajectoryScore
  {
    bool valid{false};
    double score{0.0};
    double obstacle_score{0.0};
    VelocitySample velocity;
  };

  std::size_t nearestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const;
  std::size_t lookaheadPathIndex(std::size_t nearest_index) const;
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & output) const;
  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) const;

  std::vector<geometry_msgs::msg::PoseStamped> transformPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    std::size_t nearest_index) const;
  std::vector<double> sampleRange(double min_value, double max_value, int samples) const;
  TrajectoryScore scoreTrajectory(
    const geometry_msgs::msg::PoseStamped & pose,
    const VelocitySample & velocity,
    const std::vector<geometry_msgs::msg::PoseStamped> & local_plan,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double target_yaw,
    double target_speed) const;
  SimState simulateStep(const SimState & state, const VelocitySample & velocity, double dt) const;
  bool stateInCollision(const SimState & state, double & obstacle_score) const;
  double distanceToPlan(
    const SimState & state,
    const std::vector<geometry_msgs::msg::PoseStamped> & local_plan) const;
  double scoreVelocity(double vx, double vy, double target_speed) const;
  double limitedSpeed() const;
  geometry_msgs::msg::TwistStamped rotateCommand(
    const std_msgs::msg::Header & header,
    double heading_error,
    const geometry_msgs::msg::Twist & velocity) const;
  geometry_msgs::msg::TwistStamped zeroCommand(const std_msgs::msg::Header & header) const;

  double poseYaw(const geometry_msgs::msg::PoseStamped & pose) const;
  double normalizedAngle(double angle) const;
  double distance2D(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("OmniLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  std::string name_;
  std::string costmap_frame_;
  nav_msgs::msg::Path global_plan_;

  double desired_linear_vel_{0.35};
  double max_linear_vel_{0.45};
  double max_lateral_vel_{0.45};
  double max_angular_vel_{1.2};
  double min_linear_vel_{-0.25};
  double min_lateral_vel_{-0.25};
  double min_angular_vel_{-1.2};
  double acc_lim_x_{2.5};
  double acc_lim_y_{2.5};
  double acc_lim_theta_{3.2};
  double sim_time_{1.5};
  double sim_step_{0.1};
  double controller_period_{0.1};
  double min_approach_linear_vel_{0.04};
  double lookahead_dist_{0.45};
  double approach_dist_{0.7};
  double xy_goal_tolerance_{0.18};
  double yaw_goal_tolerance_{0.08};
  double transform_tolerance_{0.2};
  double path_distance_weight_{8.0};
  double target_distance_weight_{10.0};
  double goal_distance_weight_{5.0};
  double obstacle_weight_{6.0};
  double heading_weight_{2.0};
  double velocity_weight_{1.0};
  double min_trans_vel_{0.05};
  double rotate_to_heading_min_angle_{0.35};
  double rotate_to_heading_angular_vel_{0.8};
  double active_speed_limit_{0.0};
  int vx_samples_{7};
  int vy_samples_{7};
  int vtheta_samples_{15};
  bool speed_limit_is_percentage_{false};
  bool use_forward_only_{true};
  bool allow_unknown_{false};
  unsigned char obstacle_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

}  // namespace local_planner
}  // namespace ros2_all_wheel_sim

#endif  // ROS2_ALL_WHEEL_SIM__LOCAL_PLANNER__OMNI_LOCAL_PLANNER_HPP_
