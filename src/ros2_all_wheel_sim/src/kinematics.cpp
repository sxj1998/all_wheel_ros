#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_all_wheel_sim/kinematics_core.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr const char * kOdomFrame = "odom";  ///< 本节点发布的里程计世界坐标系。
constexpr const char * kBaseFrame = "base_footprint";  ///< 机器人底盘坐标系。
constexpr const char * kCmdVelTopic = "cmd_vel";  ///< 接收期望底盘速度指令的话题。
constexpr const char * kJointStatesTopic = "joint_states";  ///< 轮子关节状态话题。
constexpr const char * kImuTopic = "imu";  ///< 从 Gazebo 桥接出的 IMU 数据话题。
constexpr const char * kOdomTopic = "odom";  ///< 导航和可视化模块使用的里程计话题。

namespace core = ros2_all_wheel_sim::core;

/**
 * @brief 生成指定轮子控制器的命令话题名。
 *
 * @param wheel_index 从 0 开始的轮子索引。
 * @return 控制器命令话题名，例如 `wheel1_controller/commands`。
 */
std::string wheelCommandTopic(int wheel_index)
{
  return "wheel" + std::to_string(wheel_index + 1) + "_controller/commands";
}

/**
 * @brief 将 yaw 角转换为 ROS 四元数消息。
 *
 * @param yaw 航向角，单位弧度。
 * @return 表示 roll = pitch = 0 且 yaw 为输入值的四元数。
 */
geometry_msgs::msg::Quaternion createYawQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}
}  // namespace

/**
 * @brief ROS2 适配层：连接平台无关运动学核心、控制器和 ROS 里程计。
 *
 * 该类只负责 ROS2 消息收发、TF 发布和消息类型转换。底层数学计算、
 * 轮速反解和里程计积分全部委托给 `ros2_all_wheel_sim::core`。
 */
class OmniKinematicsNode : public rclcpp::Node
{
public:
  /**
   * @brief 创建发布器、订阅器、TF 广播器和核心里程计对象。
   *
   * @param model 平台无关的机器人几何模型。
   */
  explicit OmniKinematicsNode(const core::RobotModel & model)
  : Node("omni_kinematics"),
    odometry_(core::OmniWheelKinematics(model), core::kDefaultModelDirectionSign)
  {
    createWheelNameIndex();
    createPublishers();
    createSubscriptions();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    last_joint_state_time_ = get_clock()->now();

    RCLCPP_INFO(get_logger(), "Omni kinematics node initialized with %d wheels", model.wheel_count);
  }

private:
  /**
   * @brief 创建从轮子关节名到轮子索引的查找表。
   *
   * ROS 的 `/joint_states` 中关节顺序不一定固定，该表用于把消息数据
   * 恢复成核心运动学要求的固定轮子顺序。
   */
  void createWheelNameIndex()
  {
    const auto & wheel_names = odometry_.kinematics().model().wheel_names;
    for (int i = 0; i < odometry_.kinematics().model().wheel_count; ++i) {
      wheel_index_by_joint_name_[wheel_names[static_cast<std::size_t>(i)]] = i;
    }
  }

  /**
   * @brief 创建轮子速度命令发布器和里程计发布器。
   */
  void createPublishers()
  {
    wheel_command_publishers_.reserve(odometry_.kinematics().model().wheel_count);
    for (int i = 0; i < odometry_.kinematics().model().wheel_count; ++i) {
      wheel_command_publishers_.push_back(
        create_publisher<std_msgs::msg::Float64MultiArray>(wheelCommandTopic(i), 10));
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, 10);
  }

  /**
   * @brief 创建速度指令、关节状态和 IMU 的订阅器。
   */
  void createSubscriptions()
  {
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      kCmdVelTopic,
      10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { onCmdVel(msg); });

    joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      kJointStatesTopic,
      10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointStates(msg); });

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      kImuTopic,
      10,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg); });
  }

  /**
   * @brief 处理期望底盘速度指令。
   *
   * @param msg 导航模块发布的 ROS Twist 速度指令。
   */
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const core::BodyTwist command{
      core::kDefaultModelDirectionSign * msg->linear.x,
      core::kDefaultModelDirectionSign * msg->linear.y,
      core::kDefaultModelDirectionSign * msg->angular.z};

    publishWheelCommands(odometry_.kinematics().calculateWheelSpeeds(command));
  }

  /**
   * @brief 处理轮子状态反馈并发布更新后的里程计。
   *
   * @param msg 包含轮子关节速度的 JointState 消息。
   */
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> wheel_speeds;
    if (!extractWheelSpeeds(*msg, wheel_speeds)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "JointState message does not contain all configured wheel velocities");
      return;
    }

    const rclcpp::Time current_time = get_clock()->now();
    const double dt = (current_time - last_joint_state_time_).seconds();
    last_joint_state_time_ = current_time;

    if (dt <= 0.0) {
      return;
    }

    const core::OdometryState & state = odometry_.integrate(wheel_speeds, dt);
    publishOdometry(current_time, state);
  }

  /**
   * @brief 处理 IMU 姿态和角速度更新。
   *
   * @param msg IMU 消息，其中姿态四元数用于提供里程计 yaw。
   */
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const tf2::Quaternion quaternion(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    odometry_.updateHeading(yaw, msg->angular_velocity.z);
  }

  /**
   * @brief 从 JointState 消息中提取配置好的轮子速度。
   *
   * @param msg 输入的 JointState 消息。
   * @param wheel_speeds 输出轮速数组，顺序与核心模型配置一致。
   * @return 如果所有配置轮子的速度都找到则返回 true，否则返回 false。
   */
  bool extractWheelSpeeds(
    const sensor_msgs::msg::JointState & msg,
    std::vector<double> & wheel_speeds) const
  {
    const int wheel_count = odometry_.kinematics().model().wheel_count;
    wheel_speeds.assign(static_cast<std::size_t>(wheel_count), 0.0);
    std::vector<bool> found(static_cast<std::size_t>(wheel_count), false);

    const std::size_t usable_size = std::min(msg.name.size(), msg.velocity.size());
    for (std::size_t i = 0; i < usable_size; ++i) {
      const auto joint = wheel_index_by_joint_name_.find(msg.name[i]);
      if (joint == wheel_index_by_joint_name_.end()) {
        continue;
      }

      wheel_speeds[static_cast<std::size_t>(joint->second)] = msg.velocity[i];
      found[static_cast<std::size_t>(joint->second)] = true;
    }

    for (bool wheel_found : found) {
      if (!wheel_found) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 向所有轮子控制器发布角速度命令。
   *
   * @param wheel_speeds 按配置轮子顺序排列的轮子角速度。
   */
  void publishWheelCommands(const std::vector<double> & wheel_speeds)
  {
    for (std::size_t i = 0; i < wheel_speeds.size(); ++i) {
      std_msgs::msg::Float64MultiArray message;
      message.data = {wheel_speeds[i]};
      wheel_command_publishers_[i]->publish(message);
    }
  }

  /**
   * @brief 使用核心层里程计状态发布 ROS Odometry 和 TF。
   *
   * @param stamp 里程计消息和对应 TF 使用的时间戳。
   * @param state 核心层计算得到的当前里程计状态。
   */
  void publishOdometry(const rclcpp::Time & stamp, const core::OdometryState & state)
  {
    const geometry_msgs::msg::Quaternion orientation = createYawQuaternion(state.pose.yaw);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = kOdomFrame;
    odom_msg.child_frame_id = kBaseFrame;
    odom_msg.pose.pose.position.x = state.pose.x;
    odom_msg.pose.pose.position.y = state.pose.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = orientation;
    odom_msg.twist.twist.linear.x = state.linear_x;
    odom_msg.twist.twist.linear.y = state.linear_y;
    odom_msg.twist.twist.angular.z = state.angular_z;

    odom_publisher_->publish(odom_msg);
    publishOdometryTransform(stamp, state, orientation);
  }

  /**
   * @brief 发布当前里程计位姿对应的 TF 坐标变换。
   *
   * @param stamp TF 消息使用的时间戳。
   * @param state 核心层计算得到的当前里程计状态。
   * @param orientation 已在 odom 消息中使用的姿态四元数。
   */
  void publishOdometryTransform(
    const rclcpp::Time & stamp,
    const core::OdometryState & state,
    const geometry_msgs::msg::Quaternion & orientation)
  {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = stamp;
    odom_tf.header.frame_id = kOdomFrame;
    odom_tf.child_frame_id = kBaseFrame;
    odom_tf.transform.translation.x = state.pose.x;
    odom_tf.transform.translation.y = state.pose.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(odom_tf);
  }

  core::OmniWheelOdometry odometry_;  ///< 平台无关的运动学和里程计核心。

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr>
    wheel_command_publishers_;  ///< 轮子速度命令发布器列表。

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;  ///< 里程计发布器。

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_subscription_;  ///< 期望底盘速度指令订阅器。

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_states_subscription_;  ///< 轮子关节速度反馈订阅器。

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    imu_subscription_;  ///< IMU 姿态和角速度订阅器。

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< TF 广播器。

  std::unordered_map<std::string, int> wheel_index_by_joint_name_;  ///< 关节名到轮子索引的映射。

  rclcpp::Time last_joint_state_time_;  ///< 上一次关节状态更新时间戳。
};

/**
 * @brief 程序入口函数。
 *
 * 创建当前 all_wheel 三轮模型，并启动 ROS2 适配节点。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniKinematicsNode>(core::makeDefaultAllWheelModel()));
  rclcpp::shutdown();
  return 0;
}
