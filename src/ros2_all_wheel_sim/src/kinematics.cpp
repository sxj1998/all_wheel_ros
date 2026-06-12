#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;  ///< 圆周率常量，用于角度与弧度转换。

constexpr double kWheelRadius = 0.035115;  ///< 全向轮半径，单位为米。

constexpr double kRobotRadius = 0.0990025403784439;  ///< 机器人中心到各轮接地点距离，单位为米。

constexpr const char * kOdomFrame = "odom";  ///< 本节点发布的里程计世界坐标系。

constexpr const char * kBaseFrame = "base_footprint";  ///< 机器人底盘坐标系。

constexpr const char * kCmdVelTopic = "cmd_vel";  ///< 接收期望底盘速度指令的话题。

constexpr const char * kJointStatesTopic = "joint_states";  ///< 轮子关节状态话题。

constexpr const char * kImuTopic = "imu";  ///< 从 Gazebo 桥接出的 IMU 数据话题。

constexpr const char * kOdomTopic = "odom";  ///< 导航和可视化模块使用的里程计话题。

/// ROS 底盘速度指令与 Gazebo 轮关节方向之间的符号补偿。
///
/// Gazebo 模型/控制器中的轮关节正方向与期望的底盘运动方向相反，
/// 因此这里保留原始实现中的统一符号补偿。
constexpr double kModelDirectionSign = -1.0;

/**
 * @brief 全向轮底盘的静态几何参数和命名描述。
 *
 * 该结构体只保存机器人模型数据和参数校验逻辑，不包含 ROS 通信行为。
 * 纯运动学类和 ROS 节点共享该模型，从而保证轮子顺序和几何尺寸一致。
 */
struct RobotModel
{
  int wheel_count = 0;  ///< 底盘中主动全向轮的数量。

  double robot_radius = 0.0;  ///< 机器人中心到轮子接地点的距离，单位为米。

  double wheel_radius = 0.0;  ///< 轮子的物理半径，单位为米。

  double heading_offset_deg = 0.0;  ///< 0 号轮相对机器人 +x 轴的角度偏移，单位为度。

  std::vector<std::string> wheel_joint_names;  ///< 轮子关节名，顺序与控制话题一致。

  /**
   * @brief 校验机器人模型尺寸和轮子元数据。
   *
   * @throws std::runtime_error 当轮子数量、半径或关节名数量不合法时抛出异常。
   */
  void validate() const
  {
    if (wheel_count <= 0) {
      throw std::runtime_error("Wheel count must be positive");
    }
    if (robot_radius <= 0.0 || wheel_radius <= 0.0) {
      throw std::runtime_error("Robot radius and wheel radius must be positive");
    }
    if (static_cast<int>(wheel_joint_names.size()) != wheel_count) {
      throw std::runtime_error("Wheel joint count does not match robot model");
    }
  }
};

/**
 * @brief 机器人机体系下的平面速度指令。
 */
struct BodyTwist
{
  double vx = 0.0;  ///< 沿机器人 +x 方向的线速度，单位 m/s。

  double vy = 0.0;  ///< 沿机器人 +y 方向的线速度，单位 m/s。

  double wz = 0.0;  ///< 绕机器人 +z 轴的角速度，单位 rad/s。
};

/**
 * @brief 用于里程计积分的最小二维位姿表示。
 */
struct Pose2D
{
  double x = 0.0;  ///< odom 坐标系下的 x 位置，单位为米。

  double y = 0.0;  ///< odom 坐标系下的 y 位置，单位为米。

  double yaw = 0.0;  ///< odom 坐标系下的航向角，单位为弧度。
};

/**
 * @brief 将角度制转换为弧度制。
 *
 * @param degrees 角度制数值。
 * @return 对应的弧度制数值。
 */
double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

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
}  // namespace

/**
 * @brief 等角度布置全向轮底盘的纯运动学模型。
 *
 * 该类不拥有任何 ROS 实体，只负责机器人底盘速度与轮子角速度之间的转换。
 * 使用的标准全向轮投影矩阵为：
 *
 * omega_i = [-sin(theta_i)/r, cos(theta_i)/r, R/r] * [vx, vy, wz]^T
 */
class OmniWheelKinematics
{
public:
  /**
   * @brief 根据机器人模型构造运动学矩阵。
   *
   * @param model 机器人几何参数和轮子关节顺序。
   * @throws std::runtime_error 当机器人模型参数不合法时抛出异常。
   */
  explicit OmniWheelKinematics(RobotModel model)
  : model_(std::move(model)),
    command_to_wheel_(buildCommandToWheelMatrix(model_)),
    wheel_to_body_xy_(pseudoInverse(command_to_wheel_).topRows(2))
  {
    model_.validate();
  }

  /**
   * @brief 将机体系速度指令转换为各轮角速度。
   *
   * @param twist 机体系下的平面速度指令。
   * @return 按配置轮子顺序排列的轮子角速度，单位 rad/s。
   */
  Eigen::VectorXd calculateWheelSpeeds(const BodyTwist & twist) const
  {
    const Eigen::Vector3d body_velocity(twist.vx, twist.vy, twist.wz);
    return command_to_wheel_ * body_velocity;
  }

  /**
   * @brief 根据测得的轮速估计机体系线速度。
   *
   * 这里只估计 vx 和 vy。yaw 角速度由 ROS 节点中的 IMU 提供，
   * 以避免轮速积分带来的角度漂移。
   *
   * @param wheel_speeds 按配置轮子顺序排列的轮子角速度。
   * @return 机体系下的线速度 [vx, vy]，单位 m/s。
   * @throws std::runtime_error 当轮速向量长度与机器人模型不匹配时抛出异常。
   */
  Eigen::Vector2d estimateBodyLinearVelocity(const Eigen::VectorXd & wheel_speeds) const
  {
    if (wheel_speeds.size() != model_.wheel_count) {
      throw std::runtime_error("Wheel speed vector size does not match robot model");
    }
    return wheel_to_body_xy_ * wheel_speeds;
  }

  /**
   * @brief 获取当前运动学对象使用的只读机器人模型。
   *
   * @return 机器人模型引用。
   */
  const RobotModel & model() const
  {
    return model_;
  }

private:
  /**
   * @brief 构造从底盘速度到轮子角速度的映射矩阵。
   *
   * @param model 已通过校验的机器人模型。
   * @return N x 3 矩阵，其中 N 为轮子数量。
   */
  static Eigen::MatrixXd buildCommandToWheelMatrix(const RobotModel & model)
  {
    model.validate();

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(model.wheel_count, 3);
    const double wheel_spacing_deg = 360.0 / static_cast<double>(model.wheel_count);

    for (int i = 0; i < model.wheel_count; ++i) {
      const double theta = degreesToRadians(wheel_spacing_deg * i + model.heading_offset_deg);

      // 对每个全向轮，有：
      // omega_i = [-sin(theta_i)/r, cos(theta_i)/r, R/r] * [vx, vy, wz]^T
      matrix(i, 0) = -std::sin(theta) / model.wheel_radius;
      matrix(i, 1) = std::cos(theta) / model.wheel_radius;
      matrix(i, 2) = model.robot_radius / model.wheel_radius;
    }

    return matrix;
  }

  /**
   * @brief 使用 SVD 计算 Moore-Penrose 伪逆。
   *
   * @param matrix 待求伪逆的矩阵。
   * @param tolerance 小于等于该阈值的奇异值会被视为 0。
   * @return `matrix` 的伪逆矩阵。
   */
  static Eigen::MatrixXd pseudoInverse(
    const Eigen::MatrixXd & matrix,
    double tolerance = 1e-8)
  {
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix,
      Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto & singular_values = svd.singularValues();
    Eigen::MatrixXd singular_inverse =
      Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (int i = 0; i < singular_values.size(); ++i) {
      if (singular_values(i) > tolerance) {
        singular_inverse(i, i) = 1.0 / singular_values(i);
      }
    }

    return svd.matrixV() * singular_inverse * svd.matrixU().transpose();
  }

  RobotModel model_;  ///< 已校验的机器人几何参数和轮子顺序。

  Eigen::MatrixXd command_to_wheel_;  ///< 将 [vx, vy, wz]^T 映射为轮子角速度的矩阵。

  Eigen::MatrixXd wheel_to_body_xy_;  ///< 伪逆矩阵前两行，用于轮速到 [vx, vy]^T。
};

/**
 * @brief 连接全向轮运动学、控制器和里程计的 ROS2 节点。
 *
 * 主要职责：
 * - 订阅 `/cmd_vel` 并发布各轮速度控制命令。
 * - 订阅 `/joint_states` 并积分平面里程计。
 * - 订阅 `/imu` 获取 yaw 和角速度。
 * - 发布 `/odom` 和 `odom -> base_footprint` 坐标变换。
 */
class OmniKinematicsNode : public rclcpp::Node
{
public:
  /**
   * @brief 创建发布器、订阅器、TF 广播器和节点状态。
   *
   * @param model 机器人几何参数和轮子关节顺序。
   */
  explicit OmniKinematicsNode(const RobotModel & model)
  : Node("omni_kinematics"),
    kinematics_(model)
  {
    createJointNameIndex();
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
   * 这样 `/joint_states` 中的关节可以按任意顺序到达，
   * 节点仍能恢复出运动学所需的固定轮子顺序。
   */
  void createJointNameIndex()
  {
    const auto & joint_names = kinematics_.model().wheel_joint_names;
    for (int i = 0; i < kinematics_.model().wheel_count; ++i) {
      wheel_index_by_joint_name_[joint_names[i]] = i;
    }
  }

  /**
   * @brief 创建轮子速度命令发布器和里程计发布器。
   */
  void createPublishers()
  {
    wheel_command_publishers_.reserve(kinematics_.model().wheel_count);
    for (int i = 0; i < kinematics_.model().wheel_count; ++i) {
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
    const BodyTwist command{
      kModelDirectionSign * msg->linear.x,
      kModelDirectionSign * msg->linear.y,
      kModelDirectionSign * msg->angular.z};

    publishWheelCommands(kinematics_.calculateWheelSpeeds(command));
  }

  /**
   * @brief 处理轮子状态反馈并发布更新后的里程计。
   *
   * @param msg 包含轮子关节速度的 JointState 消息。
   */
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    Eigen::VectorXd wheel_speeds;
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

    integrateOdometry(wheel_speeds, dt);
    publishOdometry(current_time);
  }

  /**
   * @brief 处理 IMU 姿态和角速度更新。
   *
   * @param msg IMU 消息，其中姿态四元数用于提供里程计 yaw。
   */
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    angular_velocity_z_ = msg->angular_velocity.z;

    const tf2::Quaternion quaternion(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, pose_.yaw);
  }

  /**
   * @brief 从 JointState 消息中提取配置好的轮子速度。
   *
   * @param msg 输入的 JointState 消息。
   * @param wheel_speeds 输出轮速向量，顺序与机器人模型配置一致。
   * @return 如果所有配置轮子的速度都找到则返回 true，否则返回 false。
   */
  bool extractWheelSpeeds(
    const sensor_msgs::msg::JointState & msg,
    Eigen::VectorXd & wheel_speeds) const
  {
    const int wheel_count = kinematics_.model().wheel_count;
    wheel_speeds = Eigen::VectorXd::Zero(wheel_count);
    std::vector<bool> found(static_cast<std::size_t>(wheel_count), false);

    const std::size_t usable_size = std::min(msg.name.size(), msg.velocity.size());
    for (std::size_t i = 0; i < usable_size; ++i) {
      const auto joint = wheel_index_by_joint_name_.find(msg.name[i]);
      if (joint == wheel_index_by_joint_name_.end()) {
        continue;
      }

      wheel_speeds(joint->second) = msg.velocity[i];
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
   * @brief 将由轮速反解得到的线速度积分到里程计位姿中。
   *
   * @param wheel_speeds 测得的轮子角速度，单位 rad/s。
   * @param dt 距离上一次关节状态积分的时间间隔，单位秒。
   */
  void integrateOdometry(const Eigen::VectorXd & wheel_speeds, double dt)
  {
    const Eigen::Vector2d body_linear_velocity =
      kinematics_.estimateBodyLinearVelocity(wheel_speeds);

    const Eigen::Matrix2d odom_from_body = yawRotationMatrix(pose_.yaw);
    const Eigen::Vector2d odom_delta = odom_from_body * body_linear_velocity * dt;

    // 保留原始节点的符号约定，用于补偿 Gazebo 模型中的轮关节方向。
    pose_.x += kModelDirectionSign * odom_delta.x();
    pose_.y += kModelDirectionSign * odom_delta.y();

    linear_velocity_odom_.x() = kModelDirectionSign * odom_delta.x() / dt;
    linear_velocity_odom_.y() = kModelDirectionSign * odom_delta.y() / dt;
  }

  /**
   * @brief 构造从机体系到 odom 坐标系的二维旋转矩阵。
   *
   * @param yaw 机器人在 odom 坐标系下的航向角，单位弧度。
   * @return 2x2 旋转矩阵。
   */
  static Eigen::Matrix2d yawRotationMatrix(double yaw)
  {
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw),
                std::sin(yaw), std::cos(yaw);
    return rotation;
  }

  /**
   * @brief 向所有轮子控制器发布角速度命令。
   *
   * @param wheel_speeds 按配置轮子顺序排列的轮子角速度。
   */
  void publishWheelCommands(const Eigen::VectorXd & wheel_speeds)
  {
    for (int i = 0; i < wheel_speeds.size(); ++i) {
      std_msgs::msg::Float64MultiArray message;
      message.data = {wheel_speeds(i)};
      wheel_command_publishers_[static_cast<std::size_t>(i)]->publish(message);
    }
  }

  /**
   * @brief 使用当前位姿和速度状态发布 nav_msgs/Odometry。
   *
   * @param stamp 里程计消息和对应 TF 使用的时间戳。
   */
  void publishOdometry(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = kOdomFrame;
    odom_msg.child_frame_id = kBaseFrame;

    odom_msg.pose.pose.position.x = pose_.x;
    odom_msg.pose.pose.position.y = pose_.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = createYawQuaternion(pose_.yaw);

    odom_msg.twist.twist.linear.x = linear_velocity_odom_.x();
    odom_msg.twist.twist.linear.y = linear_velocity_odom_.y();
    odom_msg.twist.twist.angular.z = angular_velocity_z_;

    odom_publisher_->publish(odom_msg);
    publishOdometryTransform(stamp, odom_msg.pose.pose.orientation);
  }

  /**
   * @brief 将 yaw 角转换为 geometry_msgs 四元数。
   *
   * @param yaw 航向角，单位弧度。
   * @return 表示 roll = pitch = 0 且 yaw 为输入值的四元数。
   */
  geometry_msgs::msg::Quaternion createYawQuaternion(double yaw) const
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

  /**
   * @brief 发布当前里程计位姿对应的 TF 坐标变换。
   *
   * @param stamp TF 消息使用的时间戳。
   * @param orientation 已在 odom 消息中使用的姿态四元数。
   */
  void publishOdometryTransform(
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::Quaternion & orientation)
  {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = stamp;
    odom_tf.header.frame_id = kOdomFrame;
    odom_tf.child_frame_id = kBaseFrame;
    odom_tf.transform.translation.x = pose_.x;
    odom_tf.transform.translation.y = pose_.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(odom_tf);
  }

  OmniWheelKinematics kinematics_;  ///< 回调使用的纯运动学模型。

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

  Pose2D pose_;  ///< 当前积分得到的平面里程计位姿。

  Eigen::Vector2d linear_velocity_odom_ = Eigen::Vector2d::Zero();  ///< odom 系线速度，单位 m/s。

  double angular_velocity_z_ = 0.0;  ///< 最近一次 IMU z 轴角速度，单位 rad/s。

  rclcpp::Time last_joint_state_time_;  ///< 上一次关节状态更新时间戳。
};

/**
 * @brief 程序入口函数。
 *
 * 创建本包使用的固定三轮机器人模型，并启动 ROS2 运动学节点。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const RobotModel model{
    3,
    kRobotRadius,
    kWheelRadius,
    0.0,
    {
      "front_wheel_joint",
      "left_wheel_joint",
      "right_wheel_joint",
    }};

  rclcpp::spin(std::make_shared<OmniKinematicsNode>(model));
  rclcpp::shutdown();
  return 0;
}
