#pragma once

#include <memory>
#include <string>
#include <vector>

namespace ros2_all_wheel_sim
{
namespace core
{

constexpr double kDefaultWheelRadius = 0.035115;  ///< 默认全向轮半径，单位为米。
constexpr double kDefaultRobotRadius = 0.0990025403784439;  ///< 默认机器人中心到轮子距离，单位为米。
constexpr double kDefaultModelDirectionSign = -1.0;  ///< 默认模型方向补偿系数。

/**
 * @brief 全向轮底盘的几何参数和轮子顺序描述。
 *
 * 该结构体不依赖 ROS2，可被其他平台直接复用。`wheel_names`
 * 只是可选的轮子命名信息，用于上层适配器把外部传感器数据排成固定顺序。
 */
struct RobotModel
{
  int wheel_count = 0;  ///< 底盘中主动全向轮的数量。
  double robot_radius = 0.0;  ///< 机器人中心到轮子接地点的距离，单位为米。
  double wheel_radius = 0.0;  ///< 轮子的物理半径，单位为米。
  double heading_offset_deg = 0.0;  ///< 0 号轮相对机器人 +x 轴的角度偏移，单位为度。
  std::vector<std::string> wheel_names;  ///< 轮子名称，顺序与运动学矩阵行顺序一致。

  /**
   * @brief 校验机器人模型参数。
   *
   * @throws std::runtime_error 当轮子数量、半径或名称数量不合法时抛出异常。
   */
  void validate() const;
};

/**
 * @brief 机器人机体系下的平面速度。
 */
struct BodyTwist
{
  double vx = 0.0;  ///< 沿机器人 +x 方向的线速度，单位 m/s。
  double vy = 0.0;  ///< 沿机器人 +y 方向的线速度，单位 m/s。
  double wz = 0.0;  ///< 绕机器人 +z 轴的角速度，单位 rad/s。
};

/**
 * @brief 二维平面位姿。
 */
struct Pose2D
{
  double x = 0.0;  ///< 世界/里程计坐标系下的 x 位置，单位为米。
  double y = 0.0;  ///< 世界/里程计坐标系下的 y 位置，单位为米。
  double yaw = 0.0;  ///< 世界/里程计坐标系下的航向角，单位为弧度。
};

/**
 * @brief 平台无关的里程计状态。
 */
struct OdometryState
{
  Pose2D pose;  ///< 当前积分得到的二维位姿。
  double linear_x = 0.0;  ///< 世界/里程计坐标系下的 x 方向线速度，单位 m/s。
  double linear_y = 0.0;  ///< 世界/里程计坐标系下的 y 方向线速度，单位 m/s。
  double angular_z = 0.0;  ///< 绕 z 轴角速度，单位 rad/s。
};

/**
 * @brief 等角度布置全向轮底盘的纯运动学模型。
 *
 * 该类只依赖 C++ 标准库和 Eigen 实现文件，不包含 ROS2 类型。
 * 上层平台只需要提供底盘速度或按固定顺序排列的轮速即可复用。
 */
class OmniWheelKinematics
{
public:
  /**
   * @brief 根据机器人模型构造运动学矩阵。
   *
   * @param model 机器人几何参数和轮子顺序。
   */
  explicit OmniWheelKinematics(RobotModel model);
  ~OmniWheelKinematics();
  OmniWheelKinematics(const OmniWheelKinematics & other);
  OmniWheelKinematics & operator=(const OmniWheelKinematics & other);
  OmniWheelKinematics(OmniWheelKinematics &&) noexcept;
  OmniWheelKinematics & operator=(OmniWheelKinematics &&) noexcept;

  /**
   * @brief 将机体系速度指令转换为各轮角速度。
   *
   * @param twist 机体系平面速度。
   * @return 按模型轮子顺序排列的轮子角速度，单位 rad/s。
   */
  std::vector<double> calculateWheelSpeeds(const BodyTwist & twist) const;

  /**
   * @brief 根据轮子角速度估计机体系线速度。
   *
   * @param wheel_speeds 按模型轮子顺序排列的轮子角速度，单位 rad/s。
   * @return 机体系速度，其中 `wz` 固定为 0，角速度由上层 IMU 或陀螺仪提供。
   */
  BodyTwist estimateBodyLinearVelocity(const std::vector<double> & wheel_speeds) const;

  /**
   * @brief 获取当前运动学模型。
   *
   * @return 机器人模型引用。
   */
  const RobotModel & model() const;

private:
  RobotModel model_;  ///< 已校验的机器人几何参数和轮子顺序。

  class Impl;
  std::unique_ptr<Impl> impl_;  ///< 隐藏 Eigen 实现细节，减少头文件对移植平台的侵入。
};

/**
 * @brief 平台无关的全向轮里程计积分器。
 *
 * 该类只关心轮速、yaw、角速度和 dt，不关心数据来自 ROS2、串口、
 * CAN、仿真器还是其他实时系统。
 */
class OmniWheelOdometry
{
public:
  /**
   * @brief 创建里程计积分器。
   *
   * @param kinematics 纯运动学模型。
   * @param model_direction_sign 模型方向补偿系数，默认与当前 Gazebo 模型一致。
   */
  explicit OmniWheelOdometry(
    OmniWheelKinematics kinematics,
    double model_direction_sign = kDefaultModelDirectionSign);

  /**
   * @brief 使用外部姿态源更新 yaw 和 z 轴角速度。
   *
   * @param yaw 当前航向角，单位弧度。
   * @param angular_z 绕 z 轴角速度，单位 rad/s。
   */
  void updateHeading(double yaw, double angular_z);

  /**
   * @brief 根据轮速和时间间隔积分里程计。
   *
   * @param wheel_speeds 按模型轮子顺序排列的轮速，单位 rad/s。
   * @param dt 距离上次积分的时间间隔，单位秒。
   * @return 更新后的里程计状态。
   */
  const OdometryState & integrate(const std::vector<double> & wheel_speeds, double dt);

  /**
   * @brief 重置里程计状态。
   *
   * @param pose 新的二维位姿。
   */
  void reset(const Pose2D & pose = {});

  /**
   * @brief 获取当前里程计状态。
   *
   * @return 当前里程计状态引用。
   */
  const OdometryState & state() const;

  /**
   * @brief 获取内部运动学模型。
   *
   * @return 运动学模型引用。
   */
  const OmniWheelKinematics & kinematics() const;

private:
  OmniWheelKinematics kinematics_;  ///< 用于轮速反解的运动学模型。
  double model_direction_sign_ = kDefaultModelDirectionSign;  ///< 模型方向补偿系数。
  OdometryState state_;  ///< 当前里程计状态。
};

/**
 * @brief 创建当前 all_wheel 模型使用的默认三轮机器人模型。
 *
 * @return 默认机器人模型。
 */
RobotModel makeDefaultAllWheelModel();

}  // namespace core
}  // namespace ros2_all_wheel_sim
