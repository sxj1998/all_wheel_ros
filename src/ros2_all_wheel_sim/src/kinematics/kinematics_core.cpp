#include "ros2_all_wheel_sim/kinematics/kinematics_core.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

namespace ros2_all_wheel_sim
{
namespace core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;  ///< 圆周率常量，用于角度与弧度转换。

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
 * @brief 将 std::vector 轮速转换为 Eigen 向量。
 *
 * @param values 输入标量数组。
 * @return Eigen 动态列向量。
 */
Eigen::VectorXd toEigenVector(const std::vector<double> & values)
{
  Eigen::VectorXd vector(static_cast<int>(values.size()));
  for (std::size_t i = 0; i < values.size(); ++i) {
    vector(static_cast<int>(i)) = values[i];
  }
  return vector;
}

/**
 * @brief 将 Eigen 向量转换为 std::vector。
 *
 * @param vector Eigen 动态列向量。
 * @return 普通 C++ 标量数组。
 */
std::vector<double> toStdVector(const Eigen::VectorXd & vector)
{
  std::vector<double> values(static_cast<std::size_t>(vector.size()));
  for (int i = 0; i < vector.size(); ++i) {
    values[static_cast<std::size_t>(i)] = vector(i);
  }
  return values;
}

/**
 * @brief 使用 SVD 计算 Moore-Penrose 伪逆。
 *
 * @param matrix 待求伪逆的矩阵。
 * @param tolerance 小于等于该阈值的奇异值会被视为 0。
 * @return `matrix` 的伪逆矩阵。
 */
Eigen::MatrixXd pseudoInverse(
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

/**
 * @brief 构造从机体系速度到轮子角速度的运动学矩阵。
 *
 * @param model 已校验的机器人模型。
 * @return N x 3 矩阵，其中 N 为轮子数量。
 */
Eigen::MatrixXd buildCommandToWheelMatrix(const RobotModel & model)
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
 * @brief 构造从机体系到世界/里程计坐标系的二维旋转矩阵。
 *
 * @param yaw 机器人航向角，单位弧度。
 * @return 2x2 旋转矩阵。
 */
Eigen::Matrix2d yawRotationMatrix(double yaw)
{
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw),
              std::sin(yaw), std::cos(yaw);
  return rotation;
}
}  // namespace

class OmniWheelKinematics::Impl
{
public:
  explicit Impl(const RobotModel & model)
  : command_to_wheel(buildCommandToWheelMatrix(model)),
    wheel_to_body_xy(pseudoInverse(command_to_wheel).topRows(2))
  {
  }

  Eigen::MatrixXd command_to_wheel;  ///< 将 [vx, vy, wz]^T 映射为轮速的矩阵。
  Eigen::MatrixXd wheel_to_body_xy;  ///< 伪逆前两行，用于轮速到 [vx, vy]^T。
};

void RobotModel::validate() const
{
  if (wheel_count <= 0) {
    throw std::runtime_error("Wheel count must be positive");
  }
  if (robot_radius <= 0.0 || wheel_radius <= 0.0) {
    throw std::runtime_error("Robot radius and wheel radius must be positive");
  }
  if (!wheel_names.empty() && static_cast<int>(wheel_names.size()) != wheel_count) {
    throw std::runtime_error("Wheel name count does not match robot model");
  }
}

OmniWheelKinematics::OmniWheelKinematics(RobotModel model)
: model_(std::move(model))
{
  model_.validate();
  impl_ = std::make_unique<Impl>(model_);
}

OmniWheelKinematics::~OmniWheelKinematics() = default;

OmniWheelKinematics::OmniWheelKinematics(const OmniWheelKinematics & other)
: model_(other.model_),
  impl_(std::make_unique<Impl>(*other.impl_))
{
}

OmniWheelKinematics & OmniWheelKinematics::operator=(const OmniWheelKinematics & other)
{
  if (this == &other) {
    return *this;
  }

  model_ = other.model_;
  impl_ = std::make_unique<Impl>(*other.impl_);
  return *this;
}

OmniWheelKinematics::OmniWheelKinematics(OmniWheelKinematics &&) noexcept = default;

OmniWheelKinematics & OmniWheelKinematics::operator=(OmniWheelKinematics &&) noexcept = default;

std::vector<double> OmniWheelKinematics::calculateWheelSpeeds(const BodyTwist & twist) const
{
  const Eigen::Vector3d body_velocity(twist.vx, twist.vy, twist.wz);
  return toStdVector(impl_->command_to_wheel * body_velocity);
}

BodyTwist OmniWheelKinematics::estimateBodyLinearVelocity(
  const std::vector<double> & wheel_speeds) const
{
  if (static_cast<int>(wheel_speeds.size()) != model_.wheel_count) {
    throw std::runtime_error("Wheel speed vector size does not match robot model");
  }

  const Eigen::Vector2d body_linear_velocity =
    impl_->wheel_to_body_xy * toEigenVector(wheel_speeds);

  return {body_linear_velocity.x(), body_linear_velocity.y(), 0.0};
}

const RobotModel & OmniWheelKinematics::model() const
{
  return model_;
}

OmniWheelOdometry::OmniWheelOdometry(
  OmniWheelKinematics kinematics,
  double model_direction_sign)
: kinematics_(std::move(kinematics)),
  model_direction_sign_(model_direction_sign)
{
}

void OmniWheelOdometry::updateHeading(double yaw, double angular_z)
{
  state_.pose.yaw = yaw;
  state_.angular_z = angular_z;
}

const OdometryState & OmniWheelOdometry::integrate(
  const std::vector<double> & wheel_speeds,
  double dt)
{
  if (dt <= 0.0) {
    return state_;
  }

  const BodyTwist body_linear_velocity =
    kinematics_.estimateBodyLinearVelocity(wheel_speeds);

  const Eigen::Vector2d body_velocity(body_linear_velocity.vx, body_linear_velocity.vy);
  const Eigen::Vector2d odom_delta =
    yawRotationMatrix(state_.pose.yaw) * body_velocity * dt;

  state_.pose.x += model_direction_sign_ * odom_delta.x();
  state_.pose.y += model_direction_sign_ * odom_delta.y();
  state_.linear_x = model_direction_sign_ * odom_delta.x() / dt;
  state_.linear_y = model_direction_sign_ * odom_delta.y() / dt;

  return state_;
}

void OmniWheelOdometry::reset(const Pose2D & pose)
{
  state_ = {};
  state_.pose = pose;
}

const OdometryState & OmniWheelOdometry::state() const
{
  return state_;
}

const OmniWheelKinematics & OmniWheelOdometry::kinematics() const
{
  return kinematics_;
}

RobotModel makeDefaultAllWheelModel()
{
  return {
    3,
    kDefaultRobotRadius,
    kDefaultWheelRadius,
    0.0,
    {
      "front_wheel_joint",
      "left_wheel_joint",
      "right_wheel_joint",
    }};
}

}  // namespace core
}  // namespace ros2_all_wheel_sim
