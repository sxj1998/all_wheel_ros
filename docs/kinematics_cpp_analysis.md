# kinematics.cpp 详细分析与三全向轮运动学矩阵推导

本文档分析源码：

`/home/shexingju/work/ROS2/all_wheel_ros/src/ros2_all_wheel_sim/src/kinematics.cpp`

该文件实现了一个 ROS2 节点 `omni_kinematics`，用于三全向轮机器人在 Gazebo/ROS2 中的速度控制与里程计发布。

---

## 1. 节点总体作用

`kinematics.cpp` 的核心职责可以概括为两条数据链路：

1. 控制链路：订阅 `/cmd_vel`，将机器人期望速度转换为三个轮子的角速度命令。
2. 里程计链路：订阅 `/joint_states` 和 `/imu`，根据轮速和 IMU 姿态估算机器人位姿，并发布 `/odom` 和 `odom -> base_footprint` TF。

```text
控制链路

  +-----------------------------+
  | /cmd_vel                    |
  | geometry_msgs/msg/Twist     |
  +--------------+--------------+
                 |
                 v
  +-----------------------------+
  | cmd_vel_callback()          |
  +--------------+--------------+
                 |
                 v
  +-----------------------------+
  | calculate_motor_speed()     |
  | M = tM * [vx, vy, wz]^T     |
  +--------------+--------------+
                 |
                 v
  +-----------------------------+
  | set_motor_speed()           |
  +------+---------------+------+
         |               |
         |               +------------------------------+
         |                                              |
         v                                              v
  +-----------------------------+          +-----------------------------+
  | wheel1_controller/commands  |          | wheel3_controller/commands  |
  | front_wheel_joint           |          | right_wheel_joint           |
  +-----------------------------+          +-----------------------------+
         |
         v
  +-----------------------------+
  | wheel2_controller/commands  |
  | left_wheel_joint            |
  +-----------------------------+


里程计链路

  +-----------------------------+          +-----------------------------+
  | /joint_states               |          | /imu                        |
  | sensor_msgs/msg/JointState  |          | sensor_msgs/msg/Imu         |
  +--------------+--------------+          +--------------+--------------+
                 |                                        |
                 v                                        v
  +-----------------------------+          +-----------------------------+
  | join_states_callback()      |          | imu_callback()              |
  | 提取 front/left/right 轮速  |          | quaternion -> yaw           |
  +--------------+--------------+          | angular_velocity.z -> w     |
                 |                         +--------------+--------------+
                 |                                        |
                 +------------------------+---------------+
                                          |
                                          v
                         +-----------------------------+
                         | tMI * wheel_speed           |
                         | 计算底盘平移速度            |
                         +--------------+--------------+
                                        |
                                        v
                         +-----------------------------+
                         | 结合 yaw 旋转到 odom 坐标系 |
                         +--------------+--------------+
                                        |
                                        v
                         +-----------------------------+
                         | publish_odom()              |
                         +------+---------------+------+
                                |               |
                                v               v
                         +------------+  +---------------------------+
                         | /odom      |  | TF: odom -> base_footprint |
                         +------------+  +---------------------------+
```

---

## 2. 机器人模型与参数

源码中定义了两个关键几何参数：

```cpp
#define WHEEL_RADIUS        0.035115
#define ROBOT_RADIUS        0.0990025403784439
```

| 参数 | 变量 | 数值 | 含义 |
|---|---:|---:|---|
| 轮子半径 | `r` | `0.035115 m` | 轮子自身半径 |
| 机器人半径 | `R` | `0.0990025403784439 m` | 机器人中心到轮子安装点/接地点的距离 |
| 轮子数量 | `N` | `3` | 三全向轮结构 |
| 角度偏移 | `heading_offset` | `0 deg` | 第一个轮子相对机器人坐标系的角度偏移 |

`main()` 中固定使用三个轮关节：

```cpp
const std::vector<std::string> wheel_joint_names = {
  "front_wheel_joint",
  "left_wheel_joint",
  "right_wheel_joint",
};
```

与控制器配置对应关系如下：

| 控制器 topic | 关节名 | 轮子编号 | 几何角度 |
|---|---|---:|---:|
| `wheel1_controller/commands` | `front_wheel_joint` | 0 | `0 deg` |
| `wheel2_controller/commands` | `left_wheel_joint` | 1 | `120 deg` |
| `wheel3_controller/commands` | `right_wheel_joint` | 2 | `240 deg` |

---

## 3. 三轮布局示意图

机器人坐标系约定：

- `+x` 指向机器人前方。
- `+y` 指向机器人左侧。
- `+z` 垂直向上。
- 正 yaw 为绕 `+z` 逆时针旋转。

三轮在俯视图中的位置如下：

```text
三轮位置角以 +x 前方为 0 deg，逆时针为正：

                              +y 左侧
                                ^
                                |
                                |
                left wheel      |      theta_1 = 120 deg
          (-R/2, +sqrt(3)R/2)   |
                         [W1]   |
                           o    |
                            \   |
                             \  |  120 deg
                              \ |
                               \|
                                O----------------------o [W0]
                               /|                      front wheel
                              / |                      theta_0 = 0 deg
                             /  |                      (R, 0)
                            /   |
                           o    |
                         [W2]   |
          (-R/2, -sqrt(3)R/2)   |
                right wheel     |      theta_2 = 240 deg
                                |
                                +----------------------------> +x 前方


三条半径射线之间的夹角：

             W1
              o
               \
                \ 120 deg
                 \
                  O----------o W0
                 /
                / 120 deg
               /
              o
             W2

  angle(W0, W1) = 120 deg
  angle(W1, W2) = 120 deg
  angle(W2, W0) = 120 deg


轮子坐标：

  W0/front : ( R, 0 )
  W1/left  : ( R*cos120, R*sin120 )
           = ( -R/2, +sqrt(3)R/2 )
  W2/right : ( R*cos240, R*sin240 )
           = ( -R/2, -sqrt(3)R/2 )


关键几何量

  O -> front wheel  : R = 0.0990025403784439 m
  O -> left wheel   : R = 0.0990025403784439 m
  O -> right wheel  : R = 0.0990025403784439 m


三个轮子的位置角

  theta_0 = 0 deg
  theta_1 = 120 deg
  theta_2 = 240 deg


各轮主动滚动方向 u_i

  u_i = [-sin(theta_i), cos(theta_i)]

  theta_0 = 0 deg:

              +y
               ^
               |
               |  u_0 = [0, 1]
               |
        O------+----> +x
               |
               |
             front

  theta_1 = 120 deg:

              +y
               ^
               |
         left  |
            \  |
             \ |
              \|
        O------+----> +x
               \
                \  u_1 = [-sqrt(3)/2, -1/2]

  theta_2 = 240 deg:

              +y
               ^
               | /
               |/  u_2 = [sqrt(3)/2, -1/2]
        O------+----> +x
              /
             /
         right


每个轮子的线速度由两部分相加：

  wheel linear velocity
    = 底盘平移速度在 u_i 方向上的投影
    + 底盘绕中心旋转在轮子处产生的切向速度

  v_wheel_i = u_i dot [vx, vy] + R * wz
```

> 注意：上图是运动学推导示意图，用于说明三个轮子相隔 120 deg 的布局。实际 URDF 中三个关节均使用 `axis="1 0 0"`，并通过各自关节姿态 `rpy` 旋转到对应方向。

---

## 4. 从单个轮子推导运动学关系

设机器人在自身坐标系中的速度为：

```text
v = [vx, vy, wz]^T
```

其中：

- `vx`：机器人前后方向速度。
- `vy`：机器人左右方向速度。
- `wz`：机器人绕 z 轴角速度。

第 `i` 个轮子位于角度：

```text
theta_i = i * 360 / N + heading_offset
```

三轮时：

```text
theta_0 = 0 deg
theta_1 = 120 deg
theta_2 = 240 deg
```

对于全向轮，轮子允许沿被动滚子方向侧滑，但沿主动滚动方向会产生约束和驱动力。代码中使用的第 `i` 个轮子的主动滚动方向单位向量为：

```text
u_i = [-sin(theta_i), cos(theta_i)]
```

机器人平移速度 `[vx, vy]` 在该方向上的投影为：

```text
u_i dot [vx, vy]
= -sin(theta_i) * vx + cos(theta_i) * vy
```

机器人绕中心旋转时，轮子处的切向速度大小为：

```text
R * wz
```

因此第 `i` 个轮子的线速度为：

```text
v_wheel_i = -sin(theta_i) * vx + cos(theta_i) * vy + R * wz
```

轮子角速度等于线速度除以轮半径 `r`：

```text
omega_i = v_wheel_i / r
```

所以：

```text
omega_i =
[-sin(theta_i)/r, cos(theta_i)/r, R/r] * [vx, vy, wz]^T
```

这正是源码中 `init_transform_matrix()` 的三列：

```cpp
M(i,0) = -sin((del_angle * i + heading_offset) * M_PI / 180)/r;
M(i,1) =  cos((del_angle * i + heading_offset) * M_PI / 180)/r;
M(i,2) = R/r;
```

---

## 5. 三轮运动学矩阵展开

通用矩阵形式：

```text
[omega_0]   [ -sin(theta_0)/r   cos(theta_0)/r   R/r ] [vx]
[omega_1] = [ -sin(theta_1)/r   cos(theta_1)/r   R/r ] [vy]
[omega_2]   [ -sin(theta_2)/r   cos(theta_2)/r   R/r ] [wz]
```

当前 `heading_offset = 0`，所以：

```text
theta_0 = 0 deg
theta_1 = 120 deg
theta_2 = 240 deg
```

三角函数值：

| 轮子 | `theta` | `sin(theta)` | `cos(theta)` |
|---|---:|---:|---:|
| front | `0 deg` | `0` | `1` |
| left | `120 deg` | `sqrt(3)/2` | `-1/2` |
| right | `240 deg` | `-sqrt(3)/2` | `-1/2` |

代入后：

```text
omega_0 = (0 * vx + 1 * vy + R * wz) / r

omega_1 = (-(sqrt(3)/2) * vx - (1/2) * vy + R * wz) / r

omega_2 = ((sqrt(3)/2) * vx - (1/2) * vy + R * wz) / r
```

矩阵写成：

```text
            [ 0              1       R ]          [vx]
[omega] = 1/r * [ -sqrt(3)/2  -1/2    R ]    *     [vy]
            [ sqrt(3)/2   -1/2    R ]          [wz]
```

带入本项目参数：

```text
r = 0.035115
R = 0.0990025403784439
```

可得到近似数值矩阵：

```text
tM =
[  0.0000   28.4779   2.8194 ]
[ -24.6625 -14.2389   2.8194 ]
[  24.6625 -14.2389   2.8194 ]
```

也就是：

```text
[front_wheel_speed]   [  0.0000   28.4779   2.8194 ] [vx]
[left_wheel_speed ] = [ -24.6625 -14.2389   2.8194 ] [vy]
[right_wheel_speed]   [  24.6625 -14.2389   2.8194 ] [wz]
```

---

## 6. 运动学矩阵与代码对应关系

```text
                +--------------------------------------+
                | init_transform_matrix(N, offset)     |
                +-------------------+------------------+
                                    |
                                    v
                +--------------------------------------+
                | 对每个轮子逐行计算：                 |
                | [-sin(theta)/r, cos(theta)/r, R/r]   |
                +-------------------+------------------+
                                    |
                                    v
                             +-------------+
                             | tM 矩阵     |
                             +------+------+ 
                                    |
                                    v
  +-------------------+     +---------------+     +------------------------+
  | 输入速度          | --> | M = tM * v    | --> | wheel1_controller     |
  | v=[vx,vy,wz]^T    |     | 计算轮速      |     | wheel2_controller     |
  +-------------------+     +---------------+     | wheel3_controller     |
                                                  +------------------------+
```

对应源码：

```cpp
Eigen::VectorXd calculate_motor_speed(float x_, float y_, float w_) {
  Eigen::Vector3d v(x_, y_, w_);
  Eigen::VectorXd M = tM*v;
  return M;
}
```

`cmd_vel_callback()` 中调用时取了负号：

```cpp
Eigen::VectorXd M = calculate_motor_speed(
  -msg->linear.x,
  -msg->linear.y,
  -msg->angular.z
);
```

这说明作者在代码层面对模型方向做了符号修正。也就是说，外部发出的 `/cmd_vel` 与 URDF/控制器中轮关节正方向之间存在相反关系，因此这里把 `linear.x`、`linear.y`、`angular.z` 全部取反。

---

## 7. 逆运动学与里程计矩阵

正向控制使用：

```text
wheel_speed = tM * robot_speed
```

里程计需要反过来，根据轮速估计机器人速度：

```text
robot_speed = tM^-1 * wheel_speed
```

源码中使用 SVD 计算伪逆：

```cpp
tMI = pseudo_inverse(tM);
tMI = Eigen::MatrixXd(tMI.block(0, 0, 2, tMI.cols()));
```

这里 `pseudo_inverse(tM)` 原本会得到 `3 x 3` 矩阵，对应：

```text
[vx]      [ ... ] [omega_0]
[vy]  =   [ ... ] [omega_1]
[wz]      [ ... ] [omega_2]
```

但代码只取前两行：

```cpp
tMI.block(0, 0, 2, tMI.cols())
```

所以最终只用轮速估计 `vx`、`vy`，不从轮速估计 `wz`。角速度 `wz` 来自 IMU：

```cpp
w = msg->angular_velocity.z;
```

当前参数下，伪逆前两行大致为：

```text
tMI =
[ 0.00000  -0.02027   0.02027 ]
[ 0.02341  -0.01170  -0.01170 ]
```

也就是：

```text
body_vx =  0.00000 * omega_front
         - 0.02027 * omega_left
         + 0.02027 * omega_right

body_vy =  0.02341 * omega_front
         - 0.01170 * omega_left
         - 0.01170 * omega_right
```

这个关系也符合三轮布局直觉：

- 左右轮差速主要影响 `vx`。
- 前轮和左右轮的组合主要影响 `vy`。

---

## 8. 里程计积分过程

`join_states_callback()` 中先从 `/joint_states` 提取三个轮子的实际角速度：

```cpp
Eigen::VectorXd w(N);

for (size_t i = 0; i < msg->name.size(); ++i)
{
  for (const auto& pair : wheel_joint_map_index) {
    if (msg->name[i] == pair.first){
      w(pair.second) = msg->velocity[i];
      break;
    }
  }
}
```

然后计算时间差：

```cpp
double dt = (current_time - last_time).seconds();
```

通过轮速估计机器人机体系平移速度：

```cpp
tMI * w
```

再根据 IMU yaw 构造二维旋转矩阵：

```cpp
rM << cos(yaw), -sin(yaw),
      sin(yaw),  cos(yaw);
```

用该旋转矩阵把机体系速度转换到 `odom` 坐标系，并乘以 `dt` 得到位移增量：

```cpp
dp = rM * tMI * w * dt;
```

积分位置：

```cpp
pos_x -= dp(0);
pos_y -= dp(1);
vx = -dp(0) / dt;
vy = -dp(1) / dt;
```

这里再次使用负号，和 `/cmd_vel` 输入处的负号相呼应，属于方向修正。

里程计流程图：

```text
  +------------------+       +----------------------------------+
  | /joint_states    | ----> | 按 joint name 提取轮速           |
  +------------------+       | front / left / right             |
                             +----------------+-----------------+
                                              |
                                              v
                             +----------------------------------+
                             | wheel vector                     |
                             | [omega0, omega1, omega2]^T       |
                             +----------------+-----------------+
                                              |
                                              v
                             +----------------------------------+
                             | body velocity                    |
                             | [vx, vy]^T = tMI * wheel vector  |
                             +----------------+-----------------+
                                              |
  +------------------+       +----------------+-----------------+
  | /imu             | ----> | quaternion -> roll, pitch, yaw    |
  +------------------+       +----------------+-----------------+
                                              |
                                              v
                             +----------------------------------+
                             | yaw -> 旋转矩阵 rM               |
                             +----------------+-----------------+
                                              |
                                              v
                             +----------------------------------+
                             | dp = rM * [vx, vy]^T * dt        |
                             +----------------+-----------------+
                                              |
                                              v
                             +----------------------------------+
                             | pos_x, pos_y 积分                |
                             +----------------+-----------------+
                                              |
                                   +----------+----------+
                                   |                     |
                                   v                     v
                             +------------+      +-------------------------+
                             | 发布 /odom |      | 发布 TF                 |
                             |            |      | odom -> base_footprint  |
                             +------------+      +-------------------------+
```

---

## 9. IMU 在该节点中的作用

IMU 回调函数：

```cpp
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  w = msg->angular_velocity.z;

  tf2::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w
  );

  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}
```

IMU 提供两类信息：

| IMU 字段 | 代码变量 | 用途 |
|---|---|---|
| `orientation` | `roll, pitch, yaw` | 里程计姿态，主要使用 `yaw` |
| `angular_velocity.z` | `w` | `/odom.twist.twist.angular.z` |

源码没有用轮速积分角度，而是直接采用 IMU 的 yaw 作为机器人朝向。这在仿真中通常更稳定，也能避免轮速积分造成的角度漂移。

---

## 10. /odom 与 TF 发布

`publish_odom()` 发布两个内容：

1. `nav_msgs/msg/Odometry`，topic 为 `/odom`。
2. TF 变换，`odom -> base_footprint`。

```cpp
odom_msg.header.frame_id = "odom";
odom_msg.child_frame_id = "base_footprint";
```

位置：

```cpp
odom_msg.pose.pose.position.x = pos_x;
odom_msg.pose.pose.position.y = pos_y;
```

姿态：

```cpp
tf2::Quaternion q;
q.setRPY(0, 0, yaw);
```

速度：

```cpp
odom_msg.twist.twist.linear.x = vx;
odom_msg.twist.twist.linear.y = vy;
odom_msg.twist.twist.angular.z = w;
```

TF：

```cpp
odom_tf.header.frame_id = "odom";
odom_tf.child_frame_id = "base_footprint";
odom_tf.transform.translation.x = pos_x;
odom_tf.transform.translation.y = pos_y;
odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
```

发布关系：

```text
                 +------------------+
                 | pos_x, pos_y     |
                 | yaw              |
                 +--------+---------+
                          |
             +------------+------------+
             |                         |
             v                         v
  +-----------------------+  +----------------------------+
  | nav_msgs/msg/Odometry |  | TransformStamped           |
  | frame_id: odom        |  | frame_id: odom             |
  | child: base_footprint |  | child: base_footprint      |
  +-----------+-----------+  +-------------+--------------+
              |                            |
              v                            v
          +-------+          +-----------------------------+
          | /odom |          | tf_broadcaster.sendTransform |
          +-------+          +--------------+--------------+
                                             |
                                             v
                              +----------------------------+
                              | TF tree                    |
                              | odom -> base_footprint     |
                              +----------------------------+
```

---

## 11. 关键函数说明

| 函数 | 位置 | 作用 |
|---|---|---|
| `OmniKinematics()` | 构造函数 | 初始化参数、矩阵、发布者、订阅者、TF broadcaster |
| `cmd_vel_callback()` | `/cmd_vel` 回调 | 将期望底盘速度转成轮速命令 |
| `join_states_callback()` | `/joint_states` 回调 | 根据实际轮速估算平移里程计 |
| `imu_callback()` | `/imu` 回调 | 获取 yaw 和 z 轴角速度 |
| `publish_odom()` | 发布函数 | 发布 `/odom` 与 TF |
| `calculate_motor_speed()` | 正运动学计算 | `wheel_speed = tM * robot_speed` |
| `set_motor_speed()` | 控制输出 | 发布三个轮子的速度命令 |
| `init_transform_matrix()` | 矩阵初始化 | 生成三全向轮运动学矩阵 |
| `pseudo_inverse()` | 矩阵工具 | 使用 SVD 计算伪逆 |

---

## 12. 代码中值得注意的问题

### 12.1 缺少 `<unordered_map>` 头文件

代码使用了：

```cpp
unordered_map<string, int> wheel_joint_map_index;
```

但文件开头没有显式包含：

```cpp
#include <unordered_map>
```

如果当前能编译，可能是其他头文件间接包含了它。但从规范性和可移植性看，应显式加入。

### 12.2 轮速向量没有初始化

当前写法：

```cpp
Eigen::VectorXd w(N);
```

如果某次 `/joint_states` 消息中缺少某个关节，该元素可能保持未定义值。更稳妥写法：

```cpp
Eigen::VectorXd w = Eigen::VectorXd::Zero(N);
```

### 12.3 没有检查 `velocity` 数组长度

当前代码默认：

```cpp
msg->velocity[i]
```

一定存在。实际工程中最好检查：

```cpp
if (i < msg->velocity.size()) {
  ...
}
```

### 12.4 `mOd` 计算后未使用

构造函数中计算了：

```cpp
double mOd[2][3]
```

但后续没有任何地方使用。它可能是旧版本的里程计矩阵，后来被 `tMI` 替代。

### 12.5 `timer_` 与 `print_vector()` 未使用

```cpp
rclcpp::TimerBase::SharedPtr timer_;
```

和：

```cpp
template <typename T>
void print_vector(vector<T>& vec)
```

目前都是未使用代码，可以清理。

### 12.6 `/odom.twist` 的坐标系语义需注意

源码中：

```cpp
vx = -dp(0) / dt;
vy = -dp(1) / dt;
```

这里的 `dp` 已经经过 `rM` 旋转，表示 `odom` 坐标系下的位移增量，因此 `vx/vy` 更接近 `odom` 坐标系速度。

但 `nav_msgs/Odometry` 中的 `twist` 通常应表达在 `child_frame_id`，也就是 `base_footprint` 坐标系下。很多系统不会严格检查这个细节，但如果后续接入 EKF、Nav2 或其他状态估计器，需要确认坐标系约定。

---

## 13. 总结

`kinematics.cpp` 实现的是一个三全向轮底盘的运动学桥接节点。

控制方向：

```text
/cmd_vel -> tM -> wheel controller commands
```

里程计方向：

```text
/joint_states -> tMI -> body velocity
/imu -> yaw
body velocity + yaw -> odom velocity/position
```

核心运动学矩阵来自单个轮子的速度投影：

```text
omega_i =
[-sin(theta_i)/r, cos(theta_i)/r, R/r] * [vx, vy, wz]^T
```

三轮展开后：

```text
            [ 0              1       R ]          [vx]
[omega] = 1/r * [ -sqrt(3)/2  -1/2    R ]    *     [vy]
            [ sqrt(3)/2   -1/2    R ]          [wz]
```

该节点整体思路清晰，适合三全向轮仿真控制；主要改进点是补齐头文件、初始化轮速向量、检查 JointState 数组长度、清理未使用变量，并确认 `/odom.twist` 的坐标系语义。
