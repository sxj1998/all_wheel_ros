#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ros2_all_wheel_sim
{
namespace exploration
{

/**
 * @brief 基于 frontier 的自主探索节点。
 *
 * 该节点订阅 SLAM 输出的 OccupancyGrid，从已知自由栅格与未知栅格的边界中
 * 提取 frontier，筛选可达且安全的导航目标，并通过 Nav2 的规划与导航 action
 * 驱动机器人逐步探索未知区域。探索结束后可调用 map_saver 保存地图。
 */
class FrontierExplorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandlePlan = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using SaveMap = nav2_msgs::srv::SaveMap;

  /**
   * @brief 创建 frontier 探索节点并初始化 ROS 通信接口。
   *
   * 构造函数会读取探索参数，订阅地图，创建 Nav2 action 客户端、
   * 地图保存客户端以及周期性规划定时器。
   */
  FrontierExplorer()
  : Node("frontier_explorer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    navigator_(rclcpp_action::create_client<NavigateToPose>(
      this, declare_parameter<std::string>("navigate_action", "navigate_to_pose"))),
    planner_(rclcpp_action::create_client<ComputePathToPose>(
      this, declare_parameter<std::string>("plan_action", "compute_path_to_pose")))
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_footprint");
    plan_period_ = declare_parameter<double>("plan_period", 2.0);
    min_frontier_size_ = declare_parameter<int>("min_frontier_size", 8);
    free_threshold_ = declare_parameter<int>("free_threshold", 20);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    frontier_goal_offset_ = declare_parameter<double>("frontier_goal_offset", 0.35);
    goal_search_radius_ = declare_parameter<double>("goal_search_radius", 0.70);
    min_goal_obstacle_clearance_ = declare_parameter<double>("min_goal_obstacle_clearance", 0.30);
    path_obstacle_clearance_ = declare_parameter<double>("path_obstacle_clearance", 0.30);
    goal_blacklist_radius_ = declare_parameter<double>("goal_blacklist_radius", 0.45);
    goal_reached_radius_ = declare_parameter<double>("goal_reached_radius", 0.35);
    max_goal_duration_ = declare_parameter<double>("max_goal_duration", 120.0);
    completion_idle_cycles_ = declare_parameter<int>("completion_idle_cycles", 8);
    auto_save_map_ = declare_parameter<bool>("auto_save_map", true);
    map_save_service_ = declare_parameter<std::string>("map_save_service", "/map_saver/save_map");
    map_save_path_ = declare_parameter<std::string>("map_save_path", "explored_map");

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = msg; });

    map_saver_ = create_client<SaveMap>(map_save_service_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(plan_period_),
      std::bind(&FrontierExplorer::tick, this));
  }

private:
  /**
   * @brief 二维世界坐标点。
   */
  struct Point
  {
    double x{0.0};  ///< map 坐标系下的 x 坐标，单位米。
    double y{0.0};  ///< map 坐标系下的 y 坐标，单位米。
  };

  /**
   * @brief 周期性探索入口。
   *
   * 每次触发时检查地图、动作状态和 Nav2 服务状态。如果当前没有正在执行
   * 的目标，则从地图中选择一个新的 frontier 目标并先进行规划预检。
   */
  void tick()
  {
    if (!map_ || completed_) {
      return;
    }
    if (goal_handle_) {
      cancelTimedOutGoal();
      return;
    }
    if (plan_handle_) {
      cancelTimedOutPlan();
      return;
    }
    if (active_goal_) {
      cancelPendingAction();
      return;
    }
    if (!navigator_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Nav2 NavigateToPose action server");
      return;
    }
    if (!planner_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Nav2 ComputePathToPose action server");
      return;
    }

    const auto robot = robotPosition();
    if (!robot) {
      return;
    }

    const auto goal = chooseFrontierGoal(*robot);
    if (!goal) {
      handleNoReachableFrontier();
      return;
    }

    no_reachable_cycles_ = 0;
    preflightGoal(*goal, *robot);
  }

  /**
   * @brief 处理当前地图中没有可达 frontier 的情况。
   *
   * 探索尚未开始时只打印等待信息；探索开始后连续多轮找不到可达 frontier，
   * 则认为建图完成并尝试保存地图。
   */
  void handleNoReachableFrontier()
  {
    if (!exploration_started_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No reachable frontier found");
      return;
    }

    ++no_reachable_cycles_;
    if (no_reachable_cycles_ < completion_idle_cycles_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "No reachable frontier found; waiting for map updates");
      return;
    }

    completed_ = true;
    RCLCPP_INFO(get_logger(), "Exploration complete: no reachable frontier remains");
    saveMap();
  }

  /**
   * @brief 调用 Nav2 map_saver 保存当前地图。
   */
  void saveMap()
  {
    if (!auto_save_map_) {
      return;
    }
    if (!map_saver_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Map saver service is not available: %s", map_save_service_.c_str());
      return;
    }

    const std::filesystem::path save_path(map_save_path_);
    if (save_path.has_parent_path()) {
      std::error_code error;
      std::filesystem::create_directories(save_path.parent_path(), error);
      if (error) {
        RCLCPP_WARN(
          get_logger(), "Could not create map directory '%s': %s",
          save_path.parent_path().string().c_str(), error.message().c_str());
      }
    }

    auto request = std::make_shared<SaveMap::Request>();
    request->map_topic = map_topic_;
    request->map_url = map_save_path_;
    request->image_format = "pgm";
    request->map_mode = "trinary";
    request->free_thresh = 0.25F;
    request->occupied_thresh = 0.65F;

    map_saver_->async_send_request(
      request,
      [this](rclcpp::Client<SaveMap>::SharedFuture future) {
        const auto response = future.get();
        if (response->result) {
          RCLCPP_INFO(get_logger(), "Map saved to %s.{yaml,pgm}", map_save_path_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Map saver reported failure for %s", map_save_path_.c_str());
        }
      });
  }

  /**
   * @brief 从 TF 中读取机器人在地图坐标系下的位置。
   *
   * @return 成功时返回机器人位置；TF 尚未就绪时返回 std::nullopt。
   */
  std::optional<Point> robotPosition()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
      return Point{transform.transform.translation.x, transform.transform.translation.y};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "Robot pose is not ready: %s", ex.what());
      return std::nullopt;
    }
  }

  /**
   * @brief 从当前地图中选择一个最合适的 frontier 导航目标。
   *
   * @param robot 机器人当前在 map 坐标系下的位置。
   * @return 可达且安全的目标点；没有候选目标时返回 std::nullopt。
   */
  std::optional<Point> chooseFrontierGoal(const Point & robot) const
  {
    const auto width = static_cast<int>(map_->info.width);
    const auto height = static_cast<int>(map_->info.height);
    if (width <= 0 || height <= 0) {
      return std::nullopt;
    }

    const auto frontiers = frontierCells(width, height);
    const auto clusters = clusterCells(frontiers, width, height);

    double best_score = -std::numeric_limits<double>::infinity();
    std::optional<Point> best_goal;
    for (const auto & cluster : clusters) {
      if (static_cast<int>(cluster.size()) < min_frontier_size_) {
        continue;
      }

      const auto candidates = clusterGoalCandidates(cluster, width, height);
      for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
        const auto & goal = candidates[candidate_index];
        if (isBlacklisted(goal)) {
          continue;
        }
        if (!isReachableOnMap(robot, goal, width, height)) {
          continue;
        }

        const double distance = distanceBetween(goal, robot);
        if (distance < goal_reached_radius_) {
          continue;
        }

        // 较大的 frontier 通常能带来更多信息，较近的 frontier 能减少无效绕行。
        // 候选序号惩罚让簇中心优先，同时保留边缘/角落候选以补齐局部房间。
        const double score =
          static_cast<double>(cluster.size()) - distance * 8.0 -
          static_cast<double>(candidate_index) * 0.25;
        if (score > best_score) {
          best_score = score;
          best_goal = goal;
        }
      }
    }
    return best_goal;
  }

  /**
   * @brief 找出地图中所有 frontier 栅格。
   *
   * frontier 定义为：自身是已知自由栅格，且 8 邻域中至少有一个未知栅格。
   *
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return frontier 栅格的一维索引集合。
   */
  std::unordered_set<int> frontierCells(int width, int height) const
  {
    const auto & data = map_->data;
    std::unordered_set<int> cells;
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        const int index = y * width + x;
        if (data[index] < 0 || data[index] > free_threshold_) {
          continue;
        }
        if (touchesUnknown(index, width, height)) {
          cells.insert(index);
        }
      }
    }
    return cells;
  }

  /**
   * @brief 将相邻 frontier 栅格聚类。
   *
   * @param cells 待聚类的 frontier 栅格集合。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return frontier 连通簇列表。
   */
  std::vector<std::vector<int>> clusterCells(
    const std::unordered_set<int> & cells, int width, int height) const
  {
    std::vector<std::vector<int>> clusters;
    auto remaining = cells;
    while (!remaining.empty()) {
      const int start = *remaining.begin();
      remaining.erase(start);

      std::vector<int> cluster{start};
      std::deque<int> queue{start};
      while (!queue.empty()) {
        const int cell = queue.front();
        queue.pop_front();
        for (const int neighbor : neighbors(cell, width, height)) {
          if (remaining.erase(neighbor) == 0) {
            continue;
          }
          cluster.push_back(neighbor);
          queue.push_back(neighbor);
        }
      }
      clusters.push_back(std::move(cluster));
    }
    return clusters;
  }

  /**
   * @brief 为一个 frontier 连通簇生成多个候选导航点。
   *
   * 原始 frontier 位于未知边界，直接导航容易贴墙或进入未知区域。该函数会把
   * 候选点从未知区域反向偏移到已知自由区，再搜索满足安全距离的可站立栅格。
   *
   * @param cluster frontier 连通簇。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 按靠近簇中心优先排序后的候选目标点。
   */
  std::vector<Point> clusterGoalCandidates(
    const std::vector<int> & cluster, int width, int height) const
  {
    if (cluster.empty()) {
      return {};
    }

    double center_x = 0.0;
    double center_y = 0.0;
    for (const int cell : cluster) {
      center_x += static_cast<double>(cell % width);
      center_y += static_cast<double>(cell / width);
    }
    center_x /= static_cast<double>(cluster.size());
    center_y /= static_cast<double>(cluster.size());

    std::vector<std::pair<double, int>> safe_cells;
    std::unordered_set<int> seen_safe_cells;
    safe_cells.reserve(cluster.size());

    for (const int frontier_cell : cluster) {
      // 原始 frontier 位于未知边界，通常离墙或未知区域过近。
      // 先向已知自由区回退，再去重得到最终可站立候选点。
      const auto stand_off_cell = offsetFromUnknown(frontier_cell, width, height);
      const auto safe_cell = nearestSafeCell(stand_off_cell, width, height);
      if (!safe_cell || !seen_safe_cells.insert(*safe_cell).second) {
        continue;
      }

      const double safe_x = static_cast<double>(*safe_cell % width);
      const double safe_y = static_cast<double>(*safe_cell / width);
      safe_cells.emplace_back(std::hypot(safe_x - center_x, safe_y - center_y), *safe_cell);
    }

    std::sort(
      safe_cells.begin(), safe_cells.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });

    constexpr std::size_t kMaxCandidatesPerCluster = 20;
    std::vector<Point> goals;
    goals.reserve(std::min(kMaxCandidatesPerCluster, safe_cells.size()));
    for (std::size_t i = 0; i < safe_cells.size() && i < kMaxCandidatesPerCluster; ++i) {
      const int cell = safe_cells[i].second;
      goals.push_back(mapToWorld(cell % width, cell / width));
    }
    return goals;
  }

  /**
   * @brief 使用 Nav2 全局规划器预检目标是否可规划。
   *
   * @param xy 候选目标点。
   * @param robot 当前机器人位置，用于设置目标朝向。
   */
  void preflightGoal(const Point & xy, const Point & robot)
  {
    ComputePathToPose::Goal goal;
    goal.goal = makePose(xy, std::atan2(xy.y - robot.y, xy.x - robot.x));
    goal.planner_id = "GridBased";
    goal.use_start = false;
    active_goal_ = xy;
    goal_started_at_ = std::chrono::steady_clock::now();

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, xy](const GoalHandlePlan::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "Planner rejected frontier at x=%.2f, y=%.2f", xy.x, xy.y);
          blacklist_.push_back(xy);
          active_goal_.reset();
          plan_handle_.reset();
          return;
        }
        plan_handle_ = handle;
      };
    options.result_callback =
      [this, xy, robot](const GoalHandlePlan::WrappedResult & result) {
        plan_handle_.reset();
        const bool has_path =
          result.result && !result.result->path.poses.empty();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !has_path) {
          RCLCPP_WARN(
            get_logger(), "Planner found no path to frontier at x=%.2f, y=%.2f", xy.x, xy.y);
          blacklist_.push_back(xy);
          active_goal_.reset();
          return;
        }
        sendGoal(xy, robot);
      };

    planner_->async_send_goal(goal, options);
  }

  /**
   * @brief 向 Nav2 NavigateToPose action 发送探索目标。
   *
   * @param xy 探索目标点。
   * @param robot 当前机器人位置，用于设置目标朝向。
   */
  void sendGoal(const Point & xy, const Point & robot)
  {
    NavigateToPose::Goal goal;
    goal.pose = makePose(xy, std::atan2(xy.y - robot.y, xy.x - robot.x));
    exploration_started_ = true;
    active_goal_ = xy;
    goal_started_at_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Exploring frontier at x=%.2f, y=%.2f", xy.x, xy.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, xy](const GoalHandleNavigate::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "Exploration goal was rejected");
          blacklist_.push_back(xy);
          active_goal_.reset();
          goal_handle_.reset();
          return;
        }
        goal_handle_ = handle;
      };
    options.result_callback =
      [this, xy](const GoalHandleNavigate::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Frontier reached");
        } else {
          RCLCPP_WARN(get_logger(), "Frontier failed; skipping nearby goals");
          blacklist_.push_back(xy);
        }
        active_goal_.reset();
        goal_handle_.reset();
      };

    navigator_->async_send_goal(goal, options);
  }

  /**
   * @brief 取消执行时间过长的导航目标。
   *
   * 超时目标会加入黑名单，避免短时间内反复选择同一片不可达区域。
   */
  void cancelTimedOutGoal()
  {
    if (!active_goal_ || max_goal_duration_ <= 0.0) {
      return;
    }

    const auto elapsed_duration = std::chrono::steady_clock::now() - goal_started_at_;
    const double elapsed = std::chrono::duration<double>(elapsed_duration).count();
    if (elapsed < max_goal_duration_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Frontier timed out after %.1fs; skipping nearby goals", elapsed);
    blacklist_.push_back(*active_goal_);
    navigator_->async_cancel_goal(goal_handle_);
    active_goal_.reset();
    goal_handle_.reset();
  }

  /**
   * @brief 取消执行时间过长的规划预检请求。
   */
  void cancelTimedOutPlan()
  {
    if (!active_goal_ || max_goal_duration_ <= 0.0) {
      return;
    }

    const auto elapsed_duration = std::chrono::steady_clock::now() - goal_started_at_;
    const double elapsed = std::chrono::duration<double>(elapsed_duration).count();
    if (elapsed < std::min(10.0, max_goal_duration_)) {
      return;
    }

    RCLCPP_WARN(get_logger(), "Planner timed out after %.1fs; skipping frontier", elapsed);
    blacklist_.push_back(*active_goal_);
    planner_->async_cancel_goal(plan_handle_);
    active_goal_.reset();
    plan_handle_.reset();
  }

  /**
   * @brief 处理 action 请求已发送但长时间未返回 goal handle 的异常状态。
   */
  void cancelPendingAction()
  {
    if (!active_goal_ || max_goal_duration_ <= 0.0) {
      return;
    }

    const auto elapsed_duration = std::chrono::steady_clock::now() - goal_started_at_;
    const double elapsed = std::chrono::duration<double>(elapsed_duration).count();
    if (elapsed < std::min(10.0, max_goal_duration_)) {
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Exploration action did not provide a handle after %.1fs; skipping frontier",
      elapsed);
    blacklist_.push_back(*active_goal_);
    active_goal_.reset();
  }

  /**
   * @brief 生成 Nav2 使用的目标位姿消息。
   *
   * @param xy 目标位置，位于 map 坐标系。
   * @param yaw 目标朝向，单位弧度。
   * @return 带 frame_id 和时间戳的 PoseStamped 消息。
   */
  geometry_msgs::msg::PoseStamped makePose(const Point & xy, double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = xy.x;
    pose.pose.position.y = xy.y;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    return pose;
  }

  /**
   * @brief 将地图栅格坐标转换为世界坐标。
   *
   * @param x 栅格 x 坐标。
   * @param y 栅格 y 坐标。
   * @return 对应栅格中心点在 map 坐标系下的位置。
   */
  Point mapToWorld(int x, int y) const
  {
    const auto & info = map_->info;
    return Point{
      info.origin.position.x + (static_cast<double>(x) + 0.5) * info.resolution,
      info.origin.position.y + (static_cast<double>(y) + 0.5) * info.resolution};
  }

  /**
   * @brief 将世界坐标转换为地图一维栅格索引。
   *
   * @param point map 坐标系下的点。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 有效地图范围内的一维索引；越界时返回 std::nullopt。
   */
  std::optional<int> worldToMap(const Point & point, int width, int height) const
  {
    const auto & info = map_->info;
    const int x = static_cast<int>(std::floor((point.x - info.origin.position.x) / info.resolution));
    const int y = static_cast<int>(std::floor((point.y - info.origin.position.y) / info.resolution));
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return std::nullopt;
    }
    return y * width + x;
  }

  /**
   * @brief 使用当前占据栅格快速判断目标在地图上是否连通可达。
   *
   * 该检查不替代 Nav2 全局规划，只用于在发送 action 前过滤明显不可达的
   * frontier，减少撞墙区域和未知区域附近的无效目标。
   *
   * @param robot 机器人当前位置。
   * @param goal 候选目标点。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 机器人附近自由栅格与目标附近自由栅格连通时返回 true。
   */
  bool isReachableOnMap(const Point & robot, const Point & goal, int width, int height) const
  {
    const auto robot_seed = worldToMap(robot, width, height);
    const auto goal_seed = worldToMap(goal, width, height);
    if (!robot_seed || !goal_seed) {
      return false;
    }

    const auto start = nearestTraversableCell(*robot_seed, width, height, 0.40);
    const auto target = nearestTraversableCell(*goal_seed, width, height, 0.15);
    if (!start || !target) {
      return false;
    }
    if (*start == *target) {
      return true;
    }

    std::vector<std::uint8_t> visited(static_cast<std::size_t>(width * height), 0);
    std::deque<int> queue{*start};
    visited[static_cast<std::size_t>(*start)] = 1;

    while (!queue.empty()) {
      const int cell = queue.front();
      queue.pop_front();
      for (const int neighbor : neighbors(cell, width, height)) {
        const auto index = static_cast<std::size_t>(neighbor);
        if (visited[index] || !isPathTraversableCell(neighbor, width, height)) {
          continue;
        }
        if (neighbor == *target) {
          return true;
        }
        visited[index] = 1;
        queue.push_back(neighbor);
      }
    }
    return false;
  }

  /**
   * @brief 从种子栅格附近搜索最近的路径可通行栅格。
   *
   * @param seed 起始栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @param search_radius 搜索半径，单位米。
   * @return 最近的可通行栅格；没有找到时返回 std::nullopt。
   */
  std::optional<int> nearestTraversableCell(
    int seed, int width, int height, double search_radius) const
  {
    if (isPathTraversableCell(seed, width, height)) {
      return seed;
    }

    const int max_radius =
      std::max(1, static_cast<int>(std::ceil(search_radius / map_->info.resolution)));
    const int seed_x = seed % width;
    const int seed_y = seed / width;

    std::optional<int> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dy = -max_radius; dy <= max_radius; ++dy) {
      for (int dx = -max_radius; dx <= max_radius; ++dx) {
        const double distance = std::hypot(dx, dy) * map_->info.resolution;
        if (distance > search_radius || distance >= best_distance) {
          continue;
        }

        const int x = seed_x + dx;
        const int y = seed_y + dy;
        if (x < 0 || x >= width || y < 0 || y >= height) {
          continue;
        }

        const int candidate = y * width + x;
        if (isPathTraversableCell(candidate, width, height)) {
          best = candidate;
          best_distance = distance;
        }
      }
    }
    return best;
  }

  /**
   * @brief 判断栅格是否是已知自由空间。
   *
   * @param cell 一维栅格索引。
   * @return 栅格占据值在自由阈值内时返回 true。
   */
  bool isTraversableCell(int cell) const
  {
    const auto value = map_->data[cell];
    return value >= 0 && value <= free_threshold_;
  }

  /**
   * @brief 判断栅格是否满足路径通行要求。
   *
   * 除了自身必须是自由栅格，还要求周围指定半径内没有障碍，避免路径
   * 被规划到贴墙区域。
   *
   * @param cell 一维栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 满足通行安全距离时返回 true。
   */
  bool isPathTraversableCell(int cell, int width, int height) const
  {
    if (!isTraversableCell(cell)) {
      return false;
    }

    const int x = cell % width;
    const int y = cell / width;
    const int radius =
      std::max(1, static_cast<int>(std::ceil(path_obstacle_clearance_ / map_->info.resolution)));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::hypot(dx, dy) * map_->info.resolution > path_obstacle_clearance_) {
          continue;
        }

        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          return false;
        }
        if (map_->data[ny * width + nx] >= occupied_threshold_) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * @brief 将 frontier 栅格沿远离未知区域的方向偏移。
   *
   * @param cell frontier 栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 偏移后的栅格索引。
   */
  int offsetFromUnknown(int cell, int width, int height) const
  {
    const int x = cell % width;
    const int y = cell / width;
    double away_x = 0.0;
    double away_y = 0.0;

    const auto & data = map_->data;
    for (const int neighbor : neighbors(cell, width, height)) {
      if (data[neighbor] != -1) {
        continue;
      }
      away_x += static_cast<double>(x - neighbor % width);
      away_y += static_cast<double>(y - neighbor / width);
    }

    const double norm = std::hypot(away_x, away_y);
    if (norm <= 1e-6) {
      return cell;
    }

    const double cells = frontier_goal_offset_ / map_->info.resolution;
    const int goal_x = std::clamp(
      static_cast<int>(std::lround(static_cast<double>(x) + away_x / norm * cells)), 0, width - 1);
    const int goal_y = std::clamp(
      static_cast<int>(std::lround(static_cast<double>(y) + away_y / norm * cells)), 0, height - 1);
    return goal_y * width + goal_x;
  }

  /**
   * @brief 从种子栅格附近搜索最近的安全目标栅格。
   *
   * @param seed 起始栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 最近的安全目标栅格；没有找到时返回 std::nullopt。
   */
  std::optional<int> nearestSafeCell(int seed, int width, int height) const
  {
    const int max_radius =
      std::max(1, static_cast<int>(std::ceil(goal_search_radius_ / map_->info.resolution)));
    const int seed_x = seed % width;
    const int seed_y = seed / width;

    std::optional<int> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dy = -max_radius; dy <= max_radius; ++dy) {
      for (int dx = -max_radius; dx <= max_radius; ++dx) {
        const int x = seed_x + dx;
        const int y = seed_y + dy;
        if (x < 0 || x >= width || y < 0 || y >= height) {
          continue;
        }
        const double distance = std::hypot(dx, dy) * map_->info.resolution;
        if (distance > goal_search_radius_) {
          continue;
        }
        const int candidate = y * width + x;
        if (distance < best_distance && isSafeGoalCell(candidate, width, height)) {
          best = candidate;
          best_distance = distance;
        }
      }
    }
    return best;
  }

  /**
   * @brief 判断栅格是否适合作为导航目标。
   *
   * 目标栅格必须是已知自由空间，并且在最小障碍物安全距离内没有障碍。
   *
   * @param cell 一维栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 满足目标安全条件时返回 true。
   */
  bool isSafeGoalCell(int cell, int width, int height) const
  {
    const auto & data = map_->data;
    if (data[cell] < 0 || data[cell] > free_threshold_) {
      return false;
    }

    const int x = cell % width;
    const int y = cell / width;
    const int radius =
      std::max(1, static_cast<int>(std::ceil(min_goal_obstacle_clearance_ / map_->info.resolution)));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::hypot(dx, dy) * map_->info.resolution > min_goal_obstacle_clearance_) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }
        if (data[ny * width + nx] >= occupied_threshold_) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * @brief 判断栅格 8 邻域是否接触未知区域。
   *
   * @param index 一维栅格索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 任一邻居为未知栅格时返回 true。
   */
  bool touchesUnknown(int index, int width, int height) const
  {
    const auto & data = map_->data;
    for (const int neighbor : neighbors(index, width, height)) {
      if (data[neighbor] == -1) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 判断目标是否落在失败目标黑名单附近。
   *
   * @param xy 待检查目标点。
   * @return 与任一黑名单点距离过近时返回 true。
   */
  bool isBlacklisted(const Point & xy) const
  {
    return std::any_of(
      blacklist_.begin(), blacklist_.end(),
      [this, &xy](const Point & blocked) {
        return distanceBetween(xy, blocked) < goal_blacklist_radius_;
      });
  }

  /**
   * @brief 获取栅格的 8 邻域索引。
   *
   * @param index 当前栅格的一维索引。
   * @param width 地图宽度，单位栅格。
   * @param height 地图高度，单位栅格。
   * @return 位于地图范围内的邻居索引。
   */
  static std::vector<int> neighbors(int index, int width, int height)
  {
    const int x = index % width;
    const int y = index / width;
    std::vector<int> output;
    output.reserve(8);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
          output.push_back(ny * width + nx);
        }
      }
    }
    return output;
  }

  /**
   * @brief 计算两个世界坐标点之间的欧氏距离。
   *
   * @param a 第一个点。
   * @param b 第二个点。
   * @return 两点间距离，单位米。
   */
  static double distanceBetween(const Point & a, const Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  std::string map_topic_;  ///< 订阅的占据栅格地图话题。
  std::string global_frame_;  ///< 地图和导航目标所在的全局坐标系。
  std::string robot_frame_;  ///< 用于 TF 查询的机器人底盘坐标系。
  double plan_period_{2.0};  ///< 探索规划周期，单位秒。
  int min_frontier_size_{8};  ///< frontier 连通簇的最小栅格数量。
  int free_threshold_{20};  ///< 小于等于该值的栅格视为自由空间。
  int occupied_threshold_{65};  ///< 大于等于该值的栅格视为障碍。
  double frontier_goal_offset_{0.35};  ///< frontier 目标向已知自由区回退距离，单位米。
  double goal_search_radius_{0.70};  ///< 从回退点搜索安全目标的半径，单位米。
  double min_goal_obstacle_clearance_{0.30};  ///< 目标点周围最小障碍物间距，单位米。
  double path_obstacle_clearance_{0.30};  ///< 连通性预检使用的路径安全间距，单位米。
  double goal_blacklist_radius_{0.45};  ///< 失败目标黑名单影响半径，单位米。
  double goal_reached_radius_{0.35};  ///< 小于该距离的候选目标视为已到达，单位米。
  double max_goal_duration_{120.0};  ///< 单个目标或规划请求的最大耗时，单位秒。
  int completion_idle_cycles_{8};  ///< 连续无可达 frontier 后判定探索完成的周期数。
  int no_reachable_cycles_{0};  ///< 当前连续无可达 frontier 的周期计数。
  bool auto_save_map_{true};  ///< 探索完成后是否自动保存地图。
  bool completed_{false};  ///< 探索是否已经完成。
  bool exploration_started_{false};  ///< 是否已经至少发送过一个导航目标。
  std::string map_save_service_;  ///< Nav2 map_saver 服务名。
  std::string map_save_path_;  ///< 地图保存路径前缀。

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;  ///< 最近一次收到的占据栅格地图。
  std::vector<Point> blacklist_;  ///< 失败或超时目标列表。
  GoalHandleNavigate::SharedPtr goal_handle_;  ///< 当前导航 action 的 goal handle。
  GoalHandlePlan::SharedPtr plan_handle_;  ///< 当前规划预检 action 的 goal handle。
  std::optional<Point> active_goal_;  ///< 当前正在处理的探索目标。
  std::chrono::steady_clock::time_point goal_started_at_;  ///< 当前目标开始处理的墙钟时间。
  rclcpp::Client<SaveMap>::SharedPtr map_saver_;  ///< 地图保存服务客户端。

  tf2_ros::Buffer tf_buffer_;  ///< 用于查询 map 到 base_footprint 的 TF 缓冲区。
  tf2_ros::TransformListener tf_listener_;  ///< TF 监听器。
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigator_;  ///< Nav2 导航 action 客户端。
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_;  ///< Nav2 全局规划 action 客户端。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;  ///< 地图订阅器。
  rclcpp::TimerBase::SharedPtr timer_;  ///< 周期性探索定时器。
};

}  // namespace exploration
}  // namespace ros2_all_wheel_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_all_wheel_sim::exploration::FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
