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

class FrontierExplorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandlePlan = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using SaveMap = nav2_msgs::srv::SaveMap;

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
  struct Point
  {
    double x{0.0};
    double y{0.0};
  };

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

        // Larger frontiers are useful, but nearby frontiers keep exploration responsive.
        // The small candidate penalty keeps the cluster-center candidate preferred when it is valid,
        // while still allowing edge/corner candidates to recover partially explored rooms.
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
      // A raw frontier cell lies on the unknown boundary and is often too close to a wall.
      // Shift each candidate back into known free space, then deduplicate the resulting safe cells.
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

  Point mapToWorld(int x, int y) const
  {
    const auto & info = map_->info;
    return Point{
      info.origin.position.x + (static_cast<double>(x) + 0.5) * info.resolution,
      info.origin.position.y + (static_cast<double>(y) + 0.5) * info.resolution};
  }

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

  bool isTraversableCell(int cell) const
  {
    const auto value = map_->data[cell];
    return value >= 0 && value <= free_threshold_;
  }

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

  bool isBlacklisted(const Point & xy) const
  {
    return std::any_of(
      blacklist_.begin(), blacklist_.end(),
      [this, &xy](const Point & blocked) {
        return distanceBetween(xy, blocked) < goal_blacklist_radius_;
      });
  }

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

  static double distanceBetween(const Point & a, const Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  std::string map_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  double plan_period_{2.0};
  int min_frontier_size_{8};
  int free_threshold_{20};
  int occupied_threshold_{65};
  double frontier_goal_offset_{0.35};
  double goal_search_radius_{0.70};
  double min_goal_obstacle_clearance_{0.30};
  double path_obstacle_clearance_{0.30};
  double goal_blacklist_radius_{0.45};
  double goal_reached_radius_{0.35};
  double max_goal_duration_{120.0};
  int completion_idle_cycles_{8};
  int no_reachable_cycles_{0};
  bool auto_save_map_{true};
  bool completed_{false};
  bool exploration_started_{false};
  std::string map_save_service_;
  std::string map_save_path_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<Point> blacklist_;
  GoalHandleNavigate::SharedPtr goal_handle_;
  GoalHandlePlan::SharedPtr plan_handle_;
  std::optional<Point> active_goal_;
  std::chrono::steady_clock::time_point goal_started_at_;
  rclcpp::Client<SaveMap>::SharedPtr map_saver_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigator_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
