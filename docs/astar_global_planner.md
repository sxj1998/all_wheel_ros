# A* 全局路径规划器实现说明

本文档说明 `ros2_all_wheel_sim` 中自定义 Nav2 全局规划器的实现。源码位于：

- `src/ros2_all_wheel_sim/include/ros2_all_wheel_sim/global_planner/astar_global_planner.hpp`
- `src/ros2_all_wheel_sim/src/global_planner/astar_global_planner.cpp`
- `src/ros2_all_wheel_sim/plugins.xml`
- `src/ros2_all_wheel_sim/config/nav2_params.yaml`

## 1. 模块定位

该规划器实现了 Nav2 的 `nav2_core::GlobalPlanner` 接口，作为 `planner_server` 的全局路径规划插件使用。

它负责把：

- 当前机器人位姿 `start`
- RViz 或 action 发送的目标位姿 `goal`
- Nav2 全局代价地图 `global_costmap`

转换成一条 `nav_msgs/msg/Path`，交给后续的 smoother 和 controller 执行。

当前配置中，插件名字仍然叫 `GridBased`：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "ros2_all_wheel_sim/AStarGlobalPlanner"
```

这样做是为了兼容 Nav2 默认行为树。默认行为树会请求名为 `GridBased` 的规划器；如果这里改成 `AStar`，RViz 发送目标时可能会出现：

```text
planner GridBased is not a valid planner
```

因此，外部名字是 `GridBased`，内部实现是 `AStarGlobalPlanner`。

## 2. 插件注册

`plugins.xml` 中将 C++ 类注册为 Nav2 全局规划器插件：

```xml
<library path="custom_nav2_global_planner">
  <class
    name="ros2_all_wheel_sim/AStarGlobalPlanner"
    type="ros2_all_wheel_sim::global_planner::AStarGlobalPlanner"
    base_class_type="nav2_core::GlobalPlanner">
  </class>
</library>
```

`CMakeLists.txt` 通过 `pluginlib_export_plugin_description_file(nav2_core plugins.xml)` 把该插件导出给 pluginlib。运行时 `planner_server` 会根据 YAML 中的 `plugin` 字段加载动态库 `libcustom_nav2_global_planner.so`。

## 3. 生命周期函数

类 `AStarGlobalPlanner` 继承自 `nav2_core::GlobalPlanner`，主要实现以下函数：

```cpp
void configure(...);
void cleanup();
void activate();
void deactivate();
nav_msgs::msg::Path createPlan(start, goal);
```

其中真正执行规划的是 `createPlan()`。

`configure()` 在 Nav2 lifecycle 的 configure 阶段调用，完成以下工作：

- 保存 lifecycle node、TF buffer、costmap_ros 指针。
- 从 `costmap_ros` 获取底层 `Costmap2D`。
- 获取全局坐标系名称，通常是 `map`。
- 声明并读取规划器参数。
- 记录日志，确认插件配置成功。

## 4. 参数说明

当前 `nav2_params.yaml` 中参数如下：

```yaml
GridBased:
  plugin: "ros2_all_wheel_sim/AStarGlobalPlanner"
  tolerance: 0.5
  allow_unknown: false
  use_8_connected: true
  cost_penalty: 6.0
  interpolation_resolution: 0.10
  obstacle_threshold: 220
```

参数含义：

| 参数 | 含义 |
|---|---|
| `tolerance` | 如果起点或终点落在不可通行栅格上，允许在该半径内寻找最近可通行栅格。单位 m。 |
| `allow_unknown` | 是否允许穿过未知区域。当前为 `false`。 |
| `use_8_connected` | 是否使用 8 邻接搜索。`true` 时允许斜向移动。 |
| `cost_penalty` | 代价地图 cost 对路径代价的惩罚系数。越大越倾向远离高代价区域。 |
| `interpolation_resolution` | 输出路径插值间距。当前约每 0.10 m 一个 path pose。 |
| `obstacle_threshold` | 小于该 cost 的栅格才认为可通行。当前为 220，比 lethal obstacle 更保守。 |

## 5. createPlan() 执行流程

`createPlan()` 是全局规划的主入口。整体流程如下：

```text
1. 创建空 Path，设置 header.frame_id = map
2. 检查 costmap 是否可用
3. 将 start / goal 世界坐标转换为 costmap 栅格坐标
4. 如果起点或终点不可通行，在 tolerance 范围内寻找最近可通行栅格
5. 初始化 A* 搜索数据结构
6. 在 costmap 上执行 A* 搜索
7. 如果找到路径，回溯 parents 得到栅格路径
8. 对栅格路径做 line-of-sight 简化
9. 将路径转成世界坐标并插值
10. 输出 nav_msgs/msg/Path
```

如果任何关键步骤失败，例如起点在地图外、目标在地图外、找不到可通行目标、A* 搜索失败，则返回空路径。

重构后的代码中，`createPlan()` 不再直接写完整 A* 细节，而是作为流程编排函数存在：

```text
createPlan()
  -> makeEmptyPath()
  -> poseToGridCell(start)
  -> poseToGridCell(goal)
  -> normalizeEndpoint(start_cell)
  -> normalizeEndpoint(goal_cell)
  -> searchPath(start_cell, goal_cell)
  -> buildPath(parents, start_index, goal_index, start, goal)
```

这样拆分的好处是每个函数只负责一个明确动作：

- `poseToGridCell()` 只关心坐标转换。
- `normalizeEndpoint()` 只关心端点是否可通行。
- `searchPath()` 只关心 A* 搜索。
- `buildPath()` 只关心把搜索结果变成 Nav2 可跟踪的 `Path`。

后续如果要替换启发函数、调节代价函数、换路径平滑方式，不需要改动整个 `createPlan()`。

### 5.1 一个完整示例

假设有一个 7x5 的小地图，`S` 是起点，`G` 是目标，`#` 是障碍，`.` 是可通行区域：

```text
y=4  . . . . . . .
y=3  . # # # . # .
y=2  S . . # . # G
y=1  . # . . . . .
y=0  . . . . # . .
     x=0 1 2 3 4 5 6
```

起点和终点是：

```text
start_cell = (0, 2)
goal_cell  = (6, 2)
```

如果使用 8 邻接，A* 可以走斜线。它不会直接从 `(0,2)` 直冲 `(6,2)`，因为中间 `(3,2)`、`(5,2)` 附近有障碍。搜索过程会逐步选择 `f = g + h` 最小的候选点，绕开障碍，最后得到类似下面的栅格路径：

```text
(0,2) -> (1,2) -> (2,2) -> (3,1) -> (4,1) -> (5,1) -> (6,2)
```

这条路径随后会进入 `simplifyGridPath()`。如果 `(0,2)` 到 `(2,2)` 之间没有障碍，中间点可能被省略；如果 `(2,2)` 到 `(4,1)` 可以直连，也可能继续合并。最终输出给 Nav2 controller 的不是“每个 cell 一个点”的锯齿路径，而是一条经过简化和插值的连续路径。

## 6. 坐标转换

Nav2 goal 和 robot pose 是世界坐标，例如 `map` frame 下的米制坐标：

```text
x = 0.955
y = 1.685
```

A* 搜索在栅格地图上进行，所以先调用：

```cpp
costmap_->worldToMap(wx, wy, mx, my)
```

将世界坐标转换为栅格坐标：

```text
world: x/y, 单位 m
map:   mx/my, 单位 cell
```

搜索完成后，再用：

```cpp
costmap_->mapToWorld(mx, my, wx, wy)
```

将栅格路径转换回 `nav_msgs/msg/Path` 所需的世界坐标。

例如 costmap 分辨率为 0.05 m，地图原点为 `(origin_x, origin_y) = (-2.0, -2.0)`。一个世界坐标：

```text
world_x = 0.95
world_y = 1.65
```

大致会落到：

```text
mx = floor((0.95 - (-2.0)) / 0.05) = 59
my = floor((1.65 - (-2.0)) / 0.05) = 73
```

实际代码不手写这个公式，而是调用 `costmap_->worldToMap()`，这样可以复用 Nav2 对原点、分辨率和边界检查的处理。

## 7. 栅格索引

内部使用一维数组存储每个 cell 的搜索状态。二维坐标到一维索引的转换为：

```cpp
index = y * size_x + x;
```

对应函数：

```cpp
unsigned int toIndex(unsigned int x, unsigned int y) const;
```

使用一维数组的原因是：

- `g_score` 可以快速按 index 访问。
- `parents` 可以记录每个 cell 的父节点。
- `closed` 可以记录 cell 是否已完成搜索。

例子：如果地图宽度 `size_x = 7`，栅格 `(x=4, y=2)` 的索引是：

```text
index = 2 * 7 + 4 = 18
```

所以：

```text
g_score[18]  表示从起点走到 (4,2) 的当前最小代价
parents[18]  表示 (4,2) 是从哪个父节点走过来的
closed[18]   表示 (4,2) 是否已经完成扩展
```

## 8. 可通行判断

函数：

```cpp
bool isCellTraversable(unsigned int x, unsigned int y) const;
```

逻辑如下：

```text
cost == NO_INFORMATION:
  返回 allow_unknown

cost < obstacle_threshold:
  可通行

否则:
  不可通行
```

当前 `obstacle_threshold` 是 220。Nav2 costmap 中常见含义大致是：

- `0`: 完全自由
- `1~252`: 不同程度的代价
- `253`: inscribed inflated obstacle
- `254`: lethal obstacle
- `255`: unknown

因此设置为 220 会让规划器避开较高代价的膨胀区域，路径更保守，不容易贴墙。

例如：

| cell cost | 判断结果 | 原因 |
|---|---|---|
| `0` | 可通行 | 完全自由区域 |
| `120` | 可通行 | 小于 `obstacle_threshold=220` |
| `230` | 不可通行 | 高于阈值，离障碍太近 |
| `254` | 不可通行 | lethal obstacle |
| `255` | 取决于 `allow_unknown` | 未知区域 |

## 9. 起点/终点容错

有时目标点刚好点在障碍膨胀区、墙边、未知区或 cost 较高的 cell 上。此时直接规划会失败。

函数：

```cpp
bool findNearestTraversableCell(seed_x, seed_y, tolerance, cell) const;
```

会在 `tolerance` 半径内，以逐圈扩展的方式寻找最近可通行栅格。

当前默认：

```yaml
tolerance: 0.5
```

也就是允许在 0.5 m 内找一个可用替代点。

例如目标点落在 `(10, 8)`，但该栅格 cost 为 230，不可通行。`findNearestTraversableCell()` 会按半径逐圈尝试：

```text
radius = 0: 检查 (10,8)
radius = 1: 检查 (9,7) 到 (11,9) 外圈
radius = 2: 检查更外一圈
...
```

如果地图分辨率是 0.05 m，`tolerance = 0.5`，最多搜索：

```text
ceil(0.5 / 0.05) = 10 个 cell
```

也就是在目标点周围 10 格以内寻找最近可通行点。这样用户在 RViz 中点到墙边时，规划器仍有机会把目标修正到附近空地。

## 10. A* 搜索数据结构

实现中使用以下容器：

```cpp
std::vector<double> g_score;
std::vector<int> parents;
std::vector<bool> closed;
std::priority_queue<QueueEntry, ..., std::greater<QueueEntry>> open;
```

含义：

| 结构 | 含义 |
|---|---|
| `g_score[index]` | 从起点到该 cell 的当前最小已知代价。 |
| `parents[index]` | 路径回溯用的父节点 index。 |
| `closed[index]` | 是否已经完成扩展。 |
| `open` | 优先队列，按 `f_score` 从小到大取下一个候选 cell。 |

A* 的核心评价函数：

```text
f(n) = g(n) + h(n)
```

其中：

- `g(n)` 是从起点走到当前 cell 的累计代价。
- `h(n)` 是当前 cell 到目标 cell 的启发式估计。

### 10.1 open、closed、parents 如何协作

以起点 `(0,2)`、目标 `(6,2)` 为例，搜索刚开始时：

```text
open   = [(0,2)]
closed = []
g_score[(0,2)] = 0
parents[(0,2)] = (0,2)
```

第一轮从 `open` 取出 `(0,2)`，检查它周围的邻居。假设 `(1,2)`、`(0,1)`、`(0,3)` 可通行，它们会被加入 `open`：

```text
closed = [(0,2)]
open   = [(1,2), (0,1), (0,3), ...]
parents[(1,2)] = (0,2)
parents[(0,1)] = (0,2)
parents[(0,3)] = (0,2)
```

下一轮继续从 `open` 中取 `f_score` 最小的点。因为目标在右侧，`(1,2)` 的启发距离通常更小，所以它可能优先被扩展。

当某个点已经进入 `closed`，说明 A* 已经用当前最优方式处理过它；后续即使队列里还有重复条目，也会直接跳过。

## 11. 邻接方式

规划器支持 4 邻接和 8 邻接。

4 邻接：

```text
上、下、左、右
```

8 邻接：

```text
上、下、左、右、左上、右上、左下、右下
```

当前配置：

```yaml
use_8_connected: true
```

因此允许斜向移动，路径通常更短、更自然。

每次移动的基础距离为：

```cpp
step = hypot(nx - cx, ny - cy)
```

所以水平/竖直移动代价是 `1.0`，斜向移动代价约是 `1.414`。

例如从 `(2,2)` 扩展：

4 邻接只会考虑：

```text
(3,2), (2,3), (1,2), (2,1)
```

8 邻接会额外考虑：

```text
(3,3), (1,3), (1,1), (3,1)
```

对全向轮小车来说，8 邻接更符合实际运动能力，因为机器人可以更自然地沿斜向路线移动。

## 12. 代价函数

普通 A* 只考虑移动距离。本实现额外考虑 costmap cost：

```cpp
normalized_cost = cost / 252.0;
tentative_g = g_score[current] + step * (1.0 + cost_penalty_ * normalized_cost);
```

直观含义：

```text
越靠近障碍物，cost 越高，走该 cell 的代价越大。
```

当前：

```yaml
cost_penalty: 6.0
```

所以路径会明显倾向远离障碍物和膨胀区。

这对小车导航很重要，因为如果全局路径贴墙，局部控制器 `RegulatedPurePursuitController` 容易报：

```text
RegulatedPurePursuitController detected collision ahead
```

### 12.1 代价计算例子

假设当前从 `(2,2)` 移动到 `(3,2)`：

```text
step = 1.0
cost = 100
cost_penalty = 6.0
normalized_cost = 100 / 252 = 0.397
```

则移动代价是：

```text
traversal_cost = 1.0 * (1.0 + 6.0 * 0.397)
               = 3.382
```

如果另一个栅格更靠近障碍，`cost = 200`：

```text
normalized_cost = 200 / 252 = 0.794
traversal_cost = 1.0 * (1.0 + 6.0 * 0.794)
               = 5.764
```

虽然两个格子的几何距离一样，但第二个格子更靠近障碍，因此代价更高。A* 会更倾向选择第一条更安全的路线。

如果是斜向移动，例如 `(2,2)` 到 `(3,1)`：

```text
step = sqrt(2) = 1.414
cost = 80
normalized_cost = 80 / 252 = 0.317
traversal_cost = 1.414 * (1.0 + 6.0 * 0.317)
               = 4.105
```

这说明斜向移动既考虑几何距离，也考虑靠近障碍的风险。

## 13. 启发函数

启发函数为欧氏距离：

```cpp
h = hypot(goal_x - x, goal_y - y)
```

对应函数：

```cpp
double heuristic(...) const;
```

因为当前地图是二维栅格，且允许 8 邻接，欧氏距离是简单、自然、稳定的选择。

例如当前点是 `(2,2)`，目标点是 `(6,2)`：

```text
h = hypot(6 - 2, 2 - 2)
  = 4.0
```

当前点是 `(3,1)`，目标点是 `(6,2)`：

```text
h = hypot(6 - 3, 2 - 1)
  = sqrt(10)
  = 3.162
```

启发函数越小，表示该点在几何上越接近目标。但最终排序用的是 `f = g + h`，所以 A* 不会只贪心地追目标，也会考虑已经走过来的真实代价。

## 14. 路径回溯

A* 找到目标后，`parents` 数组中保存了每个 cell 的父节点。

回溯过程：

```text
current = goal_index
while current != start_index:
  push current
  current = parents[current]
push start_index
reverse
```

这样得到从起点到目标的原始栅格路径。

原始路径通常非常密集，因为地图分辨率是 0.05 m，每个 cell 都会成为一个路径点。

例子：搜索结束后，`parents` 可能记录了这样的关系：

```text
parents[(6,2)] = (5,1)
parents[(5,1)] = (4,1)
parents[(4,1)] = (3,1)
parents[(3,1)] = (2,2)
parents[(2,2)] = (1,2)
parents[(1,2)] = (0,2)
parents[(0,2)] = (0,2)
```

从目标反向追溯：

```text
(6,2) -> (5,1) -> (4,1) -> (3,1) -> (2,2) -> (1,2) -> (0,2)
```

反转后得到正向路径：

```text
(0,2) -> (1,2) -> (2,2) -> (3,1) -> (4,1) -> (5,1) -> (6,2)
```

## 15. 视线简化

为了避免输出锯齿路径，实现中加入了 line-of-sight 简化：

```cpp
std::vector<GridCell> simplifyGridPath(const std::vector<GridCell> & grid_path) const;
```

核心思路：

```text
从当前 anchor 出发，尽量向后找最远的、仍然能直线连通的路径点。
如果 anchor 到某个后续点之间没有障碍，就跳过中间点。
```

是否直线可达由：

```cpp
bool hasLineOfSight(const GridCell & from, const GridCell & to) const;
```

判断。内部使用类似 Bresenham 的栅格射线遍历，沿线检查每个 cell 是否可通行。

简化前：

```text
cell0 -> cell1 -> cell2 -> cell3 -> ... -> cellN
```

简化后：

```text
cell0 -> corner1 -> corner2 -> cellN
```

这样输出给局部控制器的路径更干净，不会因为每 5 cm 一个锯齿点而频繁调整方向。

例如原始路径是：

```text
(0,2) -> (1,2) -> (2,2) -> (3,1) -> (4,1) -> (5,1) -> (6,2)
```

如果 `(0,2)` 能直连 `(2,2)`，并且 `(2,2)` 能直连 `(6,2)`，简化后可能变成：

```text
(0,2) -> (2,2) -> (6,2)
```

但如果 `(2,2)` 到 `(6,2)` 的直线穿过障碍，`hasLineOfSight()` 会返回 false，算法就会退回到更近的可直连点，例如：

```text
(0,2) -> (2,2) -> (5,1) -> (6,2)
```

这一步不是重新规划，而是对已找到的安全路径做压缩。它只删除确认可以直连的中间点，不会让路径穿过障碍。

## 16. 路径插值

简化后的路径点较少，但 Nav2 controller 仍然需要一条连续可跟踪的 `Path`。

因此 `buildPath()` 会把简化后的世界坐标点按 `interpolation_resolution` 插值：

```yaml
interpolation_resolution: 0.10
```

也就是大约每 10 cm 生成一个 `PoseStamped`。

每段插值逻辑：

```text
1. 取当前点 (x0, y0) 和下一点 (x1, y1)
2. 计算距离 distance
3. steps = ceil(distance / interpolation_resolution)
4. 按 ratio = step / steps 生成中间点
5. yaw 使用该线段方向 atan2(dy, dx)
```

最后会强制加入原始 goal pose，保留用户给定目标点和目标朝向。

例如简化后的世界坐标路径为：

```text
P0 = (0.00, 0.00)
P1 = (0.30, 0.00)
P2 = (0.60, 0.20)
```

当 `interpolation_resolution = 0.10` 时，`P0 -> P1` 长度为 0.30 m，会生成约 3 个插值间隔：

```text
(0.00, 0.00)
(0.10, 0.00)
(0.20, 0.00)
```

然后最后追加 `P1` 到下一段，继续插值 `P1 -> P2`。相邻段连接处会通过 `skip_first_point` 避免重复点。

每个插值点的朝向由当前线段方向决定：

```text
yaw = atan2(y1 - y0, x1 - x0)
```

所以 `P0 -> P1` 的 yaw 是 0，`P1 -> P2` 的 yaw 会指向右上方。最后一个点使用用户原始 goal pose 的朝向，方便机器人到达目标后按 RViz 指定方向停稳。

## 17. 为什么保留精确起点/终点

内部搜索时，起点/终点可能会映射到 cell 中心，或者由于 `tolerance` 被替换到附近可通行 cell。

但输出给 Nav2 的路径使用：

```cpp
world_points.emplace_back(start.pose.position.x, start.pose.position.y);
world_points.emplace_back(goal.pose.position.x, goal.pose.position.y);
```

这样可以保证：

- 路径从机器人当前真实位姿开始。
- 路径最终到达用户指定目标点。
- RViz 中看到的 path 不会因为 cell 中心转换产生明显偏移。

## 18. 当前实现的优点

当前 A* 插件有几个适合本项目的特点：

- 使用 Nav2 原生 `Costmap2D`，能直接利用 static layer、obstacle layer、voxel layer、inflation layer。
- 通过 `cost_penalty` 远离障碍物，不只是简单避开 lethal obstacle。
- 通过 `obstacle_threshold` 让路径更保守。
- 支持 8 邻接，路径更自然。
- 支持起点/终点容错。
- 输出前做 line-of-sight 简化，减少锯齿。
- 插值后仍适合 Nav2 controller 跟踪。
- 插件名字保持 `GridBased`，兼容 Nav2 默认行为树。

## 19. 局限性

当前实现仍然是二维栅格 A*，不是完整的车辆动力学规划器。主要局限：

- 不考虑机器人朝向约束。
- 不考虑最小转弯半径。
- 不做时间维度规划。
- 不预测动态障碍物。
- line-of-sight 简化只检查 cell 可通行，没有额外做机器人 footprint 扫掠检测。
- 规划出来的是全局路径，真正避障和速度控制仍由局部控制器完成。

对全向轮小车来说，这些限制通常可以接受，因为机器人可以横向移动，运动约束比差速车更宽松。

## 20. 调试方法

启动导航：

```bash
cd ~/work/ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=maze2
```

查看 planner 日志：

```bash
tail -f ~/work/ROS2/logs/navigation_*/planner_server*.log
```

直接测试路径规划 action：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{planner_id: GridBased, use_start: true,
  start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
  goal: {header: {frame_id: map}, pose: {position: {x: 0.955, y: 1.685, z: 0.0}, orientation: {w: 1.0}}}}"
```

测试完整导航：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.955, y: 1.685, z: 0.0}, orientation: {w: 1.0}}}}"
```

如果规划失败，优先检查：

- 起点/目标是否在地图内。
- 起点/目标是否落在障碍物或膨胀区。
- `obstacle_threshold` 是否过低。
- `allow_unknown` 是否需要打开。
- `tolerance` 是否太小。
- `/global_costmap/costmap` 是否正确发布。
- `planner_server` 日志中是否加载了 `ros2_all_wheel_sim/AStarGlobalPlanner`。

### 20.1 打开 A* 点位追踪日志

代码中提供了编译期开关 `ASTAR_TRACE_POINTS`。默认关闭，不会输出大量点位日志。

打开方式：

```bash
cd ~/work/ROS2/all_wheel_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_all_wheel_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DASTAR_TRACE_POINTS=ON
source install/setup.bash
```

关闭方式：

```bash
cd ~/work/ROS2/all_wheel_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_all_wheel_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DASTAR_TRACE_POINTS=OFF
source install/setup.bash
```

打开后，日志会统一带有前缀：

```text
[AStarTrace]
```

因此可以只看 A* 点位日志：

```bash
tail -f ~/work/ROS2/logs/navigation_*/planner_server*.log | grep AStarTrace
```

或者启动时直接看终端输出：

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=maze2 | grep AStarTrace
```

点位追踪会打印这些信息：

| 日志内容 | 作用 |
|---|---|
| `createPlan start_world / goal_world` | 查看 RViz 或 action 传进来的真实世界坐标。 |
| `raw_start_cell / raw_goal_cell` | 查看世界坐标转换后的 costmap 栅格坐标。 |
| `normalized_start_cell / normalized_goal_cell` | 查看端点容错后实际参与搜索的栅格。 |
| `expand cell` | 查看 A* 当前正在扩展哪个栅格。 |
| `update neighbor` | 查看哪个邻居被更新，以及它的 `g/h/f` 代价。 |
| `reconstructed_grid_path` | 查看 A* 回溯出来的原始栅格路径。 |
| `simplified_grid_path` | 查看 line-of-sight 简化后的栅格路径。 |
| `world_key_points` | 查看插值前的世界坐标关键点。 |
| `interpolate segment` | 查看每段世界坐标路径如何被插值。 |
| `final path poses` | 查看最终输出给 Nav2 controller 的路径点数量。 |

示例日志：

```text
[AStarTrace] createPlan start_world=(0.000, 0.000) goal_world=(0.955, 1.685) frame=map
[AStarTrace] raw_start_cell=(40,40)
[AStarTrace] raw_goal_cell=(59,73)
[AStarTrace] expand cell=(40,40) index=4040 f=38.013 g=0.000 open_size=0
[AStarTrace] update neighbor=(41,40) parent=(40,40) index=4041 cost=0 g=1.000 h=37.483 f=38.483
[AStarTrace] reconstructed_grid_path size=24 (40,40) -> ... -> (59,73)
[AStarTrace] simplified_grid_path size=5 (40,40) -> ... -> (59,73)
[AStarTrace] final path poses=28
```

注意：`expand cell` 和 `update neighbor` 会很多，只建议在分析路径问题时临时打开。正常导航建议关闭该宏，避免日志过大。

## 21. 关键代码对应表

| 功能 | 函数 |
|---|---|
| Nav2 插件配置 | `configure()` |
| 生命周期激活 | `activate()` |
| 生命周期清理 | `cleanup()` |
| 规划主入口 | `createPlan()` |
| 创建空路径 | `makeEmptyPath()` |
| pose 转栅格坐标 | `poseToGridCell()` |
| 起点/终点归一化 | `normalizeEndpoint()` |
| A* 搜索主逻辑 | `searchPath()` |
| 4/8 邻接方向选择 | `neighborOffsets()` |
| 移动代价计算 | `traversalCost()` |
| 二维 cell 转一维 index | `toIndex()` |
| 判断 cell 是否可通行 | `isCellTraversable()` |
| 起点/终点容错 | `findNearestTraversableCell()` |
| A* 启发函数 | `heuristic()` |
| 直线可达检测 | `hasLineOfSight()` |
| 路径简化 | `simplifyGridPath()` |
| 路径回溯 | `reconstructGridPath()` |
| 栅格路径转世界坐标 | `toWorldPoints()` |
| 分段插值 | `appendInterpolatedSegment()` |
| 创建路径点 pose | `makePose()` |
| Path 编排构建 | `buildPath()` |

## 22. 从 RViz 目标点到机器人运动的数据流

全局规划器不是单独运行的节点，而是 Nav2 导航链路中的一个插件。一次完整导航大致经过下面的数据流：

```text
RViz 2D Goal
  -> /navigate_to_pose action
  -> bt_navigator
  -> planner_server
  -> AStarGlobalPlanner::createPlan()
  -> nav_msgs/msg/Path
  -> controller_server
  -> /cmd_vel
  -> Gazebo 中的机器人模型
```

其中 A* 插件只负责中间这一步：

```text
start pose + goal pose + global costmap -> global path
```

它不直接控制轮子，也不直接发布 `/cmd_vel`。真正把路径转换成速度指令的是局部控制器，例如当前 Nav2 配置中的 `RegulatedPurePursuitController`。

所以调试时要分清楚问题属于哪一层：

| 现象 | 更可能的问题位置 |
|---|---|
| RViz 点目标后完全没有路径 | planner_server / A* 插件 / costmap |
| 有全局路径但机器人不动 | controller_server / TF / lifecycle 状态 |
| 有路径但贴墙后失败 | 全局路径代价参数或局部控制器碰撞检查 |
| 机器人走偏 | TF、odom、base frame、机器人模型或控制器参数 |

本 A* 插件的主要责任边界是：只要能够稳定输出不穿越障碍、离障碍相对安全的 `Path`，它就完成了全局规划职责。后续是否能顺利跟踪路径，还取决于局部控制器、机器人动力学模型和仿真物理参数。

## 23. A* 单轮迭代的详细过程

下面用一个更具体的局部例子说明 `searchPath()` 每一轮到底在做什么。

假设当前从 open 队列中取出的点是：

```text
current = (2, 2)
g(current) = 2.0
goal = (6, 2)
```

使用 8 邻接时，候选邻居是：

```text
(3,2), (3,3), (2,3), (1,3),
(1,2), (1,1), (2,1), (3,1)
```

算法会对每个邻居依次执行以下判断：

```text
1. 是否越界？
2. 是否可通行？
3. 是否已经进入 closed？
4. 计算 tentative_g
5. 如果 tentative_g 更小，则更新 parents 和 g_score
6. 计算 f_score，并把该邻居加入 open
```

例如邻居 `(3,2)`：

```text
step = 1.0
cost = 100
cost_penalty = 6.0
traversal_cost = 1.0 * (1.0 + 6.0 * 100 / 252)
               = 3.382

tentative_g = g(current) + traversal_cost
            = 2.0 + 3.382
            = 5.382

h = hypot(6 - 3, 2 - 2)
  = 3.0

f = g + h
  = 5.382 + 3.0
  = 8.382
```

如果 `tentative_g < g_score[(3,2)]`，说明这是一条到 `(3,2)` 更便宜的路线，于是更新：

```text
parents[(3,2)] = (2,2)
g_score[(3,2)] = 5.382
open.push((f=8.382, index=(3,2)))
```

再看邻居 `(3,1)`，它是斜向移动：

```text
step = 1.414
cost = 80
traversal_cost = 1.414 * (1.0 + 6.0 * 80 / 252)
               = 4.105

tentative_g = 2.0 + 4.105
            = 6.105

h = hypot(6 - 3, 2 - 1)
  = 3.162

f = 6.105 + 3.162
  = 9.267
```

虽然 `(3,1)` 在几何上也靠近目标，但因为斜向距离更长，且 cost 也参与惩罚，最终 `f` 可能比 `(3,2)` 大。A* 下一轮会优先扩展 `f` 更小的点。

这个过程不断重复，直到：

- 目标点进入 `closed`，说明找到路径。
- `open` 为空，说明没有可达路径。

## 24. 为什么使用 priority_queue

`open` 的作用是保存“已经发现、但还没有完成扩展”的候选点。A* 每次都要从里面取出 `f_score` 最小的点。

如果用普通数组，每轮都要遍历所有候选点找最小值：

```text
每轮复杂度约 O(n)
```

使用 `std::priority_queue` 后，插入和取最小候选点的成本更低：

```text
push: O(log n)
pop:  O(log n)
```

C++ 的 `priority_queue` 默认是大顶堆，也就是最大值优先。代码中定义了：

```cpp
bool operator>(const QueueNode & other) const
{
  return f_score > other.f_score;
}
```

并使用：

```cpp
std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;
```

这样就把默认行为改成了“小的 `f_score` 优先弹出”。

这里还有一个实现细节：同一个 cell 可能被多次加入 `open`。例如先找到一条较贵路线，后面又找到一条更便宜路线。代码没有主动删除旧条目，而是在弹出时检查：

```cpp
if (closed[current_index]) {
  continue;
}
```

这是一种常见写法，简单可靠。旧条目即使留在队列里，最终也会因为该 cell 已经进入 `closed` 而被跳过。

## 25. 参数如何影响规划结果

不同参数会明显改变路径风格。可以把它们理解成几组“旋钮”。

### 25.1 `cost_penalty`

`cost_penalty` 控制路径离障碍物多远。

```yaml
cost_penalty: 6.0
```

较小的值：

- 路径更短。
- 更可能贴近墙边或障碍膨胀区。
- 局部控制器更容易检测到前方碰撞。

较大的值：

- 路径更保守。
- 更愿意绕远一点走空旷区域。
- 在窄通道中可能因为代价过高而找不到理想路径。

调试建议：

```text
路径贴墙：适当增大 cost_penalty
路径绕得太远：适当减小 cost_penalty
```

### 25.2 `obstacle_threshold`

`obstacle_threshold` 决定 cost 达到多少就完全不允许通过。

```yaml
obstacle_threshold: 220
```

较低的阈值：

- 更保守。
- 更不愿意靠近障碍。
- 窄通道更容易被判定为不可通行。

较高的阈值：

- 更容易穿过窄通道。
- 但路径可能贴近障碍。

调试建议：

```text
窄通道规划失败：可适当增大 obstacle_threshold
路径离墙太近：可适当减小 obstacle_threshold 或增大 cost_penalty
```

### 25.3 `tolerance`

`tolerance` 只影响起点和终点附近的容错，不影响中间路径搜索。

```yaml
tolerance: 0.5
```

如果 RViz 目标点点在墙边，`tolerance` 允许规划器在附近寻找一个真正可通行的目标 cell。

调试建议：

```text
目标点稍微点偏就失败：增大 tolerance
目标必须严格到达点击位置：减小 tolerance
```

### 25.4 `interpolation_resolution`

`interpolation_resolution` 控制输出 Path 的点间距。

```yaml
interpolation_resolution: 0.10
```

较小的值：

- Path 点更密。
- controller 跟踪更连续。
- 消息更大，路径点更多。

较大的值：

- Path 点更少。
- 计算和传输更轻。
- 太大时局部控制器可能跟踪不够细腻。

对当前仿真车，`0.10 m` 是比较折中的值。

## 26. 为什么要先简化再插值

路径后处理的顺序是：

```text
A* 原始栅格路径 -> line-of-sight 简化 -> 世界坐标转换 -> 插值
```

不建议直接对原始 A* 路径插值，因为原始路径可能长这样：

```text
(0,0) -> (1,0) -> (2,0) -> (3,1) -> (4,1) -> (5,2)
```

它包含很多按栅格移动产生的小转折。如果直接插值，controller 会看到很多短小方向变化，机器人可能频繁调整方向。

先简化后，路径可能变成：

```text
(0,0) -> (2,0) -> (5,2)
```

再插值后，输出给 controller 的路径就是平滑连续的线段采样点：

```text
(0.0,0.0), (0.1,0.0), (0.2,0.0), ...
```

这种顺序兼顾了两件事：

- A* 保证拓扑上能绕开障碍。
- line-of-sight 和插值让路径更适合实际控制器跟踪。

## 27. 常见失败场景和定位方法

### 27.1 起点在地图外

日志通常类似：

```text
Start pose is outside the global costmap
```

原因可能是：

- `map -> odom -> base_footprint` TF 不正确。
- AMCL 定位还没稳定。
- global costmap 尺寸或 origin 不覆盖机器人位置。

优先检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /amcl_pose
ros2 topic echo /global_costmap/costmap --once
```

### 27.2 目标在地图外

日志通常类似：

```text
Goal pose is outside the global costmap
```

原因可能是 RViz 点到了地图外，或者使用了错误地图。检查当前启动命令中的 map 参数：

```bash
ros2 launch ros2_all_wheel_sim navigation_gazebo_sim.launch.py world:=maze2 map:=maze2
```

确认 RViz 中显示的 map 和实际加载的 `maze2.yaml` 一致。

### 27.3 起点或终点附近没有可通行栅格

日志通常类似：

```text
No traversable goal cell found within 0.50 m
```

说明目标点附近 `tolerance` 半径内都被判定为不可通行。可能原因：

- 点到了墙体内部。
- `obstacle_threshold` 太低。
- inflation layer 膨胀半径太大。
- 地图障碍边界和 Gazebo 世界不一致。

可以尝试：

```yaml
tolerance: 0.8
obstacle_threshold: 230
```

但如果目标确实在墙内，参数只能提高容错，不能让机器人穿墙。

### 27.4 A* 找不到路径

日志通常类似：

```text
A* could not find a path to the goal
```

这说明起点和终点本身都可通行，但中间没有连通路径。排查顺序：

```text
1. RViz 看 /global_costmap/costmap 是否把通道堵死
2. 检查 obstacle_threshold 是否太保守
3. 检查 allow_unknown 是否需要开启
4. 检查 map 是否和 world 匹配
5. 检查机器人 footprint/inflation 是否过大
```

如果地图中有窄门，A* 可能因为 `obstacle_threshold=220` 把门附近膨胀区判为不可通行。此时可以适当提高阈值，或减小 inflation 半径。

## 28. 阅读源码的推荐顺序

如果是第一次读这份代码，建议不要从 `searchPath()` 的循环细节开始。更好的顺序是：

```text
1. createPlan()
2. poseToGridCell()
3. normalizeEndpoint()
4. searchPath()
5. traversalCost()
6. heuristic()
7. reconstructGridPath()
8. simplifyGridPath()
9. toWorldPoints()
10. appendInterpolatedSegment()
11. buildPath()
```

这样读的原因是：

- 先看 `createPlan()`，知道整体流程。
- 再看坐标转换和端点处理，理解输入如何进入栅格世界。
- 然后看 A* 搜索，理解路径如何被找到。
- 最后看路径后处理，理解栅格路径如何变成 Nav2 controller 可跟踪的 `Path`。

也可以把这套实现理解成三个层次：

```text
接口层：configure(), createPlan()
算法层：searchPath(), traversalCost(), heuristic()
后处理层：reconstructGridPath(), simplifyGridPath(), toWorldPoints(), appendInterpolatedSegment()
```

这种分层就是这次重构的主要目标：让 Nav2 插件接口、A* 算法核心、路径后处理彼此独立，后续修改其中一层时不容易影响其他层。
