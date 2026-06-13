cmake_minimum_required(VERSION 3.8)
project(ros2_all_wheel_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ASTAR_TRACE_POINTS "Enable verbose A* planner point trace logs" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

set(KINEMATICS_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(NAV2_PLUGIN_DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_library(kinematics_core src/kinematics/kinematics_core.cpp)
target_include_directories(kinematics_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics_core PUBLIC Eigen3::Eigen)

add_executable(kinematics src/kinematics/kinematics.cpp)
target_link_libraries(kinematics kinematics_core)
ament_target_dependencies(kinematics ${KINEMATICS_DEPENDENCIES})
target_include_directories(kinematics PRIVATE include)

add_library(custom_nav2_global_planner SHARED
  src/global_planner/astar_global_planner.cpp
)
target_include_directories(custom_nav2_global_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(custom_nav2_global_planner ${NAV2_PLUGIN_DEPENDENCIES})
target_compile_definitions(custom_nav2_global_planner
  PRIVATE
    $<$<BOOL:${ASTAR_TRACE_POINTS}>:ASTAR_TRACE_POINTS=1>
)

add_library(custom_nav2_local_planner SHARED
  src/local_planner/omni_local_planner.cpp
)
target_include_directories(custom_nav2_local_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(custom_nav2_local_planner ${NAV2_PLUGIN_DEPENDENCIES})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS kinematics_core kinematics custom_nav2_global_planner custom_nav2_local_planner
  EXPORT export_kinematics_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  launch
  urdf
  rviz
  worlds
  meshes
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_kinematics_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
