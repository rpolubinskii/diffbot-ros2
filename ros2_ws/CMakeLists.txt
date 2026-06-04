cmake_minimum_required(VERSION 3.16)
project(diffbot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  action_msgs
  hardware_interface
  nav2_behavior_tree
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(behaviortree_cpp QUIET)
if(behaviortree_cpp_FOUND)
  set(BTCPP_DEPENDENCY behaviortree_cpp)
else()
  find_package(behaviortree_cpp_v3 REQUIRED)
  set(BTCPP_DEPENDENCY behaviortree_cpp_v3)
endif()

## COMPILE
add_library(
  diffbot
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(diffbot PUBLIC cxx_std_17)
target_include_directories(diffbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffbot>
)
ament_target_dependencies(
  diffbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(diffbot_lidar_standby_manager
  src/lidar_standby_manager.cpp
)
target_compile_features(diffbot_lidar_standby_manager PUBLIC cxx_std_17)
ament_target_dependencies(
  diffbot_lidar_standby_manager PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(diffbot_wait_for_fresh_odom_bt_node SHARED
  src/wait_for_fresh_odom_bt_node.cpp
)
target_compile_features(diffbot_wait_for_fresh_odom_bt_node PUBLIC cxx_std_17)
ament_target_dependencies(
  diffbot_wait_for_fresh_odom_bt_node PUBLIC
  nav_msgs
  nav2_behavior_tree
  rclcpp
  ${BTCPP_DEPENDENCY}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffbot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffbot
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/diffbot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffbot
)
install(
  DIRECTORY bringup/behavior_trees
  DESTINATION share/diffbot
)
install(TARGETS diffbot
  EXPORT export_diffbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS diffbot_lidar_standby_manager
  DESTINATION lib/diffbot
)
install(TARGETS diffbot_wait_for_fresh_odom_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_diffbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
