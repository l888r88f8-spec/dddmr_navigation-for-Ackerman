cmake_minimum_required(VERSION 3.5)
project(p2p_move_base)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(local_planner REQUIRED)
find_package(recovery_behaviors REQUIRED)
find_package(dddmr_sys_core REQUIRED)

find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  angles
  local_planner
  recovery_behaviors
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
)

# p2p_fsm
add_library(p2p_fsm SHARED
  src/p2p_fsm.cpp
)
target_link_libraries(p2p_fsm
  ${PCL_LIBRARIES}
)
ament_target_dependencies(p2p_fsm
  ${dependencies}
)

# p2p_global_plan_manager
add_library(p2p_global_plan_manager SHARED
  src/p2p_global_plan_manager.cpp
)
target_link_libraries(p2p_global_plan_manager
  ${PCL_LIBRARIES}
)
ament_target_dependencies(p2p_global_plan_manager
  ${dependencies}
)

add_library(p2p_navigation_fault_policy SHARED
  src/navigation_fault_policy.cpp
)
ament_target_dependencies(p2p_navigation_fault_policy
  ${dependencies}
)

add_library(p2p_runtime_budget_monitor SHARED
  src/runtime_budget_monitor.cpp
)
ament_target_dependencies(p2p_runtime_budget_monitor
  rclcpp
)

add_library(p2p_local_obstacle_layer SHARED
  src/local_obstacle_layer.cpp
)

add_library(p2p_local_avoidance_policy SHARED
  src/local_avoidance_policy.cpp
)
target_link_libraries(p2p_local_avoidance_policy
  p2p_local_obstacle_layer
)

add_library(p2p_local_obstacle_layer_ros SHARED
  src/local_obstacle_layer_ros.cpp
)
target_link_libraries(p2p_local_obstacle_layer_ros
  p2p_local_obstacle_layer
)
ament_target_dependencies(p2p_local_obstacle_layer_ros
  sensor_msgs
  std_msgs
)

# p2p_mov2_base
add_library(p2p_mov2_base SHARED
  src/p2p_move_base.cpp
)
target_link_libraries(p2p_mov2_base
  p2p_fsm
  p2p_global_plan_manager
  p2p_navigation_fault_policy
  p2p_runtime_budget_monitor
  p2p_local_obstacle_layer_ros
)
ament_target_dependencies(p2p_mov2_base
  ${dependencies}
)

#p2p_move_base_node
add_executable(p2p_move_base_node
  src/p2p_move_base_node.cpp
)
target_link_libraries(p2p_move_base_node 
  p2p_mov2_base
)
ament_target_dependencies(p2p_move_base_node
  ${dependencies}
)

install(TARGETS
  p2p_fsm
  p2p_mov2_base
  p2p_global_plan_manager
  p2p_navigation_fault_policy
  p2p_runtime_budget_monitor
  p2p_local_obstacle_layer
  p2p_local_avoidance_policy
  p2p_local_obstacle_layer_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  p2p_move_base_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  script/clicked2goal.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(
  p2p_fsm
  p2p_mov2_base
  p2p_global_plan_manager
  p2p_navigation_fault_policy
  p2p_runtime_budget_monitor
  p2p_local_obstacle_layer
  p2p_local_avoidance_policy
  p2p_local_obstacle_layer_ros
)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  ament_add_gtest(test_navigation_fault_policy
    test/test_navigation_fault_policy.cpp
  )
  target_link_libraries(test_navigation_fault_policy
    p2p_navigation_fault_policy
  )
  ament_target_dependencies(test_navigation_fault_policy
    ${dependencies}
  )

  ament_add_gtest(test_runtime_budget_monitor
    test/test_runtime_budget_monitor.cpp
  )
  target_link_libraries(test_runtime_budget_monitor
    p2p_runtime_budget_monitor
  )
  ament_target_dependencies(test_runtime_budget_monitor
    rclcpp
  )

  ament_add_gtest(test_local_obstacle_layer
    test/test_local_obstacle_layer.cpp
  )
  target_link_libraries(test_local_obstacle_layer
    p2p_local_obstacle_layer
  )

  ament_add_gtest(test_local_avoidance_policy
    test/test_local_avoidance_policy.cpp
  )
  target_link_libraries(test_local_avoidance_policy
    p2p_local_avoidance_policy
  )

  ament_add_gtest(test_local_obstacle_layer_ros
    test/test_local_obstacle_layer_ros.cpp
  )
  target_link_libraries(test_local_obstacle_layer_ros
    p2p_local_obstacle_layer_ros
  )
  ament_target_dependencies(test_local_obstacle_layer_ros
    sensor_msgs
    std_msgs
  )

  ament_add_gtest(test_p2p_move_base_config
    test/test_p2p_move_base_config.cpp
  )
  target_compile_definitions(test_p2p_move_base_config PRIVATE
    P2P_MOVE_BASE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
