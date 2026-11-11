cmake_minimum_required(VERSION 3.5)
project(local_planner)

find_package(ament_cmake REQUIRED)
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
find_package(base_trajectory REQUIRED)
find_package(perception_3d REQUIRED)
find_package(mpc_critics REQUIRED)
find_package(trajectory_generators REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(dddmr_local_map REQUIRED)

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
  base_trajectory
  perception_3d
  mpc_critics
  trajectory_generators
  dddmr_sys_core
  dddmr_local_map
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
)


function(enable_default_optimized_runtime target_name)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND
     NOT CMAKE_CONFIGURATION_TYPES AND
     NOT CMAKE_BUILD_TYPE)
    # Keep real-time targets optimized for the common colcon invocation while
    # preserving explicit Debug/Release and multi-config generator settings.
    target_compile_options(${target_name} PRIVATE -O2)
  endif()
endfunction()

add_library(local_planner_mppi_controller SHARED
  src/mppi_critic_manager.cpp
  src/mppi_controller.cpp
  src/mppi_directed_retreat.cpp
  src/mppi_obstacle_geometry.cpp
  src/mppi_local_map_rollout.cpp
  src/mppi_measured_travel.cpp
)
target_include_directories(local_planner_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(local_planner_mppi_controller
  ${PCL_LIBRARIES}
  dddmr_local_map::dddmr_local_map_core
)
ament_target_dependencies(local_planner_mppi_controller
  ${dependencies}
)
enable_default_optimized_runtime(local_planner_mppi_controller)

# local_planner
add_library(local_planner SHARED
  src/local_planner.cpp
)
target_link_libraries(local_planner
  local_planner_mppi_controller
  dddmr_local_map::dddmr_local_map_ros
  ${PCL_LIBRARIES}
)
ament_target_dependencies(local_planner
  ${dependencies}
)
enable_default_optimized_runtime(local_planner)

#local_planner_node
add_executable(local_planner_node
  src/local_planner_node.cpp
)
target_link_libraries(local_planner_node 
  local_planner
)
ament_target_dependencies(local_planner_node
  ${dependencies}
)

#local_planner_play_ground_node
add_executable(local_planner_play_ground_node
  src/local_planner_play_ground_node.cpp
)
target_link_libraries(local_planner_play_ground_node 
  local_planner
)
ament_target_dependencies(local_planner_play_ground_node
  ${dependencies}
)

install(TARGETS
  local_planner_mppi_controller
  local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test-only cross-check against the authoritative driver conversion helper.
  # Production navigation does not link or depend on the driver package.
  set(HUNTER_DRIVER_INCLUDE_DIR
    "$ENV{HOME}/hunterse_ws/src/hunter_ros2/hunter_base/include"
    CACHE PATH "Optional Hunter driver include directory for cross-checks")
  if(EXISTS "${HUNTER_DRIVER_INCLUDE_DIR}/hunter_base/hunter_command.hpp")
    ament_add_gtest(test_driver_nav_geometry
      test/test_driver_nav_geometry.cpp
    )
    target_include_directories(test_driver_nav_geometry PRIVATE
      ${HUNTER_DRIVER_INCLUDE_DIR}
    )
    target_link_libraries(test_driver_nav_geometry
      local_planner_mppi_controller
    )
  else()
    message(STATUS
      "Skipping test_driver_nav_geometry: ${HUNTER_DRIVER_INCLUDE_DIR} "
      "does not contain hunter_base/hunter_command.hpp")
  endif()

  ament_add_gtest(test_mppi_directed_retreat
    test/test_mppi_directed_retreat.cpp
  )
  target_link_libraries(test_mppi_directed_retreat
    local_planner_mppi_controller
  )

  ament_add_gtest(test_mppi_controller
    test/test_mppi_controller.cpp
  )
  target_link_libraries(test_mppi_controller
    local_planner_mppi_controller
  )

  ament_add_gtest(test_mppi_critic_manager
    test/test_mppi_critic_manager.cpp
  )
  target_link_libraries(test_mppi_critic_manager
    local_planner_mppi_controller
  )
  ament_add_gtest(test_mppi_measured_travel
    test/test_mppi_measured_travel.cpp
  )
  target_link_libraries(test_mppi_measured_travel
    local_planner_mppi_controller
  )

  ament_add_gtest(test_mppi_local_map_rollout
    test/test_mppi_local_map_rollout.cpp
  )
  target_link_libraries(test_mppi_local_map_rollout
    local_planner_mppi_controller
  )

  # Diagnostic aggregated_pc publish-cache decision (header-only helper).
  ament_add_gtest(test_published_cloud_cache
    test/test_published_cloud_cache.cpp
  )
  target_include_directories(test_published_cloud_cache PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_published_cloud_cache
    ${PCL_LIBRARIES}
  )

  # Diagnostic topic gate: publish only while a subscriber exists.
  ament_add_gtest(test_debug_publish
    test/test_debug_publish.cpp
  )
  target_include_directories(test_debug_publish PRIVATE
    include
  )

  # Deterministic model-in-the-loop simulation of the bypass strategy. It runs
  # the production controller/critic/classifier code against a synthetic world
  # and a bicycle model; it is not a hardware or ROS-stack validation.
  ament_add_gtest(test_mppi_bypass_simulation
    test/test_mppi_bypass_simulation.cpp
    TIMEOUT 900
  )
  target_link_libraries(test_mppi_bypass_simulation
    local_planner_mppi_controller
  )
  enable_default_optimized_runtime(test_mppi_local_map_rollout)

  enable_default_optimized_runtime(test_mppi_critic_manager)
endif()

#install node
install(TARGETS
  local_planner_node
  local_planner_play_ground_node
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
  #script/example_client.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(local_planner_mppi_controller local_planner)
ament_export_dependencies(${dependencies})
ament_package()
