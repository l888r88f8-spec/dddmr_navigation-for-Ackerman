cmake_minimum_required(VERSION 3.16)
project(dddmr_local_map CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(dddmr_local_map_core
  src/voxel_map.cpp
  src/terrain.cpp
  src/local_map.cpp
)
target_include_directories(dddmr_local_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dddmr_local_map_core PUBLIC Eigen3::Eigen)
target_compile_features(dddmr_local_map_core PUBLIC cxx_std_17)
target_compile_options(dddmr_local_map_core PRIVATE -Wall -Wextra -Wpedantic)
# This core is linked into the shared local_planner controller library. Keep it
# static but PIC so downstream shared libraries can link it on every platform.
set_target_properties(dddmr_local_map_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dddmr_local_map_core
  EXPORT dddmr_local_map_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md VEHICLE_GEOMETRY.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(dddmr_local_map_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

add_library(dddmr_local_map_ros src/ros_adapter.cpp)
target_link_libraries(dddmr_local_map_ros dddmr_local_map_core)
ament_target_dependencies(dddmr_local_map_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_compile_options(dddmr_local_map_ros PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(dddmr_local_map_ros PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

add_executable(local_map_node src/local_map_node.cpp)
target_link_libraries(local_map_node dddmr_local_map_ros)
target_compile_definitions(local_map_node PRIVATE
  DDDMR_LOCAL_MAP_ENTRY_NAME="local_map_node"
)

# Compatibility entry point. It runs the same process implementation and main
# loop as local_map_node; only the ROS node name differs.
add_executable(local_map_shadow src/local_map_node.cpp)
target_link_libraries(local_map_shadow dddmr_local_map_ros)
target_compile_definitions(local_map_shadow PRIVATE
  DDDMR_LOCAL_MAP_ENTRY_NAME="local_map_shadow"
)

add_executable(local_map_demo_scene src/local_map_demo_scene.cpp)
target_link_libraries(local_map_demo_scene dddmr_local_map_ros)
target_compile_options(local_map_demo_scene PRIVATE -Wall -Wextra -Wpedantic)

# Export only the ROS library. Node entry points must be installed where
# ros2 run/launch looks for package executables.
install(TARGETS dddmr_local_map_ros
  EXPORT dddmr_local_map_rosTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS local_map_node local_map_shadow local_map_demo_scene
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(dddmr_local_map_rosTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs tf2 tf2_ros geometry_msgs)

if(BUILD_TESTING AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map dddmr_local_map_core)

  ament_add_gtest(test_terrain test/test_terrain.cpp)
  target_link_libraries(test_terrain dddmr_local_map_core)

  ament_add_gtest(test_terrain_export test/test_terrain_export.cpp)
  target_link_libraries(test_terrain_export dddmr_local_map_core)

  ament_add_gtest(test_ros_adapter test/test_ros_adapter.cpp)
  target_link_libraries(test_ros_adapter dddmr_local_map_ros)
  target_compile_definitions(test_ros_adapter PRIVATE
    DDDMR_LOCAL_MAP_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )

  add_executable(dddmr_local_map_benchmark test/benchmark_core.cpp)
  target_link_libraries(dddmr_local_map_benchmark dddmr_local_map_core)

  add_launch_test(test/test_local_map_demo_launch.py
    TIMEOUT 90
  )
  add_launch_test(test/test_local_map_demo_custom_topic.py
    TIMEOUT 90
  )
  add_launch_test(test/test_local_map_rviz_launch.py
    TIMEOUT 60
  )
  # The host's /home/robot/.ros can be read-only. Keep launch_testing's own
  # log/home under the writable build tree so CTest needs no external env.
  set_tests_properties(
    test_test_local_map_demo_launch.py
    test_test_local_map_demo_custom_topic.py
    test_test_local_map_rviz_launch.py
    test_ros_adapter
    PROPERTIES
    ENVIRONMENT
      "ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/launch_test_ros_home;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/launch_test_ros_logs"
  )

  # Runtime process test for parameter rejection (not a mocked unit test).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_demo_scene_validates_parameters
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_demo_scene_invalid_parameters.py
            $<TARGET_FILE:local_map_demo_scene>
  )
  set_tests_properties(test_demo_scene_validates_parameters PROPERTIES
    TIMEOUT 30
    LABELS runtime
  )

  # No DISPLAY on the target host. This test parses the installed launch actions
  # to verify the default RViz view, namespace, effective Fixed Frame, demo
  # single-owner rule, and the real-input RViz resource without rendering.
  add_test(
    NAME test_local_map_rviz_config
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_local_map_rviz_config.py
  )
  set_tests_properties(test_local_map_rviz_config PROPERTIES
    TIMEOUT 60
    ENVIRONMENT
      "ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/launch_test_ros_home;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/launch_test_ros_logs"
  )
endif()

ament_package()
