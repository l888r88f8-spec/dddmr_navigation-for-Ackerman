cmake_minimum_required(VERSION 3.16)
# rosidl_generate_interfaces() needs the C language for the generated C
# typesupport libraries, so enable C alongside the package's C++ code.
project(grid_global_planner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "grid_global_planner: defaulting CMAKE_BUILD_TYPE to Release")
endif()

include(CTest)

option(
  GRID_STANDALONE
  "Build the ROS-independent terrain core and tests without ament/ROS"
  OFF)
option(
  GRID_BUILD_TESTS
  "Build the C++/Python tests when BUILD_TESTING is enabled"
  ON)
option(
  GRID_BUILD_BENCHMARKS
  "Build the snapshot/search benchmark executable when tests are enabled"
  ON)
option(
  GRID_ENABLE_TSAN
  "Compile the core and tests with ThreadSanitizer; unsupported platforms must report the failure"
  OFF)

if(GRID_ENABLE_TSAN)
  message(STATUS "grid_global_planner: ThreadSanitizer enabled")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=thread)
endif()

find_package(Threads REQUIRED)

# Stage 5 core library. The map-update worker and snapshot store are part of the
# same ROS-independent library so the standalone path remains buildable.
set(GRID_CORE_SOURCES
  src/ackermann_kinematics.cpp
  src/continuous_ground_surface.cpp
  src/corridor_builder.cpp
  src/distance_field.cpp
  src/grid_graph_astar.cpp
  src/grid_hybrid_astar.cpp
  src/ground_surface_reconstruction.cpp
  src/input_contract.cpp
  src/latest_validation.cpp
  src/map_snapshot.cpp
  src/map_update_worker.cpp
  src/measurement_evidence.cpp
  src/multiresolution_map.cpp
  src/obstacle_tile.cpp
  src/offline_geometry.cpp
  src/offline_static_map.cpp
  src/offline_surface_evidence.cpp
  src/path_validator.cpp
  src/plan_commit_delivery.cpp
  src/snapshot_store.cpp
  src/source_observation.cpp
  src/terrain_grid_builder.cpp
  src/terrain_boundary_cost.cpp
  src/terrain_types.cpp
  src/wheel_support.cpp
)

add_library(grid_terrain_core STATIC ${GRID_CORE_SOURCES})
target_include_directories(
  grid_terrain_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(grid_terrain_core PUBLIC cxx_std_17)
target_link_libraries(grid_terrain_core PUBLIC Threads::Threads)
# The core is linked into the ROS shared library, so its objects must be
# position independent even though the archive itself is STATIC.
set_target_properties(grid_terrain_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(
  grid_terrain_core
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow)

if(NOT GRID_STANDALONE)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED)
  find_package(dddmr_sys_core REQUIRED)
  find_package(perception_3d REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # Explicit ROS evidence interfaces. They are generated only in a ROS build;
  # the standalone core remains free of ROS message/typesupport dependencies.
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ObservedVolume.msg"
    "msg/SourceOccupancyRecord.msg"
    "msg/SourceObservation.msg"
    DEPENDENCIES geometry_msgs std_msgs)

  set(GRID_ROS_DEPENDENCIES
    rclcpp
    rcutils
    rclcpp_action
    builtin_interfaces
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
    dddmr_sys_core
    perception_3d)

  # The action/scheduling library is separate from the node main so other
  # components can link the same contract. The perception adapter is ROS-build
  # only; it converts the immutable perception export into frozen core
  # observations.
  add_library(grid_ros SHARED
    src/grid_global_planner.cpp
    src/offline_pcd_loader.cpp
    src/perception_observation_ingestion.cpp
    src/scene_snapshot_adapter.cpp
    src/static_map_input.cpp
    src/terrain_visualization.cpp)
  target_include_directories(
    grid_ros
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  target_compile_features(grid_ros PUBLIC cxx_std_17)
  target_include_directories(grid_ros PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(
    grid_ros
    PUBLIC
      grid_terrain_core
      ${PCL_LIBRARIES}
      Threads::Threads)
  # Humble's ament_target_dependencies() defaults to the plain
  # target_link_libraries() signature. Request PUBLIC explicitly so it matches
  # the keyword-signature links used for grid_ros above/below.
  ament_target_dependencies(grid_ros PUBLIC
    ${GRID_ROS_DEPENDENCIES})
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(grid_ros
    PUBLIC ${cpp_typesupport_target})

  # Static-PCD-only entry: planner node only, no perception/lidar/preprocessor
  # and no localization. It serves the same /get_plan action. The reusable
  # planner core and perception adapter stay in grid_ros; the retired live
  # two-node main is intentionally not built or installed.
  add_executable(grid_global_planner_node
    src/grid_global_planner_node.cpp)
  target_link_libraries(
    grid_global_planner_node
      grid_ros
      grid_terrain_core)
  ament_target_dependencies(grid_global_planner_node
    ${GRID_ROS_DEPENDENCIES})

  # Live integration entry: exactly one perception_3d_global instance and one
  # grid planner in one process. The static executable above stays perception
  # free; this one is selected only by the navigation grid launch/profile.
  add_executable(grid_global_planner_perception_node
    src/grid_global_planner_perception_node.cpp)
  target_link_libraries(
    grid_global_planner_perception_node
      grid_ros
      grid_terrain_core)
  ament_target_dependencies(grid_global_planner_perception_node
    ${GRID_ROS_DEPENDENCIES})
endif()

if(BUILD_TESTING AND GRID_BUILD_TESTS)
  set(GRID_TEST_NAMES
    test_terrain_types
    test_terrain_grid
    test_terrain_boundary_cost
    test_multiresolution_map
    test_distance_field
    test_wheel_support
    test_grid_search
    test_production_corrections
    test_path_validation
    test_latest_validation
    test_plan_commit_delivery
    test_obstacle_updates
    test_snapshot_concurrency
    test_metrics
    test_offline_geometry
    test_offline_static_map
    test_ground_surface_reconstruction
    test_continuous_ground_surface
    test_global_cloud_surfaces
    test_measurement_evidence
    test_mls_evidence
    test_measurement_capacity
    test_live_perception_overlay)
  if(GRID_STANDALONE)
    find_package(GTest REQUIRED)
    foreach(test_name IN LISTS GRID_TEST_NAMES)
      add_executable(${test_name} test/${test_name}.cpp)
      target_link_libraries(
        ${test_name}
        PRIVATE
          grid_terrain_core
          GTest::gtest
          GTest::gtest_main
          Threads::Threads)
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
    if(GRID_BUILD_BENCHMARKS)
      add_executable(benchmark_snapshot_search
        test/benchmark_snapshot_search.cpp)
      target_link_libraries(benchmark_snapshot_search
        PRIVATE grid_terrain_core Threads::Threads)
      add_test(NAME benchmark_snapshot_search
        COMMAND benchmark_snapshot_search)
    endif()
  else()
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    foreach(test_name IN LISTS GRID_TEST_NAMES)
      ament_add_gtest(${test_name} test/${test_name}.cpp)
      # ament_add_gtest() links the gtest libraries with the plain signature;
      # keep this link plain to avoid mixing signatures on the same target.
      target_link_libraries(${test_name} grid_terrain_core)
    endforeach()
    if(GRID_BUILD_BENCHMARKS)
      # This benchmark is a plain add_executable() target (not created by
      # ament_add_gtest), so the keyword signature is valid here.
      add_executable(benchmark_snapshot_search
        test/benchmark_snapshot_search.cpp)
      target_link_libraries(benchmark_snapshot_search
        PRIVATE grid_terrain_core Threads::Threads)
    endif()
    ament_add_gtest(test_perception_observation_ingestion
      test/test_perception_observation_ingestion.cpp)
    # ament_add_gtest() linked gtest plainly, so both of these must stay plain.
    target_link_libraries(test_perception_observation_ingestion
      grid_ros)
    ament_target_dependencies(test_perception_observation_ingestion
      ${GRID_ROS_DEPENDENCIES})
    ament_add_gtest(test_scene_snapshot_adapter
      test/test_scene_snapshot_adapter.cpp)
    target_link_libraries(test_scene_snapshot_adapter grid_ros)
    ament_target_dependencies(test_scene_snapshot_adapter
      ${GRID_ROS_DEPENDENCIES})
    # The PCD loader lives in grid_ros because it is PCL/ROS-build only.
    ament_add_gtest(test_offline_pcd_loader
      test/test_offline_pcd_loader.cpp)
    target_link_libraries(test_offline_pcd_loader grid_ros)
    ament_target_dependencies(test_offline_pcd_loader
      ${GRID_ROS_DEPENDENCIES})
    ament_add_gtest(test_terrain_visualization
      test/test_terrain_visualization.cpp)
    target_link_libraries(test_terrain_visualization grid_ros)
    ament_target_dependencies(test_terrain_visualization
      ${GRID_ROS_DEPENDENCIES})
    # Static-map input validation is ROS-message-only but sensor_msgs-backed, so
    # it lives in grid_ros rather than the standalone terrain core.
    ament_add_gtest(test_static_map_input
      test/test_static_map_input.cpp)
    target_link_libraries(test_static_map_input grid_ros)
    ament_target_dependencies(test_static_map_input
      ${GRID_ROS_DEPENDENCIES})
    ament_add_gtest(test_request_pose_semantics
      test/test_request_pose_semantics.cpp)
    target_link_libraries(test_request_pose_semantics grid_ros)
    ament_target_dependencies(test_request_pose_semantics
      ${GRID_ROS_DEPENDENCIES})
    ament_add_pytest_test(test_ros_interfaces
      test/test_ros_interfaces.py)
    ament_add_pytest_test(test_rviz_integration
      test/test_rviz_integration.py)
    ament_add_pytest_test(test_rviz_bridge_behavior
      test/test_rviz_bridge_behavior.py)
    ament_add_pytest_test(test_static_launch_wiring
      test/test_static_launch_wiring.py)
    ament_add_pytest_test(test_startup_parameters
      test/test_startup_parameters.py
      ENV PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1
      TIMEOUT 120)
    ament_add_pytest_test(test_install_cleanup_generation
      test/test_install_cleanup_generation.py)
    # CPU-only real-PCD smoke. It is not a ctest: the test must be invoked with
    # an explicit PCD path so a missing robot map never looks like a pass.
    add_executable(offline_static_pcd_smoke
      test/offline_static_pcd_smoke.cpp)
    target_include_directories(offline_static_pcd_smoke PRIVATE
      ${PCL_INCLUDE_DIRS})
    target_link_libraries(offline_static_pcd_smoke
      PRIVATE grid_terrain_core ${PCL_LIBRARIES} Threads::Threads)

    # Registered ROS linters. The installed ROS uncrustify configuration is the
    # authoritative C++ format, so parse headers as C++ rather than letting the
    # tool infer C from the .h suffix. ament_cpplint keeps all of its own
    # default filters; the one exclusion below is required because ROS
    # uncrustify places an opening brace on the following line, which the
    # Google K&R-only readability/braces rule rejects; that is the only filter
    # exclusion. Four private-header build/include_subdir NOLINTs and two narrow
    # per-function readability/fn_size NOLINTs (BuildOfflineStaticMap in
    # offline_static_map.cpp, WheelSupportEvaluator::Evaluate in
    # wheel_support.cpp) are per-line annotations, not filter-wide suppressions.
    # No other check or filter is disabled or muted. See
    # docs/implementation_status.md for the measured fn_size rationale.
    ament_uncrustify(
      TESTNAME uncrustify
      LANGUAGE C++
      .)
    ament_cpplint(
      TESTNAME cpplint
      .
      FILTERS "-readability/braces")
    ament_flake8(
      TESTNAME flake8
      .)
  endif()
endif()

if(NOT GRID_STANDALONE)
  install(TARGETS
    grid_terrain_core
    grid_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(TARGETS
    grid_global_planner_node
    grid_global_planner_perception_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(PROGRAMS scripts/rviz_goal_bridge.py
    DESTINATION lib/${PROJECT_NAME})
  # Keep planner parameters separate from optional offline display helpers.
  # The .rviz file is a display layout asset, not a ROS parameter file.
  install(PROGRAMS launch/path_planning_on_static_layer.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
  install(FILES config/grid_global_planner.yaml
    config/grid_visualization.yaml
    DESTINATION share/${PROJECT_NAME}/config)
  install(FILES rviz/grid_global_planner.rviz
    DESTINATION share/${PROJECT_NAME}/rviz)
  # A previous install may still contain the retired launch/config files.
  # Remove only these exact package-owned names; never delete directories.
  # An old --symlink-install leaves symlinks behind after the source files are
  # deleted. A dangling symlink makes EXISTS false, so check IS_SYMLINK too.
  set(GRID_RETIRED_SHARE_FILES
    launch/dddmr_grid_global_planner.launch.py
    launch/offline_grid_global_planner.launch.py
    launch/p2p_grid_global_planner.launch.py
    launch/__pycache__/dddmr_grid_global_planner.launch.cpython-310.pyc
    launch/__pycache__/offline_grid_global_planner.launch.cpython-310.pyc
    config/dddmr_grid_offline_planner.yaml
    config/dddmr_grid_static_perception.yaml
    config/rviz_goal_bridge.yaml)
  # The old live two-node executable and its package prefix are retired with
  # the package rename; the stale prefix is removed by the workspace owner.
  # Never list the current executable here: an install-time cleanup must not
  # remove the active target.
  # Generate the cleanup as a standalone install-time script instead of
  # install(CODE "..."). ament_cmake's symlink-install wrapper re-expands
  # install() ARGN unquoted, so a list interpolated inside install(CODE) is
  # split at ';' and truncates cmake_install.cmake. install(SCRIPT) passes a
  # single path and is generated correctly in both install modes.
  configure_file(
    cmake/retired_install_cleanup.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/retired_install_cleanup.cmake
    @ONLY)
  install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/retired_install_cleanup.cmake)
  ament_export_include_directories(include)
  ament_export_libraries(grid_terrain_core grid_ros)
  ament_export_dependencies(${GRID_ROS_DEPENDENCIES}
    rosidl_default_runtime)
  ament_package()
endif()
